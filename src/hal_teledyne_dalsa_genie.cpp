#include "stdio.h"

#include <cstdio>
#include <cinttypes>
#include <cassert>
#include <chrono>

#include <iostream>
#include <string>
#include <functional>
#include <cstdlib>
#include <memory>

#include "cordef.h"
#include "GenApi/GenApi.h"
#include "GenApi/ChunkAdapterGEV.h"
#include "gevapi.h"

#include "hal_teledyne_dalsa_genie/utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;
namespace ph = std::placeholders;

#define MAX_CAMERAS 10
#define NUM_BUF	10

class HALGenieCam : public rclcpp::Node {
 public:

	HALGenieCam() : Node("hal_teledyne_dalsa_genie") {
		hz = declare_parameter<float>("hz", 100.0);
		model = declare_parameter("camera_model", "auto");
		camera_link = declare_parameter("frames.camera_link", "camera_link");
		debugging_mode = declare_parameter<bool>("debugging_mode", false);
		framerate = declare_parameter<int>("framerate", 100);
		exposure_time = declare_parameter<float>("exposure_time", 6e+04);
		auto_brightness = declare_parameter<bool>("auto_brightness", false);

		GevApiInitialize();

		GevGetLibraryConfigOptions( &options);
		//options.logLevel = GEV_LOG_LEVEL_OFF;
		//options.logLevel = GEV_LOG_LEVEL_TRACE;
		options.logLevel = GEV_LOG_LEVEL_WARNINGS;
		GevSetLibraryConfigOptions( &options);

		height = 0;
		width = 0;
		format = 0;
		numBuffers = NUM_BUF;
		pixFormat = 0;
		pixDepth = 0;
		convertedGevFormat = 0;

		is_init = false;
		first_display = true;
  	}
  
  	bool init() {
    	status = GevGetCameraList( pCamera, MAX_CAMERAS, &numCamera);

		if (status != 0 or numCamera == 0){
			RCLCPP_ERROR(get_logger(), "No camera found.");
			return false;
		}

		int found_idx;
		bool found = false;

		for(int idx = 0; idx < numCamera; idx ++){
			std::cout << "Found: " << pCamera[idx].model << std::endl;
			if(model.compare(pCamera[idx].model) == 0){
				found = true;
				found_idx = idx;
			}
			else if(model.compare("auto") == 0){
				found = true;
				found_idx = idx;
				break;
			}
		}

		if(found == false){
			if(model.compare("auto") == 0)
				std::cerr << "No Genie Camera connected!" << std::endl;
			else
				std::cerr << "No Genie Camera model " << model << " connected!" << std::endl;
			return false;
		}

		std::cout << "Genie " << pCamera[found_idx].model << " found with id " << found_idx << std::endl;

		status = GevOpenCamera( &pCamera[found_idx], GevExclusiveMode, &handle);
		if (status != 0){
			RCLCPP_ERROR(get_logger(), "Error opening the camera.");
			return false;
		}

		GevGetCameraInterfaceOptions( handle, &camOptions);
		//camOptions.streamFrame_timeout_ms = 1001;				// Internal timeout for frame reception.
		camOptions.streamNumFramesBuffered = 10;					// Buffer frames internally.
		//camOptions.streamMemoryLimitMax = 64*1024*1024;			// Adjust packet memory buffering limit.	
		//camOptions.streamPktSize = 9000;						// Adjust the GVSP packet size.
		//camOptions.streamPktDelay = 10;							// Add usecs between packets to pace arrival at NIC.
		GevSetCameraInterfaceOptions( handle, &camOptions);

		//=====================================================================
		// Get the GenICam FeatureNodeMap object and access the camera features.
		Camera = static_cast<GenApi::CNodeMapRef*>(GevGetFeatureNodeMap(handle));
		if (Camera){
			// Access some features using the bare GenApi interface methods
			try {
				GenApi::CIntegerPtr ptrIntNode = Camera->_GetNode("Width");
				width = (UINT32) ptrIntNode->GetValue();
				ptrIntNode = Camera->_GetNode("Height");
				height = (UINT32) ptrIntNode->GetValue();
				ptrIntNode = Camera->_GetNode("PayloadSize");
				payload_size = (UINT64) ptrIntNode->GetValue();
				GenApi::CEnumerationPtr ptrEnumNode = Camera->_GetNode("PixelFormat") ;
				format = (UINT32)ptrEnumNode->GetIntValue();
			}
			CATCH_GENAPI_ERROR(status);
		} else {
			std::cerr << "Error accessing Genie Camera" << std::endl;
			return false;
		}

		//GevSetFeatureValueAsString(handle, "ChunkModeActive", "0");
		//GevSetFeatureValueAsString(handle, "chunkCompatibilityMode", "GenAPI");		
		GevSetFeatureValueAsString(handle, "OffsetX", "0");	
		GevSetFeatureValueAsString(handle, "OffsetY", "0");	
		//GevSetFeatureValueAsString(handle, "GevSCPSPacketSize", "9000");	
		GevSetFeatureValueAsString(handle, "acquisitionFrameRateControlMode", "Programmable");	
		GevSetFeatureValueAsString(handle, "AcquisitionFrameRate", std::to_string(framerate).c_str());	
		GevSetFeatureValueAsString(handle, "ExposureTime", std::to_string(exposure_time).c_str());

		if(auto_brightness){	
			GevSetFeatureValueAsString(handle, "autoBrightnessMode", "Active");	
			GevSetFeatureValueAsString(handle, "autoBrightnessTarget", "100");			
			GevSetFeatureValueAsString(handle, "autoBrightnessAlgoConvergenceTime", "2");	
		}
		else{
			GevSetFeatureValueAsString(handle, "autoBrightnessMode", "Off");
		}				

		size = payload_size;
		
		for (i = 0; i < numBuffers; i++){
			bufAddress[i] = (PUINT8)malloc(size);
		}

		// Initialize a transfer with synchronous buffer handling.
		status = GevInitializeTransfer( handle, SynchronousNextEmpty, size, numBuffers, bufAddress);

		if (status == GEVLIB_ERROR_INVALID_HANDLE)
			std::cerr << "GevInitializeTransfer - status: GEVLIB_ERROR_INVALID_HANDLE" << std::endl;
		else if(status == GEVLIB_ERROR_ARG_INVALID)
			std::cerr << "GevInitializeTransfer - status: GEVLIB_ERROR_ARG_INVALID" << std::endl;
		else if(status == GEVLIB_ERROR_SOFTWARE)
			std::cerr << "GevInitializeTransfer - status: GEVLIB_ERROR_SOFTWARE" << std::endl;
		
		if (status != GEVLIB_OK){
			std::cerr << "Error initializing transfer with Genie Camera" << std::endl;
			return false;
		}

		for (i = 0; i < numBuffers; i++){
			memset(bufAddress[i], 0, size);
		}

	 	status = GevStartTransfer(handle, -1);

		if(status == GEVLIB_ERROR_INVALID_HANDLE)
			std::cerr << "GevStartTransfer - status: GEVLIB_ERROR_INVALID_HANDLE" << std::endl;
		else if(status == GEVLIB_ERROR_INVALID_HANDLE)
			std::cerr << "GevStartTransfer - status: GEVLIB_ERROR_INVALID_HANDLE" << std::endl;
		else if(status == GEV_STATUS_BUSY)
			std::cerr << "GevStartTransfer - status: GEV_STATUS_BUSY" << std::endl;
		else if(status == GEVLIB_ERROR_XFER_NOT_INITIALIZED)
			std::cerr << "GevStartTransfer - status: GEVLIB_ERROR_XFER_NOT_INITIALIZED" << std::endl;
		else if(status == GEVLIB_ERROR_XFER_ACTIVE)
			std::cerr << "GevStartTransfer - status: GEVLIB_ERROR_XFER_ACTIVE" << std::endl;


		if(status == GEVLIB_OK){
			is_init = true;
			cam_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 1);
			//timer_ = this->create_wall_timer(1000ms/hz, std::bind(&HALGenieCam::timer_callback, this));
			while(true){
				timer_callback();
			}
			return true;
		}
		return false;
  	}

	~HALGenieCam(){
		disconnect();
	}
  
	void disconnect(){
		
			if(is_init){
				this->close_connection();
			}

			_CloseSocketAPI ();	// must close API even on error
	}

	bool get_image(){

		if (not is_init){
			std::cerr << "Cam not initialized" << std::endl;
			return false;
		}

		// Wait for images to be received
		status_gev = GevWaitForNextFrame(handle, &img, 2000);

		if(status_gev == GEVLIB_ERROR_INVALID_HANDLE)
			std::cerr << "GevWaitForNextFrame - status: GEVLIB_ERROR_INVALID_HANDLE" << std::endl;
		else if(status_gev == GEVLIB_ERROR_TIME_OUT)
			std::cerr << "GevWaitForNextFrame - status: GEVLIB_ERROR_TIME_OUT" << std::endl;
		else if(status_gev == GEVLIB_ERROR_NULL_PTR)
			std::cerr << "GevWaitForNextFrame - status: GEVLIB_ERROR_NULL_PTR" << std::endl;

		if(status_gev != GEVLIB_OK)
			return false;

		// See if there is metadata appended to the frame.
		if (img->chunk_data != NULL){
			std::cerr << "Chunk Data Received" << std::endl;
			return false;
		}
		
		return true;
	}

	bool get_mono_frame(std::shared_ptr<std::vector<uint8_t>> buf){
		if (not get_image()){
			std::cerr << "Error acquiring image" << std::endl;
			GevReleaseImage(handle, img);
			return false;
		}

		uint8_t* pBuf = NULL;
        pBuf = (uint8_t*) img->address;

		for(unsigned int i = 0; i < img->w * img->h; i++){
			buf->push_back(pBuf[i]);
		}

		status = GevReleaseImage(handle, img);

		if(status == GEVLIB_ERROR_INVALID_HANDLE)
			std::cerr << "GevReleaseImage - status: GEVLIB_ERROR_INVALID_HANDLE" << std::endl;
		else if(status == GEVLIB_ERROR_PARAMETER_INVALID)
			std::cerr << "GevReleaseImage - status: GEVLIB_ERROR_PARAMETER_INVALID" << std::endl;
		else if(status == GEVLIB_ERROR_ARG_INVALID)
			std::cerr << "GevReleaseImage - status: GEVLIB_ERROR_ARG_INVALID" << std::endl;

		if(status != GEVLIB_OK)
			return false;

		return true;

	}

	void close_connection(){
		if(not is_init){
			return;
		}

		GevAbortTransfer(handle);
		status = GevFreeTransfer(handle);

		// Disable metadate
		GevSetFeatureValueAsString(handle, "ChunkModeActive", "0");

		for (i = 0; i < numBuffers; i++)
		{	
			free(bufAddress[i]);
		}
		
		GevCloseCamera(&handle);

		// Close down the API.
		GevApiUninitialize();

		is_init = false;
	}


    void timer_callback(){
		auto buf = std::make_shared<std::vector<uint8_t>>();

		if(get_mono_frame(buf))
		{
			auto message = std::make_unique<sensor_msgs::msg::Image>();

			message->header.stamp = get_clock()->now();
			message->header.frame_id = camera_link;

			message->height = this->height;
			message->width = this->width;

			message->encoding = "mono8";

			message->is_bigendian = 0;

			message->step = this->width;

			message->data = std::move(*(buf.get()));

			cam_publisher_->publish(std::move(message));
		}
    }


 private:

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_publisher_;
  	size_t count_;

  	GEV_DEVICE_INTERFACE pCamera[MAX_CAMERAS];
	GEV_STATUS status;
	int numCamera;

	GEVLIB_CONFIG_OPTIONS options;

	std::string model;
	std::string camera_link;
	bool debugging_mode;
	int framerate=100;
	float exposure_time=6e+04;
	bool auto_brightness=false;
	float hz;

	int i;
	UINT32 height;
	UINT32 width;
	UINT32 format;
	UINT64 size;
	UINT64 payload_size;
	int numBuffers;
	PUINT8 bufAddress[NUM_BUF];
	GEV_CAMERA_HANDLE handle;
	UINT32 pixFormat;
	UINT32 pixDepth;
	UINT32 convertedGevFormat;

	GEV_CAMERA_OPTIONS camOptions;
	GenApi::CNodeMapRef *Camera;

	int extra_lines;

	GEV_BUFFER_OBJECT *img;
	GEV_STATUS status_gev;

	// Metadata appended to frame.
	// Handle the chunk decoding (meta-data).
	BYTE*   lpPointer = NULL;
	INT64   i64Offset = 0;
	UINT32  ui32ChunkLength = 0;
	UINT32  ui32ChunkID = 0;

	TELEDYNDALSA_CHUNK_CONTAINER *chunkContainer;

	bool is_init;
	bool first_display;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<HALGenieCam>();
    if (not node->init()) rclcpp::shutdown();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
