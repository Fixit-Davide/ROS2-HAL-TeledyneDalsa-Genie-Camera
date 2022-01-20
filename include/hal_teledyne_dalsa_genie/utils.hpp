#include <cstdio>
#include "cordef.h"
#include "GenApi/GenApi.h"				//!< GenApi lib definitions.
#include "GenApi/ChunkAdapterGEV.h"	//!< GenApi lib definitions.
#include "gevapi.h"						//!< GEV lib definitions.

//using namespace std;
//using namespace GenICam;
//using namespace GenApi;

// Set upper limit on chunk data size in case of problems with device implementation
// (Adjust this if needed).
#define MAX_CHUNK_BYTES	256

// For Teledyne DALSA Genie Nano - the metadata layout is known in advance.

#define TELEDYNEDALSA_CHUNK_SIZE_DEVICEID      0x00000010
#define TELEDYNEDALSA_CHUNK_SIZE_DEVICEUSERID  0x00000010

typedef struct _TELEDYNEDALSA_GENIE_NANO_CHUNK
{
  UINT64 Available;
  UINT64 ExposureTime;
  UINT64 CyclingPresetCurrentActiveSet;
  UINT64 LineStatusAll;
  UINT64 AnalogGain;
  UINT64 DigitalGain;
  UINT16 OffsetX;
  UINT16 OffsetY;
  UINT32 CounterValueAtReset;
  UINT16 Width;
  UINT16 Height;
  UINT32 Reserved1;
  UINT64 Timestamp;
  struct 
  {
    UINT8 Horizontal : 4;
    UINT8 Vertcal : 4;
  } Binnings;
  UINT8  Reserved2[7];
  UINT64 TestImageSelector;
  char DeviceID[TELEDYNEDALSA_CHUNK_SIZE_DEVICEID];
  char DeviceUserID[TELEDYNEDALSA_CHUNK_SIZE_DEVICEUSERID];
  UINT64 PixelFormat;
  UINT64 ExposureDelay;
}  TELEDYNEDALSA_GENIE_NANO_CHUNK, *PTELEDYNEDALSA_GENIE_NANO_CHUNK;

typedef struct _TELEDYNEDALSA_CHUNK_INFO
{
	UINT32 chunkIDField;		// Network order field for chunkID
	UINT32 chunkSizeField;	// Network order field for chunk Size
} TELEDYNEDALSA_CHUNK_INFO, *PTELEDYNEDALSA_CHUNK_INFO;

typedef struct _TELEDYNDALSA_CHUNK_CONTAINER 
{
	TELEDYNEDALSA_CHUNK_INFO       imageChunkInfo;
	TELEDYNEDALSA_GENIE_NANO_CHUNK metadata;
	TELEDYNEDALSA_CHUNK_INFO       metadataChunkInfo;
} TELEDYNDALSA_CHUNK_CONTAINER, *PTELEDYNDALSA_CHUNK_CONTAINER;

// Enable/disable Bayer to RGB conversion
// (If disabled - Bayer format will be treated as Monochrome).
#define ENABLE_BAYER_CONVERSION 1

// Enable/disable buffer FULL/EMPTY handling (cycling)
#define USE_SYNCHRONOUS_BUFFER_CYCLING	1


// Enable/disable transfer tuning (buffering, timeouts, thread affinity).
#define TUNE_STREAMING_THREADS 0

void *m_latestBuffer = NULL;

static void _GetUniqueFilename( char *filename, size_t size, char *basename)
{
	// Create a filename based on the current time (to 0.01 seconds)
	struct timeval tm;
	uint32_t years, days, hours, seconds;

	if ((filename != NULL) && (basename != NULL) )
	{
		if (size > (16 + sizeof(basename)) )
		{
	
			// Get the time and turn it into a 10 msec resolution counter to use as an index.
			gettimeofday( &tm, NULL);
			years = ((tm.tv_sec / 86400) / 365);
			tm.tv_sec = tm.tv_sec - (years*86400*365);
			days  = (tm.tv_sec / 86400);
			tm.tv_sec = tm.tv_sec - (days * 86400);
			hours = (tm.tv_sec / 3600);
			seconds = tm.tv_sec - (hours * 3600);						
															
			snprintf(filename, size, "%s_%03d%02d%04d%02d", basename, days,hours, (int)seconds, (int)(tm.tv_usec/10000));
		}
	}
}
