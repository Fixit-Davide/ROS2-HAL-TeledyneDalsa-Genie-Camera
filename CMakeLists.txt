cmake_minimum_required(VERSION 3.5)
project(hal_teledyne_dalsa_genie)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -ggdb -Wno-unknown-pragmas)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
FIND_LIBRARY(GEVAPI_LIBRARY GevApi /usr/local/lib/)
FIND_LIBRARY(GCBASE_LIBRARY GCBase_gcc421_v3_0 /opt/genicam_v3_0/bin/Linux64_x64)

include_directories(include
                    #/usr/local/lib
                    /usr/dalsa/GigeV/include
                    /opt/genicam_v3_0/library/CPP/include/
                    #/usr/lib
                    #/opt/genicam_v3_0/bin/Linux64_x64/
                    /usr/include
)

add_executable(hal_teledyne_dalsa_genie src/hal_teledyne_dalsa_genie.cpp)
target_link_libraries(hal_teledyne_dalsa_genie ${GEVAPI_LIBRARY} ${GCBASE_LIBRARY} -lXext)
ament_target_dependencies(hal_teledyne_dalsa_genie rclcpp rcutils sensor_msgs std_msgs std_srvs rclcpp_action rclcpp_components)

install(TARGETS hal_teledyne_dalsa_genie
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
