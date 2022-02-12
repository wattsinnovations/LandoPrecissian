cmake_minimum_required(VERSION 3.5)
project(lando)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wno-address-of-packed-member)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)

# Determine git version
exec_program(
    "git"
    ${CMAKE_CURRENT_SOURCE_DIR}
    ARGS "describe"
    OUTPUT_VARIABLE GIT_TAG
 )

#...if not found get the SHA1 hash instead
if(${GIT_TAG} MATCHES "fatal:.*")
    exec_program(
        "git"
        ${CMAKE_CURRENT_SOURCE_DIR}
        ARGS "rev-parse HEAD"
        OUTPUT_VARIABLE GIT_VERSION
    )
else()
    set(GIT_VERSION ${GIT_TAG})
endif()

add_definitions( -DAPP_GIT_VERSION="${GIT_VERSION}")

include_directories(${CMAKE_SOURCE_DIR}/libraries/c_library_v2/auterion)
include_directories(${CMAKE_SOURCE_DIR}/src/Mavlink)
include_directories(${CMAKE_SOURCE_DIR}/src/misc)

add_executable(app
	src/main.cpp
	src/misc/global_include.cpp
    src/Mavlink/Mavlink.cpp
    src/Mavlink/udp_connection.cpp
)
ament_target_dependencies(app rclcpp std_msgs ros2_aruco_interfaces)

install(TARGETS app
	DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
