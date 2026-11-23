cmake_minimum_required(VERSION 3.20)
project(qdevice_ximu3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(XIMU3_SOFTWARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../x-IMU3-Software" CACHE PATH "Path to x-IMU3-Software repository")
set(QDEVICE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/qdevice_sdk" CACHE PATH "Path to Qualisys QDevice SDK")

# Core buffering & resampling library (platform-independent, zero external deps)
add_library(ximu3_core STATIC
    src/ImuSampleBuffer.cpp
)
target_include_directories(ximu3_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# Unit tests for core buffering, quaternion NLERP resampling, and sync events
enable_testing()
add_executable(ximu3_bridge_tests tests/test_sample_buffer.cpp)
target_link_libraries(ximu3_bridge_tests PRIVATE ximu3_core)
add_test(NAME ximu3_bridge_tests COMMAND ximu3_bridge_tests)

# Locate compiled x-IMU3 Rust static library (ximu3.lib on Windows, libximu3.a on Linux/macOS)
find_library(XIMU3_RUST_LIB
    NAMES ximu3.lib ximu3 libximu3.a
    PATHS "${XIMU3_SOFTWARE_DIR}/x-IMU3-API/Rust/target/release"
    NO_DEFAULT_PATH
)

if (XIMU3_RUST_LIB)
    add_library(ximu3_bridge STATIC
        src/ImuDeviceManager.cpp
        src/QDeviceAdapter.cpp
        "${XIMU3_SOFTWARE_DIR}/x-IMU3-API/Cpp/ConnectionConfig.cpp"
    )
    target_include_directories(ximu3_bridge PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${XIMU3_SOFTWARE_DIR}/x-IMU3-API/Cpp"
        "${XIMU3_SOFTWARE_DIR}/x-IMU3-API/C"
    )
    target_link_libraries(ximu3_bridge PUBLIC ximu3_core "${XIMU3_RUST_LIB}")

    if (UNIX AND NOT APPLE)
        target_link_libraries(ximu3_bridge PUBLIC pthread dl m udev)
    endif ()

    add_executable(ximu3_qdevice_cli src/cli_main.cpp)
    target_link_libraries(ximu3_qdevice_cli PRIVATE ximu3_bridge)

    if (WIN32 AND EXISTS "${QDEVICE_SDK_DIR}/api/qualisys/qdevice-api/qdevice_api.h")
        add_library(qdevice_ximu3 SHARED src/QDevicePlugin.cpp)
        target_include_directories(qdevice_ximu3 PRIVATE "${QDEVICE_SDK_DIR}/api")
        target_link_libraries(qdevice_ximu3 PRIVATE ximu3_bridge)
    endif ()
else ()
    message(STATUS "x-IMU3 Rust library not found at ${XIMU3_SOFTWARE_DIR}/x-IMU3-API/Rust/target/release; building core + tests only.")
endif ()
