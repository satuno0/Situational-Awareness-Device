cmake_minimum_required(VERSION 3.20)
project(sa_daemon LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(vl53l8cx_uld STATIC
    third_party/st_vl53l8cx/vl53l8cx_api.c
    third_party/st_vl53l8cx/vl53l8cx_platform.c
)
target_include_directories(vl53l8cx_uld SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/st_vl53l8cx
)

set(SA_CORE_SOURCES
    src/announcer.cpp
    src/breadcrumb_navigator.cpp
    src/desktop_camera.cpp
    src/emcon_controller.cpp
    src/emcon_hardware.cpp
    src/fusion_engine.cpp
    src/gpio_line.cpp
    src/glove_raycaster.cpp
    src/haptics.cpp
    src/hub_server.cpp
    src/imu_driver.cpp
    src/kalman_track.cpp
    src/lidar_driver.cpp
    src/marker_detector.cpp
    src/marker_driver.cpp
    src/novelda_driver.cpp
    src/occupancy_grid.cpp
    src/playback.cpp
    src/radar_driver.cpp
    src/recorder.cpp
    src/schematic.cpp
    src/sensor_mode.cpp
    src/serial_port.cpp
    src/serialization.cpp
    src/simulation.cpp
    src/slam_manager.cpp
    src/telemetry.cpp
    src/tof_driver.cpp
    src/track_manager.cpp
    src/ultrasonic_driver.cpp
    src/uwb_driver.cpp
    src/wallpad_driver.cpp
    src/wrist_haptics.cpp
)

add_library(sa_core STATIC ${SA_CORE_SOURCES})
target_include_directories(sa_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(sa_core PUBLIC cxx_std_20)
target_link_libraries(sa_core PUBLIC Threads::Threads vl53l8cx_uld)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBGPIOD QUIET IMPORTED_TARGET "libgpiod>=1.5" "libgpiod<2")
        if(LIBGPIOD_FOUND)
            target_compile_definitions(sa_core PRIVATE SA_HAS_LIBGPIOD=1)
            target_link_libraries(sa_core PRIVATE PkgConfig::LIBGPIOD)
        endif()
    endif()
endif()

if(MSVC)
    target_compile_options(sa_core PRIVATE /W4 /WX /permissive-)
else()
    target_compile_options(sa_core PRIVATE -Wall -Wextra -Wpedantic -Werror -pthread)
    target_link_options(sa_core PRIVATE -pthread)
endif()

if(WIN32)
    target_link_libraries(sa_core PUBLIC ws2_32 sapi ole32 mf mfplat mfreadwrite mfuuid)
endif()

add_executable(sa_daemon src/main.cpp)
target_link_libraries(sa_daemon PRIVATE sa_core)

add_executable(core-sa src/main.cpp)
target_link_libraries(core-sa PRIVATE sa_core)

add_executable(sa_tests tests/test_core.cpp)
target_link_libraries(sa_tests PRIVATE sa_core)

add_executable(sa_runtime_tests tests/test_runtime_modules.cpp)
target_link_libraries(sa_runtime_tests PRIVATE sa_core)

add_executable(sa_dxf_tests tests/test_dxf.cpp)
target_link_libraries(sa_dxf_tests PRIVATE sa_core)

enable_testing()
add_test(NAME sa_core_tests COMMAND sa_tests)
add_test(NAME sa_runtime_module_tests COMMAND sa_runtime_tests)
add_test(NAME sa_dxf_parser
         COMMAND sa_dxf_tests "${CMAKE_CURRENT_SOURCE_DIR}/schematics/SA Project_Box Outline.dxf")

# End-to-end check: record a short deterministic simulation, then regenerate
# every tracked/world snapshot from the raw sensor records.
set(SA_CTEST_SESSION "${CMAKE_CURRENT_BINARY_DIR}/ctest_session.bin")
add_test(NAME sa_simulate_record
         COMMAND sa_daemon --simulate --duration 6 --record=${SA_CTEST_SESSION}
                 --no-telemetry --no-haptics)
add_test(NAME sa_replay_verify
         COMMAND sa_daemon --replay=${SA_CTEST_SESSION} --verify
                 --no-telemetry --no-haptics)
set_tests_properties(sa_replay_verify PROPERTIES DEPENDS sa_simulate_record)

install(TARGETS sa_daemon core-sa RUNTIME DESTINATION bin)
install(DIRECTORY web DESTINATION share/sa_daemon)
