# ============================================================
# pi/CMakeLists.txt - NullCAT-Pi headless Linux build
#
# Builds the EtherCAT motion core as a headless daemon (web UI as the
# control surface). Reuses the shared ../src sources unchanged; the only
# Pi-specific source is main_headless.cpp here.
#
# Deliberately isolated from the top-level (Windows/Qt-GUI) CMakeLists.txt
# so the PC build is never touched. Build:
#   cmake -S pi -B pi/build && cmake --build pi/build -j2
# Keep -j2 on the Pi: 2 schedulable cores + 2 GB RAM, and higher
# parallelism can swap-thrash the box off the network.
# ============================================================
cmake_minimum_required(VERSION 3.20)
project(nullcat_pi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/../src)

find_package(Threads REQUIRED)

# Qt6 - TRANSITIONAL dependency. Only Config.cpp uses it (Qt JSON). To be
# dropped once Config moves to nlohmann/json; the shipping binary is then
# fully Qt-free. Core component only (no Widgets, no Network).
find_package(Qt6 COMPONENTS Core REQUIRED)

# SOEM - headers AND library MUST come from the same pinned tree so the
# generated ec_options.h (which sizes ecx_contextt) matches libsoem.a.
# Mixing headers and library from different trees silently changes the
# context-struct layout (ABI mismatch); one pinned tree rules that out
# by construction.
set(SOEM_ROOT "$ENV{HOME}/SOEM" CACHE PATH "SOEM source+build tree")
find_library(SOEM_LIBRARY NAMES soem PATHS "${SOEM_ROOT}/build" REQUIRED)
message(STATUS "SOEM library: ${SOEM_LIBRARY}")

add_executable(nullcat-pi
    ${CMAKE_CURRENT_SOURCE_DIR}/main_headless.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/EvdevButtons.cpp
    ${SRC}/EtherCATMaster.cpp
    ${SRC}/SdoWorker.cpp
    ${SRC}/A6Drive.cpp
    ${SRC}/TelemetryInput.cpp
    ${SRC}/MotionController.cpp
    ${SRC}/CommissioningMode.cpp
    ${SRC}/ControlLoop.cpp
    ${SRC}/StatusModel.cpp
    ${SRC}/DriveProvisioner.cpp
    ${SRC}/DriveFaultMonitor.cpp
    ${SRC}/Config.cpp
    ${SRC}/Logging.cpp
    ${SRC}/WebServer.cpp
)

target_include_directories(nullcat-pi PRIVATE
    ${SRC}
    ${SOEM_ROOT}/include
    ${SOEM_ROOT}/build/include   # generated ec_options.h
    ${SOEM_ROOT}/osal            # osal.h (pulled in by soem/ec_type.h)
    ${SOEM_ROOT}/osal/linux
    ${SOEM_ROOT}/oshw/linux      # nicdrv.h (Linux raw-socket NIC layer)
)

# SOEM_AVAILABLE gates A6Drive's real SDO/PDO calls (the shared sources
# also compile without SOEM for hardware-free targets).
target_compile_definitions(nullcat-pi PRIVATE SOEM_AVAILABLE)

target_link_libraries(nullcat-pi PRIVATE
    Qt6::Core
    ${SOEM_LIBRARY}
    Threads::Threads
)

# libgpiod (v2) - OPTIONAL physical control panel. find_library wants the .so
# dev symlink (libgpiod-dev), so a host without it just builds without the
# panel. When present, compile GpioPanel.cpp and define HAVE_LIBGPIOD so
# main_headless wires it in.
# GpioPanel.cpp uses the v2 API ONLY; a v1 library (e.g. Ubuntu <= 24.04's
# libgpiod-dev) cannot compile it. Probe a v2-only symbol so v1 hosts take
# the clean panel-disabled path instead of failing the build.
find_library(GPIOD_LIB gpiod)
find_path(GPIOD_INC gpiod.h)
if(GPIOD_LIB AND GPIOD_INC)
    include(CheckLibraryExists)
    get_filename_component(GPIOD_LIBDIR ${GPIOD_LIB} DIRECTORY)
    check_library_exists(gpiod gpiod_line_settings_new ${GPIOD_LIBDIR} GPIOD_IS_V2)
endif()
if(GPIOD_LIB AND GPIOD_INC AND GPIOD_IS_V2)
    message(STATUS "libgpiod v2 found (${GPIOD_LIB}) - building GPIO control panel")
    target_sources(nullcat-pi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/GpioPanel.cpp)
    target_include_directories(nullcat-pi PRIVATE ${GPIOD_INC})
    target_link_libraries(nullcat-pi PRIVATE ${GPIOD_LIB})
    target_compile_definitions(nullcat-pi PRIVATE HAVE_LIBGPIOD=1)
elseif(GPIOD_LIB AND GPIOD_INC)
    message(WARNING "libgpiod found but it is v1 (no gpiod_line_settings_new) - "
        "GPIO panel disabled. GpioPanel needs libgpiod >= 2.0.")
else()
    message(WARNING "libgpiod not found - GPIO panel disabled. Install libgpiod-dev (v2) to enable it.")
endif()

# Stage the web UI + a logs/ dir next to the binary.
add_custom_command(TARGET nullcat-pi POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:nullcat-pi>/logs
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/../web $<TARGET_FILE_DIR:nullcat-pi>/web
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/../Docs/drive_profiles $<TARGET_FILE_DIR:nullcat-pi>/drive_profiles
    COMMENT "Staging web/, logs/, drive_profiles/ next to nullcat-pi"
)

# Standalone drive-provisioning tool. Run with the master service STOPPED (it owns
# the NIC), like slaveinfo. Brings the bus to SafeOp (no OP/loop) and runs the
# DriveProvisioner sequence. Reuses the same core sources as nullcat-pi (minus
# main_headless + WebServer).
add_executable(provision
    ${CMAKE_CURRENT_SOURCE_DIR}/provision_main.cpp
    ${SRC}/EtherCATMaster.cpp
    ${SRC}/SdoWorker.cpp
    ${SRC}/A6Drive.cpp
    ${SRC}/TelemetryInput.cpp
    ${SRC}/MotionController.cpp
    ${SRC}/CommissioningMode.cpp
    ${SRC}/ControlLoop.cpp
    ${SRC}/StatusModel.cpp
    ${SRC}/DriveProvisioner.cpp
    ${SRC}/DriveFaultMonitor.cpp
    ${SRC}/Config.cpp
    ${SRC}/Logging.cpp
)
target_include_directories(provision PRIVATE
    ${SRC}
    ${SOEM_ROOT}/include
    ${SOEM_ROOT}/build/include
    ${SOEM_ROOT}/osal
    ${SOEM_ROOT}/osal/linux
    ${SOEM_ROOT}/oshw/linux
)
target_compile_definitions(provision PRIVATE SOEM_AVAILABLE)
target_link_libraries(provision PRIVATE Qt6::Core ${SOEM_LIBRARY} Threads::Threads)
add_custom_command(TARGET provision POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:provision>/logs
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/../Docs/drive_profiles $<TARGET_FILE_DIR:provision>/drive_profiles
    COMMENT "Staging drive_profiles/ next to provision"
)

# Regression test: the deinit "seat" mode of HomingSequence must be purely ADDITIVE to
# normal homing (byte-identical search, diverging only at the hardstop) and must establish
# NO home reference. Driven through MockA6Drive -- no SOEM, no NIC, no Qt -- so it builds
# and runs on the Pi. Built WITHOUT SOEM_AVAILABLE (it only needs the non-hardware paths).
# Run with: ctest -R HomingSeat   (or run the binary directly).
enable_testing()
add_executable(TestHomingSeat
    ${CMAKE_CURRENT_SOURCE_DIR}/../tests/TestHomingSeat.cpp
    ${SRC}/A6Drive.cpp
    ${SRC}/Logging.cpp)
target_include_directories(TestHomingSeat PRIVATE ${SRC})
target_link_libraries(TestHomingSeat PRIVATE Threads::Threads)
add_test(NAME TestHomingSeat COMMAND TestHomingSeat)

# Shared status model unit test: proves the canonical
# derivation reproduces the current web behaviour, retains raw state for later
# un-folding, computes the aggregate by precedence, and that the colour/blink table
# is a single deterministic source. Pure derivation -- no SOEM / NIC / Qt / web.
# Run with: ctest -R StatusModel
add_executable(TestStatusModel
    ${CMAKE_CURRENT_SOURCE_DIR}/../tests/TestStatusModel.cpp
    ${SRC}/StatusModel.cpp)
target_include_directories(TestStatusModel PRIVATE ${SRC})
target_link_libraries(TestStatusModel PRIVATE Threads::Threads)
add_test(NAME TestStatusModel COMMAND TestStatusModel)

# Belt / torque-mode safety invariants. Drives MotionController
# through process() with no drive object -- the belt torque command is controller-
# computed -- so it needs no SOEM/NIC/Qt. Asserts the torque path's invariants
# (raw->command mapping + clamp, config min/max honoured, e-stop -> 0, park -> 0) so
# the belt-blend refactor and any future edit cannot silently regress them.
# Run with: ctest -R TorquePath
add_executable(TestTorquePath
    ${CMAKE_CURRENT_SOURCE_DIR}/../tests/TestTorquePath.cpp
    ${SRC}/MotionController.cpp
    ${SRC}/CommissioningMode.cpp
    ${SRC}/A6Drive.cpp
    ${SRC}/Logging.cpp)
target_include_directories(TestTorquePath PRIVATE ${SRC})
target_link_libraries(TestTorquePath PRIVATE Threads::Threads)
add_test(NAME TestTorquePath COMMAND TestTorquePath)

# Pins the telemetry UDP wire-format semantics: any parser rewrite (e.g. a
# zero-allocation fast path) must pass it unchanged.
# Run with: ctest -R TelemetryParse
add_executable(TestTelemetryParse
    ${CMAKE_CURRENT_SOURCE_DIR}/../tests/TestTelemetryParse.cpp
    ${SRC}/TelemetryInput.cpp
    ${SRC}/Logging.cpp)
target_include_directories(TestTelemetryParse PRIVATE ${SRC})
target_link_libraries(TestTelemetryParse PRIVATE Threads::Threads)
add_test(NAME TestTelemetryParse COMMAND TestTelemetryParse)

# Pins the WKC monitor semantics the recovery stack depends on:
# time-based trigger threshold (configure -> max(3, hz*0.015)), signal
# fires exactly at the threshold cycle, inRecovery latches (one signal per
# sustained loss), good-frame clears, reset(). Header-only pure logic --
# needs Logging.cpp for the RT_LOG ring, nothing else.
# Run with: ctest -R WkcMonitor
add_executable(TestWkcMonitor
    ${CMAKE_CURRENT_SOURCE_DIR}/../tests/TestWkcMonitor.cpp
    ${SRC}/Logging.cpp)
target_include_directories(TestWkcMonitor PRIVATE ${SRC})
target_link_libraries(TestWkcMonitor PRIVATE Threads::Threads)
add_test(NAME TestWkcMonitor COMMAND TestWkcMonitor)

# Pins Docs/COMMAND_CONTRACT.md at the engine layer: every enqueued
# MotionCommand's promised observable state change, including the silent
# engine-layer refusals (TensionBelts under e-stop). Commands go through
# enqueueCommand() exactly as the WebServer does -- never handler calls.
# Run with: ctest -R CommandContract
add_executable(TestCommandContract
    ${CMAKE_CURRENT_SOURCE_DIR}/../tests/TestCommandContract.cpp
    ${SRC}/MotionController.cpp
    ${SRC}/CommissioningMode.cpp
    ${SRC}/A6Drive.cpp
    ${SRC}/Logging.cpp)
target_include_directories(TestCommandContract PRIVATE ${SRC})
target_link_libraries(TestCommandContract PRIVATE Threads::Threads)
add_test(NAME TestCommandContract COMMAND TestCommandContract)

# Pins Docs/COMMAND_CONTRACT.md at the HTTP layer: real POSTs against the real
# WebServer routes on the full simulation-mode stack (assembled exactly as
# main_headless.cpp does). This is where the escaped-bug class lives: an
# endpoint string that maps to the wrong engine action passes every
# engine-layer test and fails only here. Needs SOEM + Qt like the main binary
# (sim mode at runtime, full link at build time). Run with: ctest -R HttpContract
add_executable(TestHttpContract
    ${CMAKE_CURRENT_SOURCE_DIR}/../tests/TestHttpContract.cpp
    ${SRC}/EtherCATMaster.cpp
    ${SRC}/SdoWorker.cpp
    ${SRC}/A6Drive.cpp
    ${SRC}/TelemetryInput.cpp
    ${SRC}/MotionController.cpp
    ${SRC}/CommissioningMode.cpp
    ${SRC}/ControlLoop.cpp
    ${SRC}/StatusModel.cpp
    ${SRC}/DriveProvisioner.cpp
    ${SRC}/DriveFaultMonitor.cpp
    ${SRC}/Config.cpp
    ${SRC}/Logging.cpp
    ${SRC}/WebServer.cpp)
target_include_directories(TestHttpContract PRIVATE
    ${SRC}
    ${SOEM_ROOT}/include
    ${SOEM_ROOT}/build/include
    ${SOEM_ROOT}/osal
    ${SOEM_ROOT}/osal/linux
    ${SOEM_ROOT}/oshw/linux)
target_compile_definitions(TestHttpContract PRIVATE SOEM_AVAILABLE)
target_link_libraries(TestHttpContract PRIVATE Qt6::Core ${SOEM_LIBRARY} Threads::Threads)
add_test(NAME TestHttpContract COMMAND TestHttpContract)
set_tests_properties(TestHttpContract PROPERTIES TIMEOUT 120)

# ============================================================
# Extended (PC-era) LOGIC test suites, wired for cloud CI. Default OFF so
# the Pi pre-push hook stays fast; the CI workflow configures with
# -DNULLCAT_FULL_TESTS=ON, adding push-time coverage on trunk (a trunk
# break is found by the pusher, not at the next merge). The Windows build
# also runs these suites at every parity merge. The remaining PC suites
# are deliberately not ported here: they depend on WIN32+SOEM/Npcap
# threading harnesses or on Qt Widgets / Windows linkage.
# ============================================================
option(NULLCAT_FULL_TESTS "Build the extended (PC-era) logic test suites" OFF)
if(NULLCAT_FULL_TESTS)
    find_package(Qt6 REQUIRED COMPONENTS Test)
    set(FULLTEST_CORE
        ${SRC}/A6Drive.cpp
        ${SRC}/MotionController.cpp
        ${SRC}/CommissioningMode.cpp
        ${SRC}/Config.cpp
        ${SRC}/Logging.cpp
        ${SRC}/DriveFaultMonitor.cpp)
    macro(add_logic_test NAME)
        add_executable(${NAME}
            ${CMAKE_CURRENT_SOURCE_DIR}/../tests/${NAME}.cpp
            ${FULLTEST_CORE})
        target_include_directories(${NAME} PRIVATE ${SRC})
        # SOEM_AVAILABLE deliberately omitted -- these run without hardware
        # (mirrors the PC build's TEST_COMPILE_DEFS).
        set_target_properties(${NAME} PROPERTIES AUTOMOC ON)
        target_link_libraries(${NAME} PRIVATE Qt6::Core Qt6::Test Threads::Threads)
        add_test(NAME ${NAME} COMMAND ${NAME})
        set_tests_properties(${NAME} PROPERTIES TIMEOUT 120)
    endmacro()
    foreach(T TestHomingSequence TestMotionController TestFaultRecovery
              TestDriveFaultMonitor TestTrajectory TestConfigValidation
              TestConfigTwoFile TestRTLogging TestLogControls TestTrackingFilter
              TestAxisKind TestFaultCodes TestCommissioning
              TestCommissioningFlow TestB222O)
        add_logic_test(${T})
    endforeach()

    # TestTrackingFilter's Regression() case requires bit-identical FP: with
    # fused multiply-add the exact-ZOH comparison drifts and the zero-tolerance
    # check flakes (see the REQUIRED note at the top of the test). GCC/Clang
    # flag; this tree never sees MSVC.
    target_compile_options(TestTrackingFilter PRIVATE -ffp-contract=off)
endif()
