cmake_minimum_required(VERSION 3.20)
project(nullCAT VERSION 0.9.3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Qt Configuration
# Set Qt6_DIR or Qt5_DIR to your Qt installation path.
# Example: cmake -DQt6_DIR="C:/Qt/6.6.0/msvc2022_64/lib/cmake/Qt6" ..
# ============================================================
find_package(Qt6 COMPONENTS Core Widgets Network QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 COMPONENTS Core Widgets Network REQUIRED)
    set(QT_VERSION_MAJOR 5)
    message(STATUS "Using Qt5")
else()
    set(QT_VERSION_MAJOR 6)
    message(STATUS "Using Qt6")
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ============================================================
# SOEM Configuration
# Set SOEM_ROOT to your SOEM installation or build directory.
# Example: cmake -DSOEM_ROOT="C:/libs/SOEM" ..
# Expected layout:
#   SOEM_ROOT/include/  -> ethercat.h, etc.
#   SOEM_ROOT/lib/      -> soem.lib (or soem_static.lib)
# ============================================================
set(SOEM_ROOT "" CACHE PATH "Path to SOEM root directory")

if(SOEM_ROOT STREQUAL "")
    message(WARNING "SOEM_ROOT not set. Attempting to find SOEM automatically.")
    find_path(SOEM_INCLUDE_DIR soem.h
        PATHS
            "C:/libs/SOEM/include"
            "C:/SOEM/include"
            "$ENV{SOEM_ROOT}/include"
        DOC "SOEM include directory"
    )
    find_library(SOEM_LIBRARY
        NAMES soem soem_static
        PATHS
            "C:/libs/SOEM/lib"
            "C:/SOEM/lib"
            "$ENV{SOEM_ROOT}/lib"
        DOC "SOEM library"
    )
else()
    set(SOEM_INCLUDE_DIR "${SOEM_ROOT}/include")
    find_library(SOEM_LIBRARY
        NAMES soem soem_static
        PATHS "${SOEM_ROOT}/lib"
    )
endif()

if(NOT SOEM_INCLUDE_DIR OR NOT SOEM_LIBRARY)
    message(FATAL_ERROR
        "SOEM not found. Please set SOEM_ROOT to your SOEM directory.\n"
        "  cmake -DSOEM_ROOT=C:/libs/SOEM ..\n"
        "See BUILD_INSTRUCTIONS.md for details."
    )
endif()

# ============================================================
# Npcap SDK (Windows) - SOEM's raw-socket transport.
# NPCAP_SDK: root of the extracted SDK (Include/pcap.h + Lib/x64/wpcap.lib).
# Override on the command line (CI does: -DNPCAP_SDK=C:/npcap-sdk).
#
# The Include directory is required at COMPILE time, not just link time:
# SOEM's nicdrv.h does #include <pcap.h> on Windows, so every target that
# pulls in a SOEM header needs it. Declared here, before any target is
# created, so include_directories() reaches all of them.
# ============================================================
if(WIN32)
    set(NPCAP_SDK "C:/libs/npcap-sdk" CACHE PATH "Path to the Npcap SDK root")
    if(NOT EXISTS "${NPCAP_SDK}/Lib/x64/wpcap.lib" OR NOT EXISTS "${NPCAP_SDK}/Include/pcap.h")
        message(FATAL_ERROR
            "Npcap SDK not found at ${NPCAP_SDK} "
            "(expected Include/pcap.h and Lib/x64/wpcap.lib).\n"
            "  cmake -DNPCAP_SDK=C:/path/to/npcap-sdk ..\n"
            "Download: https://npcap.com/#download (SDK zip).")
    endif()
    include_directories("${NPCAP_SDK}/Include")
    message(STATUS "Npcap SDK:    ${NPCAP_SDK}")
endif()

message(STATUS "SOEM include: ${SOEM_INCLUDE_DIR}")
message(STATUS "SOEM library: ${SOEM_LIBRARY}")

# ============================================================
# Source Files
# ============================================================
set(SOURCES
    src/main.cpp
    src/MainWindow.cpp
    src/ApplicationSettingsDialog.h
    src/ApplicationSettingsDialog.cpp
    src/EtherCATMaster.cpp
    src/SdoWorker.cpp
    src/A6Drive.cpp
    src/TelemetryInput.cpp
    src/MotionController.cpp
    src/CommissioningMode.cpp
    src/ControlLoop.cpp
    src/StatusModel.cpp
    src/DriveProvisioner.cpp
    src/DriveFaultMonitor.cpp
    src/Config.cpp
    src/Logging.cpp
    src/WebServer.cpp
    src/ForegroundKeeper.cpp
    src/DirectInputButtons.cpp
)

set(HEADERS
    src/MainWindow.h
    src/EtherCATMaster.h
    src/SdoWorker.h
    src/A6Drive.h
    src/TelemetryInput.h
    src/MotionController.h
    src/ControlLoop.h
    src/HomingSequence.h
    src/WkcMonitor.h
    src/Config.h
    src/Logging.h
    src/WebServer.h
    src/ForegroundKeeper.h
    src/httplib.h
)

# ============================================================
# Executable
# ============================================================
if(WIN32)
    add_executable(nullCAT WIN32 ${SOURCES} ${HEADERS})
else()
    add_executable(nullCAT ${SOURCES} ${HEADERS})
endif()

# ============================================================
# Include Directories
# ============================================================
target_include_directories(nullCAT PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${SOEM_INCLUDE_DIR}
    # SOEM often has platform-specific subdirectories
    ${SOEM_INCLUDE_DIR}/soem
    ${SOEM_INCLUDE_DIR}/osal
    ${SOEM_INCLUDE_DIR}/oshw
)

# ============================================================
# Link Libraries
# ============================================================
if(QT_VERSION_MAJOR EQUAL 6)
    target_link_libraries(nullCAT PRIVATE
        Qt6::Core
        Qt6::Widgets
        Qt6::Network
        ${SOEM_LIBRARY}
    )
else()
    target_link_libraries(nullCAT PRIVATE
        Qt5::Core
        Qt5::Widgets
        Qt5::Network
        ${SOEM_LIBRARY}
    )
endif()

# Windows-specific system libraries required by SOEM.
# (NPCAP_SDK is defined + validated above, before any target exists, because
# its Include dir is needed at compile time by every SOEM-consuming target.)
if(WIN32)
    target_link_libraries(nullCAT PRIVATE
        ws2_32      # Winsock
    winmm       # Multimedia timer
    avrt        # Multimedia Class Scheduler (AvSetMmThreadCharacteristics)
    Iphlpapi    # IP Helper API (used by SOEM for NIC enumeration)
    dinput8     # DirectInput (USB game-controller button binding backend)
    dxguid      # DirectInput GUIDs (IID_IDirectInput8A, c_dfDIJoystick2)
    "${NPCAP_SDK}/Lib/x64/wpcap.lib"
    "${NPCAP_SDK}/Lib/x64/Packet.lib"
    )
    # Request administrator privileges (required for raw socket access by SOEM)
    set_target_properties(nullCAT PROPERTIES
        LINK_FLAGS "/MANIFESTUAC:\"level='requireAdministrator' uiAccess='false'\""
    )
endif()

# ============================================================
# Compiler Options
# ============================================================
if(MSVC)
    target_compile_options(nullCAT PRIVATE
        /W3
        /MP     # Multi-processor compilation
        /Zc:__cplusplus
    )
    # Suppress common SOEM/Winsock deprecation warnings
    target_compile_definitions(nullCAT PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _WINSOCK_DEPRECATED_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        SOEM_AVAILABLE   # Tells A6Drive.cpp to compile SDO write calls
    )
endif()

# This build links the Qt native config UI (MainWindow / ApplicationSettingsDialog),
# so it OWNS host.json (host config edited natively, not in the browser). A future
# headless build simply omits this flag, ceding host ownership to the web UI.
# Keyed off UI presence, not the OS, so a headless Windows NUC reports "web".
target_compile_definitions(nullCAT PRIVATE HAS_QT_CONFIG=1)

# ============================================================
# Bundle the MSVC runtime app-local so a zipped release runs on a CLEAN machine
# without the VC++ redistributable installed. (Npcap is still a separate
# prerequisite -- it is a kernel driver and can't be shipped in the folder.)
# ============================================================
if(MSVC)
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP ON)   # populate the list; don't add an install() rule
    include(InstallRequiredSystemLibraries)
    foreach(_runtime_lib ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS})
        add_custom_command(TARGET nullCAT POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_runtime_lib}" "$<TARGET_FILE_DIR:nullCAT>"
            COMMENT "Bundling MSVC runtime: ${_runtime_lib}")
    endforeach()
endif()

# (No config.json is shipped: the app reads host.json + rig.json next to the
# executable, and a legacy flat config.json is only ever a migration INPUT.
# Deploying one would invite editing a file the app ignores.)

# Slice 2: copy the split reference files (host.reference.json + rig.reference.json)
# -- comprehensive copy-paste reference of every host / rig field at its default --
# next to the live host.json / rig.json, available without pulling the repo on the
# test machine. See Docs/CONFIG_REFERENCE.md for descriptions and units.
add_custom_command(TARGET nullCAT POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/resources/host.reference.json"
        "$<TARGET_FILE_DIR:nullCAT>/host.reference.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/resources/rig.reference.json"
        "$<TARGET_FILE_DIR:nullCAT>/rig.reference.json"
    COMMENT "Copying host/rig reference files to output directory"
)

# License + safety texts belong in every distributed bundle (GPL binary
# distribution must carry them; SAFETY.md is non-negotiable for a rig product).
add_custom_command(TARGET nullCAT POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE"
        "$<TARGET_FILE_DIR:nullCAT>/LICENSE.txt"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/THIRD_PARTY_NOTICES.md"
        "$<TARGET_FILE_DIR:nullCAT>/THIRD_PARTY_NOTICES.md"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/SAFETY.md"
        "$<TARGET_FILE_DIR:nullCAT>/SAFETY.md"
    COMMENT "Copying license/safety texts to output directory"
)

# Create logs directory in output
add_custom_command(TARGET nullCAT POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "$<TARGET_FILE_DIR:nullCAT>/logs"
    COMMENT "Creating logs directory"
)

# Copy web/ UI files to output directory
add_custom_command(TARGET nullCAT POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/web"
        "$<TARGET_FILE_DIR:nullCAT>/web"
    COMMENT "Copying web UI to output directory"
)

# ============================================================
# Watchdog launcher (Windows only)
# Monitors nullCAT.exe and relaunches on crash/restart.
# Runs under the Task Scheduler admin task - no UAC prompt.
# ============================================================
if(WIN32)
    add_executable(nullCATWatchdog WIN32 src/Watchdog.cpp)
    set_target_properties(nullCATWatchdog PROPERTIES
        LINK_FLAGS "/MANIFESTUAC:\"level='requireAdministrator' uiAccess='false'\""
    )
    # Copy watchdog alongside the main exe after build
    add_custom_command(TARGET nullCAT POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:nullCATWatchdog>"
            "$<TARGET_FILE_DIR:nullCAT>/nullCATWatchdog.exe"
        DEPENDS nullCATWatchdog
        COMMENT "Copying watchdog to output directory"
    )
endif()

# Ship the deploy artifacts in the release folder (README + appliance scripts +
# the NUC host.json template) so a zipped Release is a complete handoff.
foreach(_deploy_file README.txt install-task.ps1 uninstall-task.ps1 host.nuc.example.json)
    add_custom_command(TARGET nullCAT POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/deploy/${_deploy_file}"
            "$<TARGET_FILE_DIR:nullCAT>/${_deploy_file}"
        COMMENT "Copying deploy/${_deploy_file} to output directory")
endforeach()

message(STATUS "")
message(STATUS "=== nullCAT Build Configuration ===")
message(STATUS "  Qt version:    ${QT_VERSION_MAJOR}")
message(STATUS "  SOEM include:  ${SOEM_INCLUDE_DIR}")
message(STATUS "  SOEM library:  ${SOEM_LIBRARY}")
message(STATUS "==========================================")
message(STATUS "")

# ============================================================
# Test Targets (Phase 2 - P2-1 through P2-4)
#
# Test binaries link Qt + MockA6Drive + MotionController only.
# They do NOT link SOEM or npcap (no hardware required).
# SOEM_AVAILABLE is intentionally NOT defined so A6Drive.cpp
# compiles without SOEM headers.
#
# Run tests:
#   cd build && ctest -C Release --output-on-failure
#   -- or --
#   build\Release\TestHomingSequence.exe -v2
#   build\Release\TestMotionController.exe -v2
# ============================================================

enable_testing()

# Shared sources for all test binaries (no EtherCATMaster, ControlLoop, UI)
set(TEST_CORE_SOURCES
    src/A6Drive.cpp
    src/MotionController.cpp
    src/CommissioningMode.cpp
    src/Config.cpp
    src/Logging.cpp
    src/DriveFaultMonitor.cpp
)

# Common includes for test targets
set(TEST_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Common compile definitions for test targets
set(TEST_COMPILE_DEFS
    _CRT_SECURE_NO_WARNINGS
    _WINSOCK_DEPRECATED_NO_WARNINGS
    WIN32_LEAN_AND_MEAN
    NOMINMAX
    # SOEM_AVAILABLE intentionally omitted -- tests run without hardware
)

# Common Qt link libraries for tests
if(QT_VERSION_MAJOR EQUAL 6)
    set(TEST_QT_LIBS Qt6::Core Qt6::Test)
else()
    set(TEST_QT_LIBS Qt5::Core Qt5::Test)
endif()

# Locate Qt bin directory so ctest can find the Qt DLLs on Windows.
# Uses the imported location of Qt::Core to derive the bin path.
if(WIN32)
    if(QT_VERSION_MAJOR EQUAL 6)
        get_target_property(_qt_core_loc Qt6::Core IMPORTED_LOCATION_RELEASE)
        if(NOT _qt_core_loc)
            get_target_property(_qt_core_loc Qt6::Core IMPORTED_LOCATION_DEBUG)
        endif()
    else()
        get_target_property(_qt_core_loc Qt5::Core IMPORTED_LOCATION_RELEASE)
        if(NOT _qt_core_loc)
            get_target_property(_qt_core_loc Qt5::Core IMPORTED_LOCATION_DEBUG)
        endif()
    endif()
    if(_qt_core_loc)
        cmake_path(GET _qt_core_loc PARENT_PATH _qt_bin_dir)
        set(TEST_PATH_ENV "PATH=${_qt_bin_dir};$ENV{PATH}")
        message(STATUS "Test Qt bin: ${_qt_bin_dir}")
    else()
        set(TEST_PATH_ENV "")
    endif()
endif()

# Helper macro to add a test executable
macro(add_motion_test TARGET_NAME SOURCE_FILE)
    if(QT_VERSION_MAJOR EQUAL 6)
        find_package(Qt6 COMPONENTS Test QUIET)
    else()
        find_package(Qt5 COMPONENTS Test QUIET)
    endif()

    add_executable(${TARGET_NAME} ${SOURCE_FILE} ${TEST_CORE_SOURCES})
    target_include_directories(${TARGET_NAME} PRIVATE ${TEST_INCLUDE_DIRS})
    target_compile_definitions(${TARGET_NAME} PRIVATE ${TEST_COMPILE_DEFS})
    target_compile_options(${TARGET_NAME} PRIVATE /W3 /MP /Zc:__cplusplus)
    target_link_libraries(${TARGET_NAME} PRIVATE ${TEST_QT_LIBS} ws2_32 winmm)
    add_test(NAME ${TARGET_NAME} COMMAND ${TARGET_NAME} -v2)
    if(WIN32 AND TEST_PATH_ENV)
        set_tests_properties(${TARGET_NAME} PROPERTIES
            TIMEOUT 120
            ENVIRONMENT "${TEST_PATH_ENV}")
    else()
        set_tests_properties(${TARGET_NAME} PROPERTIES TIMEOUT 120)
    endif()
endmacro()

# ---- P2-2: HomingSequence unit tests ----
add_motion_test(TestHomingSequence tests/TestHomingSequence.cpp)

# ---- Deinit seat-mode regression (was Pi-only; PC-3) -- the seat core
# compiles into the PC binary, so it runs here too ----
add_motion_test(TestHomingSeat tests/TestHomingSeat.cpp)

# ---- P2-3: MotionController state machine tests ----
add_motion_test(TestMotionController tests/TestMotionController.cpp)

# ---- P2-4: Fault recovery integration tests ----
add_motion_test(TestFaultRecovery tests/TestFaultRecovery.cpp)

# ---- Build 63: TF-1 DriveFaultMonitor unit tests ----
add_motion_test(TestDriveFaultMonitor tests/TestDriveFaultMonitor.cpp)

# ---- Build 63: TF-4 Trajectory planner tests ----
add_motion_test(TestTrajectory tests/TestTrajectory.cpp)

# ---- Build 65: P4-2 Config validation tests ----
add_motion_test(TestConfigValidation tests/TestConfigValidation.cpp)

# ---- Slice 2: host/rig two-file config split ----
add_motion_test(TestConfigTwoFile tests/TestConfigTwoFile.cpp)

# ---- Build 222D: RT-safe logging tests ----
add_motion_test(TestRTLogging tests/TestRTLogging.cpp)

# ---- A6 fault decode tables (0x203F panel codes / 0x603F candidates) ----
add_motion_test(TestFaultCodes tests/TestFaultCodes.cpp)

# ---- Commissioning excitation engine + identification metrics ----
add_motion_test(TestCommissioning tests/TestCommissioning.cpp)

# ---- Build 72: TestInitSequence - EtherCATMaster init sequence (sim mode) ----
#
# Unlike the other test targets, this one links EtherCATMaster.cpp which
# requires SOEM + npcap at link time.  All tests run in simulation mode
# so no hardware or network adapter is needed at runtime.
if(WIN32 AND SOEM_LIBRARY)
    if(QT_VERSION_MAJOR EQUAL 6)
        find_package(Qt6 COMPONENTS Test QUIET)
    else()
        find_package(Qt5 COMPONENTS Test QUIET)
    endif()

    set(INIT_TEST_SOURCES
        tests/TestInitSequence.cpp
        src/EtherCATMaster.cpp
        src/SdoWorker.cpp
        src/A6Drive.cpp
        src/Config.cpp
        src/Logging.cpp
    )

    add_executable(TestInitSequence ${INIT_TEST_SOURCES})
    target_include_directories(TestInitSequence PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${SOEM_INCLUDE_DIR}
        ${SOEM_INCLUDE_DIR}/soem
        ${SOEM_INCLUDE_DIR}/osal
        ${SOEM_INCLUDE_DIR}/oshw
    )
    target_compile_definitions(TestInitSequence PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _WINSOCK_DEPRECATED_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        SOEM_AVAILABLE
    )
    target_compile_options(TestInitSequence PRIVATE /W3 /MP /Zc:__cplusplus)
    target_link_libraries(TestInitSequence PRIVATE
        ${TEST_QT_LIBS}
        ${SOEM_LIBRARY}
        ws2_32
        winmm
        avrt
        Iphlpapi
        "${NPCAP_SDK}/Lib/x64/wpcap.lib"
        "${NPCAP_SDK}/Lib/x64/Packet.lib"
    )
    add_test(NAME TestInitSequence COMMAND TestInitSequence -v2)
    if(TEST_PATH_ENV)
        set_tests_properties(TestInitSequence PROPERTIES
            TIMEOUT 60
            ENVIRONMENT "${TEST_PATH_ENV}")
    else()
        set_tests_properties(TestInitSequence PROPERTIES TIMEOUT 60)
    endif()

    # ---- B222 Phase 3: TestPumpLifecycle - dispatch thread lifecycle (sim mode) ----
    set(PUMP_TEST_SOURCES
        tests/TestPumpLifecycle.cpp
        src/EtherCATMaster.cpp
        src/SdoWorker.cpp
        src/A6Drive.cpp
        src/Config.cpp
        src/Logging.cpp
    )

    add_executable(TestPumpLifecycle ${PUMP_TEST_SOURCES})
    target_include_directories(TestPumpLifecycle PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${SOEM_INCLUDE_DIR}
        ${SOEM_INCLUDE_DIR}/soem
        ${SOEM_INCLUDE_DIR}/osal
        ${SOEM_INCLUDE_DIR}/oshw
    )
    target_compile_definitions(TestPumpLifecycle PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _WINSOCK_DEPRECATED_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        SOEM_AVAILABLE
    )
    target_compile_options(TestPumpLifecycle PRIVATE /W3 /MP /Zc:__cplusplus)
    target_link_libraries(TestPumpLifecycle PRIVATE
        ${TEST_QT_LIBS}
        ${SOEM_LIBRARY}
        ws2_32
        winmm
        avrt
        Iphlpapi
        "${NPCAP_SDK}/Lib/x64/wpcap.lib"
        "${NPCAP_SDK}/Lib/x64/Packet.lib"
    )
    add_test(NAME TestPumpLifecycle COMMAND TestPumpLifecycle -v2)
    if(TEST_PATH_ENV)
        set_tests_properties(TestPumpLifecycle PROPERTIES
            TIMEOUT 60
            ENVIRONMENT "${TEST_PATH_ENV}")
    else()
        set_tests_properties(TestPumpLifecycle PROPERTIES TIMEOUT 60)
    endif()

    # ---- B222E: TestB222E - threading audit fixes C1/C3 (sim mode) ----
    set(B222E_TEST_SOURCES
        tests/TestB222E.cpp
        src/EtherCATMaster.cpp
        src/SdoWorker.cpp
        src/A6Drive.cpp
        src/Config.cpp
        src/Logging.cpp
    )

    add_executable(TestB222E ${B222E_TEST_SOURCES})
    target_include_directories(TestB222E PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${SOEM_INCLUDE_DIR}
        ${SOEM_INCLUDE_DIR}/soem
        ${SOEM_INCLUDE_DIR}/osal
        ${SOEM_INCLUDE_DIR}/oshw
    )
    target_compile_definitions(TestB222E PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _WINSOCK_DEPRECATED_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        SOEM_AVAILABLE
    )
    target_compile_options(TestB222E PRIVATE /W3 /MP /Zc:__cplusplus)
    target_link_libraries(TestB222E PRIVATE
        ${TEST_QT_LIBS}
        ${SOEM_LIBRARY}
        ws2_32
        winmm
        avrt
        Iphlpapi
        "${NPCAP_SDK}/Lib/x64/wpcap.lib"
        "${NPCAP_SDK}/Lib/x64/Packet.lib"
    )
    add_test(NAME TestB222E COMMAND TestB222E -v2)
    if(TEST_PATH_ENV)
        set_tests_properties(TestB222E PROPERTIES
            TIMEOUT 60
            ENVIRONMENT "${TEST_PATH_ENV}")
    else()
        set_tests_properties(TestB222E PROPERTIES TIMEOUT 60)
    endif()

    # ---- B222F: TestB222F - command sync, WKC math, limits, diag logs (sim mode) ----
    set(B222F_TEST_SOURCES
        tests/TestB222F.cpp
        src/EtherCATMaster.cpp
        src/SdoWorker.cpp
        src/A6Drive.cpp
        src/Config.cpp
        src/Logging.cpp
        src/MotionController.cpp
        src/CommissioningMode.cpp
        src/DriveFaultMonitor.cpp
    )

    add_executable(TestB222F ${B222F_TEST_SOURCES})
    target_include_directories(TestB222F PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${SOEM_INCLUDE_DIR}
        ${SOEM_INCLUDE_DIR}/soem
        ${SOEM_INCLUDE_DIR}/osal
        ${SOEM_INCLUDE_DIR}/oshw
    )
    target_compile_definitions(TestB222F PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _WINSOCK_DEPRECATED_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        SOEM_AVAILABLE
    )
    target_compile_options(TestB222F PRIVATE /W3 /MP /Zc:__cplusplus)
    target_link_libraries(TestB222F PRIVATE
        ${TEST_QT_LIBS}
        ${SOEM_LIBRARY}
        ws2_32
        winmm
        avrt
        Iphlpapi
        "${NPCAP_SDK}/Lib/x64/wpcap.lib"
        "${NPCAP_SDK}/Lib/x64/Packet.lib"
    )
    add_test(NAME TestB222F COMMAND TestB222F -v2)
    if(TEST_PATH_ENV)
        set_tests_properties(TestB222F PROPERTIES
            TIMEOUT 60
            ENVIRONMENT "${TEST_PATH_ENV}")
    else()
        set_tests_properties(TestB222F PROPERTIES TIMEOUT 60)
    endif()

    # ---- B222G: TestB222G - Npcap drain regression, Logger flush, drainElist cap ----
    set(B222G_TEST_SOURCES
        tests/TestB222G.cpp
        src/EtherCATMaster.cpp
        src/SdoWorker.cpp
        src/A6Drive.cpp
        src/Config.cpp
        src/Logging.cpp
        src/MotionController.cpp
        src/CommissioningMode.cpp
        src/DriveFaultMonitor.cpp
    )

    add_executable(TestB222G ${B222G_TEST_SOURCES})
    target_include_directories(TestB222G PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${SOEM_INCLUDE_DIR}
        ${SOEM_INCLUDE_DIR}/soem
        ${SOEM_INCLUDE_DIR}/osal
        ${SOEM_INCLUDE_DIR}/oshw
    )
    target_compile_definitions(TestB222G PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _WINSOCK_DEPRECATED_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        SOEM_AVAILABLE
    )
    target_compile_options(TestB222G PRIVATE /W3 /MP /Zc:__cplusplus)
    target_link_libraries(TestB222G PRIVATE
        ${TEST_QT_LIBS}
        ${SOEM_LIBRARY}
        ws2_32
        winmm
        avrt
        Iphlpapi
        "${NPCAP_SDK}/Lib/x64/wpcap.lib"
        "${NPCAP_SDK}/Lib/x64/Packet.lib"
    )
    add_test(NAME TestB222G COMMAND TestB222G -v2)
    if(TEST_PATH_ENV)
        set_tests_properties(TestB222G PROPERTIES
            TIMEOUT 60
            ENVIRONMENT "${TEST_PATH_ENV}")
    else()
        set_tests_properties(TestB222G PROPERTIES TIMEOUT 60)
    endif()

    # ---- B222H: TestB222H - ALStatusCode pre-check, mbx_rl DIAG, ecx_err2string, RTT tracking ----
    set(B222H_TEST_SOURCES
        tests/TestB222H.cpp
        src/EtherCATMaster.cpp
        src/SdoWorker.cpp
        src/A6Drive.cpp
        src/Config.cpp
        src/Logging.cpp
        src/MotionController.cpp
        src/CommissioningMode.cpp
        src/DriveFaultMonitor.cpp
        src/ControlLoop.cpp
        src/TelemetryInput.cpp
    )

    add_executable(TestB222H ${B222H_TEST_SOURCES})
    target_include_directories(TestB222H PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${SOEM_INCLUDE_DIR}
        ${SOEM_INCLUDE_DIR}/soem
        ${SOEM_INCLUDE_DIR}/osal
        ${SOEM_INCLUDE_DIR}/oshw
    )
    target_compile_definitions(TestB222H PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _WINSOCK_DEPRECATED_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        SOEM_AVAILABLE
    )
    target_compile_options(TestB222H PRIVATE /W3 /MP /Zc:__cplusplus)
    target_link_libraries(TestB222H PRIVATE
        ${TEST_QT_LIBS}
        ${SOEM_LIBRARY}
        ws2_32
        winmm
        avrt
        Iphlpapi
        "${NPCAP_SDK}/Lib/x64/wpcap.lib"
        "${NPCAP_SDK}/Lib/x64/Packet.lib"
    )
    add_test(NAME TestB222H COMMAND TestB222H -v2)
    if(TEST_PATH_ENV)
        set_tests_properties(TestB222H PROPERTIES
            TIMEOUT 120
            ENVIRONMENT "${TEST_PATH_ENV}")
    else()
        set_tests_properties(TestB222H PROPERTIES TIMEOUT 120)
    endif()

    # ---- B222I: TestB222I - ABI fix, drainElist corruption guard,
    #              A4 cyclic mailbox, A5 recovery thread ----
    set(B222I_TEST_SOURCES
        tests/TestB222I.cpp
        src/EtherCATMaster.cpp
        src/SdoWorker.cpp
        src/A6Drive.cpp
        src/Config.cpp
        src/Logging.cpp
        src/MotionController.cpp
        src/CommissioningMode.cpp
        src/DriveFaultMonitor.cpp
        src/ControlLoop.cpp
        src/TelemetryInput.cpp
    )

    add_executable(TestB222I ${B222I_TEST_SOURCES})
    target_include_directories(TestB222I PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${SOEM_INCLUDE_DIR}
        ${SOEM_INCLUDE_DIR}/soem
        ${SOEM_INCLUDE_DIR}/osal
        ${SOEM_INCLUDE_DIR}/oshw
    )
    target_compile_definitions(TestB222I PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _WINSOCK_DEPRECATED_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        SOEM_AVAILABLE
    )
    target_compile_options(TestB222I PRIVATE /W3 /MP /Zc:__cplusplus)
    target_link_libraries(TestB222I PRIVATE
        ${TEST_QT_LIBS}
        ${SOEM_LIBRARY}
        ws2_32
        winmm
        avrt
        Iphlpapi
        "${NPCAP_SDK}/Lib/x64/wpcap.lib"
        "${NPCAP_SDK}/Lib/x64/Packet.lib"
    )
    add_test(NAME TestB222I COMMAND TestB222I -v2)
    if(TEST_PATH_ENV)
        set_tests_properties(TestB222I PROPERTIES
            TIMEOUT 120
            ENVIRONMENT "${TEST_PATH_ENV}")
    else()
        set_tests_properties(TestB222I PROPERTIES TIMEOUT 120)
    endif()

    # ---- B222J: TestB222J - DS402 fault history readback, mailbox overrun
    #              counter in diag, A4 ordering fix (verified by absence of
    #              wkc=0 SDO warnings in hardware logs) ----
    set(B222J_TEST_SOURCES
        tests/TestB222J.cpp
        src/EtherCATMaster.cpp
        src/SdoWorker.cpp
        src/A6Drive.cpp
        src/Config.cpp
        src/Logging.cpp
        src/MotionController.cpp
        src/CommissioningMode.cpp
        src/DriveFaultMonitor.cpp
        src/ControlLoop.cpp
        src/TelemetryInput.cpp
    )

    add_executable(TestB222J ${B222J_TEST_SOURCES})
    target_include_directories(TestB222J PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${SOEM_INCLUDE_DIR}
        ${SOEM_INCLUDE_DIR}/soem
        ${SOEM_INCLUDE_DIR}/osal
        ${SOEM_INCLUDE_DIR}/oshw
    )
    target_compile_definitions(TestB222J PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _WINSOCK_DEPRECATED_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        SOEM_AVAILABLE
    )
    target_compile_options(TestB222J PRIVATE /W3 /MP /Zc:__cplusplus)
    target_link_libraries(TestB222J PRIVATE
        ${TEST_QT_LIBS}
        ${SOEM_LIBRARY}
        ws2_32
        winmm
        avrt
        Iphlpapi
        "${NPCAP_SDK}/Lib/x64/wpcap.lib"
        "${NPCAP_SDK}/Lib/x64/Packet.lib"
    )
    add_test(NAME TestB222J COMMAND TestB222J -v2)
    if(TEST_PATH_ENV)
        set_tests_properties(TestB222J PROPERTIES
            TIMEOUT 120
            ENVIRONMENT "${TEST_PATH_ENV}")
    else()
        set_tests_properties(TestB222J PROPERTIES TIMEOUT 120)
    endif()

    # ---- B222K: TestB222K - ControlLoop start/stop/start lifecycle,
    #              re-init mailbox handler reset, fault-history state gating ----
    set(B222K_TEST_SOURCES
        tests/TestB222K.cpp
        src/EtherCATMaster.cpp
        src/SdoWorker.cpp
        src/A6Drive.cpp
        src/Config.cpp
        src/Logging.cpp
        src/MotionController.cpp
        src/CommissioningMode.cpp
        src/DriveFaultMonitor.cpp
        src/ControlLoop.cpp
        src/TelemetryInput.cpp
    )

    add_executable(TestB222K ${B222K_TEST_SOURCES})
    target_include_directories(TestB222K PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${SOEM_INCLUDE_DIR}
        ${SOEM_INCLUDE_DIR}/soem
        ${SOEM_INCLUDE_DIR}/osal
        ${SOEM_INCLUDE_DIR}/oshw
    )
    target_compile_definitions(TestB222K PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _WINSOCK_DEPRECATED_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        SOEM_AVAILABLE
    )
    target_compile_options(TestB222K PRIVATE /W3 /MP /Zc:__cplusplus)
    target_link_libraries(TestB222K PRIVATE
        ${TEST_QT_LIBS}
        ${SOEM_LIBRARY}
        ws2_32
        winmm
        avrt
        Iphlpapi
        "${NPCAP_SDK}/Lib/x64/wpcap.lib"
        "${NPCAP_SDK}/Lib/x64/Packet.lib"
    )
    add_test(NAME TestB222K COMMAND TestB222K -v2)
    if(TEST_PATH_ENV)
        set_tests_properties(TestB222K PROPERTIES
            TIMEOUT 120
            ENVIRONMENT "${TEST_PATH_ENV}")
    else()
        set_tests_properties(TestB222K PROPERTIES TIMEOUT 120)
    endif()
endif()

# ---- TestLogControls: Logger parseLevel + diagEnabled gate + minLevel
#              suppression. No SOEM required - pure Logger unit tests.
add_motion_test(TestLogControls tests/TestLogControls.cpp)

# ---- B222M: TestB222M - ForegroundKeeper construction, flags, opacity, size.
#              Requires Qt::Widgets (for QWidget); not covered by add_motion_test
#              which links Qt::Core+Test only.
if(QT_VERSION_MAJOR EQUAL 6)
    find_package(Qt6 COMPONENTS Widgets Test QUIET)
else()
    find_package(Qt5 COMPONENTS Widgets Test QUIET)
endif()

add_executable(TestB222M
    tests/TestB222M.cpp
    src/ForegroundKeeper.cpp
    src/Logging.cpp
)
target_include_directories(TestB222M PRIVATE ${TEST_INCLUDE_DIRS})
target_compile_definitions(TestB222M PRIVATE ${TEST_COMPILE_DEFS})
target_compile_options(TestB222M PRIVATE /W3 /MP /Zc:__cplusplus)
if(QT_VERSION_MAJOR EQUAL 6)
    target_link_libraries(TestB222M PRIVATE Qt6::Core Qt6::Widgets Qt6::Test ws2_32 winmm)
else()
    target_link_libraries(TestB222M PRIVATE Qt5::Core Qt5::Widgets Qt5::Test ws2_32 winmm)
endif()
add_test(NAME TestB222M COMMAND TestB222M -v2)
if(WIN32 AND TEST_PATH_ENV)
    set_tests_properties(TestB222M PROPERTIES
        TIMEOUT 60
        ENVIRONMENT "${TEST_PATH_ENV}")
else()
    set_tests_properties(TestB222M PROPERTIES TIMEOUT 60)
endif()

# (TestB222N retired: its merge-on-save preservation cases were folded into
#  tests/TestConfigTwoFile.cpp - the two-file config contract's single home - #  and its legacy-migration case died with the migration path.)

# ---- B222O: TestB222O -- canonical save/load round-trip + scaling collapse.
#              Pulls Config + A6Drive + Logging. Qt::Core only.
add_executable(TestB222O
    tests/TestB222O.cpp
    src/Config.cpp
    src/A6Drive.cpp
    src/Logging.cpp
)
target_include_directories(TestB222O PRIVATE ${TEST_INCLUDE_DIRS})
target_compile_definitions(TestB222O PRIVATE ${TEST_COMPILE_DEFS})
target_compile_options(TestB222O PRIVATE /W3 /MP /Zc:__cplusplus)
if(QT_VERSION_MAJOR EQUAL 6)
    target_link_libraries(TestB222O PRIVATE Qt6::Core Qt6::Test ws2_32 winmm)
else()
    target_link_libraries(TestB222O PRIVATE Qt5::Core Qt5::Test ws2_32 winmm)
endif()
add_test(NAME TestB222O COMMAND TestB222O -v2)
if(WIN32 AND TEST_PATH_ENV)
    set_tests_properties(TestB222O PROPERTIES
        TIMEOUT 60
        ENVIRONMENT "${TEST_PATH_ENV}")
else()
    set_tests_properties(TestB222O PROPERTIES TIMEOUT 60)
endif()

# ============================================================
# Delay-load Npcap in the test binaries
#
# The SOEM-linking test targets import wpcap.dll / Packet.dll at LOAD time,
# which requires the Npcap *runtime driver* to be installed -- not just the
# SDK. A machine with only the SDK (any CI runner) cannot even start these
# processes: the Windows loader raises a hard error, which in a
# non-interactive session hangs instead of exiting, so the tests appear to
# time out rather than fail.
#
# These tests all run in simulation mode and never call into pcap, so
# delay-loading is sufficient: the DLLs are resolved on first use, which
# never happens. That keeps the full suite runnable anywhere -- these are the
# EtherCAT init / pump / lifecycle tests, and they are root-CMake only, so
# excluding them from CI would leave them with no automated coverage at all.
#
# nullCAT.exe itself is deliberately NOT delay-loaded: for the end user a
# missing Npcap should fail immediately and unmistakably at launch (the
# documented "wpcap.dll was not found"), rather than starting and dying later
# when the NIC is opened.
# ============================================================
if(MSVC)
    set(_npcap_delayload_tests
        TestInitSequence TestPumpLifecycle
        TestB222E TestB222F TestB222G TestB222H TestB222I TestB222J TestB222K)
    foreach(_t IN LISTS _npcap_delayload_tests)
        if(TARGET ${_t})
            target_link_options(${_t} PRIVATE
                /DELAYLOAD:wpcap.dll /DELAYLOAD:Packet.dll)
            target_link_libraries(${_t} PRIVATE delayimp)
        else()
            message(WARNING "delay-load list names a missing target: ${_t}")
        endif()
    endforeach()
endif()
