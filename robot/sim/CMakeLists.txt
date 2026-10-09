# Software-in-the-loop simulator: the real firmware in ../src, compiled for the host against a
# simulated MCU, as the Python extension module `bucky_fw`.
#
#   python -m bucky.firmware.build      (from simulation/backend; the usual way)
#   cmake -S robot/sim -B robot/sim/build/h562rg-RelWithDebInfo -G Ninja \
#         -Dpybind11_DIR=$(python -c 'import pybind11; print(pybind11.get_cmake_dir())')
cmake_minimum_required(VERSION 3.20)
project(bucky_sim LANGUAGES C CXX)

set(BUCKY_BOARD "h562rg" CACHE STRING "Board to simulate (h562rg)")
set(BUCKY_FW_SANITIZE "" CACHE STRING "Sanitizers for a debug build, e.g. address,undefined")
option(BUCKY_FW_FAST_MATH "Compile firmware sources with -ffast-math, like the target" ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS ON)        # gnu++17, the PlatformIO default
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

get_filename_component(FIRMWARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src" ABSOLUTE)
set(SIM_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

if(BUCKY_BOARD STREQUAL "h562rg")
    set(BOARD_DEFS STM32H5xx STM32H562xx)
else()
    message(FATAL_ERROR "unsupported BUCKY_BOARD=${BUCKY_BOARD}")
endif()

include(cmake/gen_programs.cmake)

find_package(Python 3.11 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

# Include order matters: shadow/ (empty escape hatch) and the simulated vendor SDK come before
# the firmware tree, so <Arduino.h>, <stm32h5xx.h>, <PinNames.h>, … resolve to the simulator.
set(SIM_INCLUDES
    "${SIM_DIR}/shadow"
    "${SIM_DIR}/boards/${BUCKY_BOARD}"
    "${SIM_DIR}/include"
    "${SIM_DIR}/vendor/stm32duino"
    "${FIRMWARE_DIR}"
    "${SIM_DIR}/src"
    "${CMAKE_BINARY_DIR}/generated")

set(WARN -Wall -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable)

# ---- the firmware, unmodified ----------------------------------------------------------------
file(GLOB_RECURSE FW_TESTS CONFIGURE_DEPENDS "${FIRMWARE_DIR}/tests/*.cpp")
set(FW_SOURCES
    "${FIRMWARE_DIR}/main.cpp"
    "${FIRMWARE_DIR}/hardware/motor/Motors.cpp"
    "${FIRMWARE_DIR}/hardware/motor/Kicker.cpp"
    "${FIRMWARE_DIR}/hardware/sensors/GPort.cpp"
    "${FIRMWARE_DIR}/hardware/sensors/LineSensor.cpp"
    "${FIRMWARE_DIR}/hardware/sensors/pos/Compass.cpp"
    "${FIRMWARE_DIR}/hardware/sensors/pos/Accelerometer.cpp"
    "${FIRMWARE_DIR}/hardware/sensors/pos/Sonar.cpp"
    "${FIRMWARE_DIR}/hardware/io/encoder/Encoder.cpp"
    "${FIRMWARE_DIR}/hardware/io/cordic/cordic.c"
    "${FIRMWARE_DIR}/strategy/EKF.cpp"
    "${FIRMWARE_DIR}/strategy/DigitalField.cpp"
    "${FIRMWARE_DIR}/helpers/MatrixMultiplyArm.cpp"
    ${FW_TESTS})
add_library(bucky_firmware OBJECT ${FW_SOURCES})
target_include_directories(bucky_firmware PRIVATE ${SIM_INCLUDES})
target_compile_definitions(bucky_firmware PRIVATE ${BOARD_DEFS} BUCKY_SIM=1)
target_compile_options(bucky_firmware PRIVATE ${WARN} -Wno-unknown-pragmas -Wno-register -fexceptions)
if(BUCKY_FW_FAST_MATH)
    target_compile_options(bucky_firmware PRIVATE -ffast-math)
endif()
set_source_files_properties("${FIRMWARE_DIR}/hardware/io/cordic/cordic.c" PROPERTIES COMPILE_OPTIONS "-fexceptions")
set(_main_defs main=fw_main)
if(BUCKY_SIM_RUN_TEST)
    list(APPEND _main_defs "${BUCKY_SIM_RUN_TEST}=bucky_sim_run_test")
endif()
set_source_files_properties("${FIRMWARE_DIR}/main.cpp" PROPERTIES COMPILE_DEFINITIONS "${_main_defs}")

# ---- the simulated MCU + glue + bindings ----------------------------------------------------
file(GLOB SIM_CORE CONFIGURE_DEPENDS
    "${SIM_DIR}/src/core/*.cpp" "${SIM_DIR}/src/arduino/*.cpp" "${SIM_DIR}/src/io/*.cpp"
    "${SIM_DIR}/src/devices/*.cpp" "${SIM_DIR}/src/firmware/*.cpp")
set(VENDOR_SOURCES
    "${SIM_DIR}/vendor/stm32duino/Print.cpp"
    "${SIM_DIR}/vendor/stm32duino/WString.cpp"
    "${SIM_DIR}/vendor/stm32duino/itoa.c"
    "${SIM_DIR}/vendor/stm32duino/avr/dtostrf.c"
    "${SIM_DIR}/boards/${BUCKY_BOARD}/PeripheralPins.cpp")
file(GLOB BINDING_SOURCES CONFIGURE_DEPENDS "${SIM_DIR}/src/bindings/*.cpp")

pybind11_add_module(bucky_fw MODULE ${SIM_CORE} ${VENDOR_SOURCES} ${BINDING_SOURCES}
                    $<TARGET_OBJECTS:bucky_firmware>)
target_include_directories(bucky_fw PRIVATE ${SIM_INCLUDES})
target_compile_definitions(bucky_fw PRIVATE ${BOARD_DEFS} BUCKY_SIM=1)
target_compile_options(bucky_fw PRIVATE ${WARN})
if(BUCKY_SIM_RUN_TEST)
    set_source_files_properties("${SIM_DIR}/src/firmware/programs.cpp" PROPERTIES COMPILE_DEFINITIONS
        "BUCKY_SIM_RUN_TEST=1;BUCKY_SIM_RUN_TEST_NAME=\"${BUCKY_SIM_RUN_TEST}\"")
endif()
find_package(Threads REQUIRED)
target_link_libraries(bucky_fw PRIVATE Threads::Threads)

if(BUCKY_FW_SANITIZE)
    foreach(t bucky_firmware bucky_fw)
        target_compile_options(${t} PRIVATE -fsanitize=${BUCKY_FW_SANITIZE} -fno-omit-frame-pointer)
    endforeach()
    target_link_options(bucky_fw PRIVATE -fsanitize=${BUCKY_FW_SANITIZE})
endif()

# Archive of the firmware objects alone, for the globals guard test (nm).
add_library(bucky_firmware_objects STATIC $<TARGET_OBJECTS:bucky_firmware>)
