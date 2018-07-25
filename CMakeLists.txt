cmake_minimum_required(VERSION 3.5)
project(VSSS C CXX ASM)
set (CMAKE_CXX_STANDARD 14)

set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

set(CMAKE_TOOLCHAIN_FILE "${PROJECT_SOURCE_DIR}/cmake/toolchain_arm.cmake")
set(TOOLCHAIN_PATH "/usr/bin/")
set(TOOLCHAIN TOOLCHAIN_GCC_ARM)

include("${PROJECT_SOURCE_DIR}/cmake/toolchain_arm.cmake")
include("${PROJECT_SOURCE_DIR}/cmake/mbed_directories.cmake")
include("${PROJECT_SOURCE_DIR}/cmake/mbed_file_list.cmake")
include("${PROJECT_SOURCE_DIR}/cmake/mbed_flags.cmake")
include("${PROJECT_SOURCE_DIR}/cmake/xbeelib_files.cmake")
include("${PROJECT_SOURCE_DIR}/cmake/eigen_files.cmake")

set(COMMON_FLAGS "-O3 -g -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 \
  -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD \
   -fno-delete-null-pointer-checks -fomit-frame-pointer -mcpu=cortex-m3 -mthumb --specs=nosys.specs ${MBED_DEFINES}")

set(CMAKE_C_FLAGS "-std=c11 ${COMMON_FLAGS} -include ${MBED_DIR}/mbed_config.h")

set(CMAKE_CXX_FLAGS "-std=c++14 -faligned-new -fno-rtti -Wvla ${COMMON_FLAGS} -include ${MBED_DIR}/mbed_config.h")

set(CMAKE_EXE_LINKER_FLAGS "-Wl,--no-wchar-size-warning -Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,_malloc_r \
 -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_memalign_r -Wl,--wrap,_calloc_r -Wl,--wrap,exit \
 -Wl,--wrap,atexit -Wl,-n -mcpu=cortex-m3 -mthumb -T ${PROJECT_SOURCE_DIR}/LPC1768.ld -Wl,--start-group \
 -u _printf_float -lstdc++ -lsupc++ -lm -lc -lgcc -lg -lrdimon -lnosys -Wl,--end-group -static --specs=nano.specs")

set(CMAKE_ASM_FLAGS "-x assembler-with-cpp ${COMMON_FLAGS}")

set(SOURCE_FILES
	${SRC_DIR}/main.cpp
	${SRC_DIR}/Robot.cpp
	${SRC_DIR}/Messenger.cpp
	${SRC_DIR}/IMU.cpp
	${SRC_DIR}/Controller.cpp
	${SRC_DIR}/PIN_MAP.h
	${SRC_DIR}/IMU_regs.h
	${SRC_DIR}/EKF.cpp
	${SRC_DIR}/SensorFusion.cpp
	${SRC_DIR}/ConfigFile.cpp
	${SRC_DIR}/helper_functions.h

	${MBED_SRC}
	${XBEELIB_SRC}
	${LIB_DIR}/QEI/QEI.cpp
	${EIGEN_FILES})

include_directories(
	${SRC_DIR}
	${PROJECT_SOURCE_DIR}
	${MBED_DIRECTORIES}
	${XBEELIB_DIR}
	${LIB_DIR}/QEI
	${EIGEN_DIRS})

add_executable(${PROJECT_NAME}.elf ${SOURCE_FILES})
add_custom_command(TARGET ${PROJECT_NAME}.elf
	POST_BUILD
	COMMAND arm-none-eabi-objcopy -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin)
