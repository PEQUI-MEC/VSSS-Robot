cmake_minimum_required(VERSION 3.5)
project(VSSS C CXX ASM)
set (CMAKE_CXX_STANDARD 11)

set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

set(CMAKE_TOOLCHAIN_FILE "${PROJECT_SOURCE_DIR}/cmake/toolchain_arm.cmake")
set(TOOLCHAIN_PATH "/usr/bin/")
set(TOOLCHAIN TOOLCHAIN_GCC_ARM)

include("${PROJECT_SOURCE_DIR}/cmake/toolchain_arm.cmake")
include("${PROJECT_SOURCE_DIR}/cmake/mbed_directories.cmake")
include("${PROJECT_SOURCE_DIR}/cmake/mbed_file_list.cmake")
include("${PROJECT_SOURCE_DIR}/cmake/mbed_flags.cmake")

set(COMMON_FLAGS "-Os -g -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fmessage-length=0 \
  -fno-exceptions -fno-builtin -ffunction-sections -fdata-sections -funsigned-char -MMD \
   -fno-delete-null-pointer-checks -fomit-frame-pointer -mcpu=cortex-m3 -mthumb --specs=nosys.specs ${MBED_DEFINES}")

set(CMAKE_C_FLAGS "-std=c11 ${COMMON_FLAGS} -include ${MBED_DIR}/mbed_config.h")

set(CMAKE_CXX_FLAGS "-std=c++11 -fno-rtti -Wvla ${COMMON_FLAGS} -include ${MBED_DIR}/mbed_config.h")

set(CMAKE_EXE_LINKER_FLAGS "-Wl,--no-wchar-size-warning -Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,_malloc_r \
 -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_memalign_r -Wl,--wrap,_calloc_r -Wl,--wrap,exit \
 -Wl,--wrap,atexit -Wl,-n -mcpu=cortex-m3 -mthumb -T ${PROJECT_SOURCE_DIR}/LPC1768.ld -Wl,--start-group \
 -lstdc++ -lsupc++ -lm -lc -lgcc -lg -lrdimon -lnosys -Wl,--end-group -static --specs=nano.specs")

set(CMAKE_ASM_FLAGS "-x assembler-with-cpp ${COMMON_FLAGS}")

set(SOURCE_FILES
	${MBED_DIR}/mbed_config.h
	${SRC_DIR}/main.cpp
	${SRC_DIR}/tiny_ekf.c
	${SRC_DIR}/Robot.cpp
	${SRC_DIR}/Messenger.cpp
	${SRC_DIR}/IMU.cpp
	${SRC_DIR}/EKF_Localization.cpp
	${SRC_DIR}/Controller.cpp
	${SRC_DIR}/PIN_MAP.h
	${SRC_DIR}/IMU_regs.h

	${MBED_SRC}
	${LIB_DIR}/XBeeLib/DigiLogger/DigiLogger.cpp
	${LIB_DIR}/XBeeLib/DigiLogger/DigiLoggerMbedSerial.cpp
	${LIB_DIR}/XBeeLib/FrameBuffer/FrameBuffer.cpp
	${LIB_DIR}/XBeeLib/FrameHandlers/FH_AtCmdResp.cpp
	${LIB_DIR}/XBeeLib/FrameHandlers/FH_IoDataSample802.cpp
	${LIB_DIR}/XBeeLib/FrameHandlers/FH_IoDataSampleDM.cpp
	${LIB_DIR}/XBeeLib/FrameHandlers/FH_IoDataSampleZB.cpp
	${LIB_DIR}/XBeeLib/FrameHandlers/FH_ModemStatus.cpp
	${LIB_DIR}/XBeeLib/FrameHandlers/FH_RxPacket802.cpp
	${LIB_DIR}/XBeeLib/FrameHandlers/FH_RxPacketDM.cpp
	${LIB_DIR}/XBeeLib/FrameHandlers/FH_RxPacketZB.cpp
	${LIB_DIR}/XBeeLib/FrameHandlers/FrameHandler.cpp
	${LIB_DIR}/XBeeLib/Frames/802_Frames.cpp
	${LIB_DIR}/XBeeLib/Frames/ApiFrame.cpp
	${LIB_DIR}/XBeeLib/Frames/AtCmdFrame.cpp
	${LIB_DIR}/XBeeLib/Frames/DigiMeshFrames.cpp
	${LIB_DIR}/XBeeLib/Frames/ZigbeeFrames.cpp
	${LIB_DIR}/XBeeLib/IO/IOSample802.cpp
	${LIB_DIR}/XBeeLib/IO/IOSampleDM.cpp
	${LIB_DIR}/XBeeLib/IO/IOSampleZB.cpp
	${LIB_DIR}/XBeeLib/RemoteXBee/RemoteXBee.cpp
	${LIB_DIR}/XBeeLib/Utils/Utils.cpp
	${LIB_DIR}/XBeeLib/XBee/AtCommands.cpp
	${LIB_DIR}/XBeeLib/XBee/RadioConfig.cpp
	${LIB_DIR}/XBeeLib/XBee/XBee.cpp
	${LIB_DIR}/XBeeLib/XBee802/XBee802.cpp
	${LIB_DIR}/XBeeLib/XBeeDM/XBeeDM.cpp
	${LIB_DIR}/XBeeLib/XBeeZB/XBeeZB.cpp
	${LIB_DIR}/QEI/QEI.cpp)

include_directories(
	${SRC_DIR}
	${PROJECT_SOURCE_DIR}
	${MBED_DIRECTORIES}
	${LIB_DIR}/XBeeLib
	${LIB_DIR}/XBeeLib/DigiLogger
	${LIB_DIR}/XBeeLib/FrameBuffer
	${LIB_DIR}/XBeeLib/FrameHandlers
	${LIB_DIR}/XBeeLib/Frames
	${LIB_DIR}/XBeeLib/IO
	${LIB_DIR}/XBeeLib/RemoteXBee
	${LIB_DIR}/XBeeLib/Utils
	${LIB_DIR}/XBeeLib/XBee
	${LIB_DIR}/XBeeLib/XBee802
	${LIB_DIR}/XBeeLib/XBeeDM
	${LIB_DIR}/XBeeLib/XBeeZB
	${LIB_DIR}/QEI)

add_executable(${PROJECT_NAME}.elf ${SOURCE_FILES})
add_custom_command(TARGET ${PROJECT_NAME}.elf
	POST_BUILD
	COMMAND arm-none-eabi-objcopy -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin)
