##########################################################################
# "THE ANY BEVERAGE-WARE LICENSE" (Revision 42 - based on beer-ware
# license):
# <dev@layer128.net> wrote this file. As long as you retain this notice
# you can do whatever you want with this stuff. If we meet some day, and
# you think this stuff is worth it, you can buy me a be(ve)er(age) in
# return. (I don't like beer much.)
#
# Matthias Kleemann
#
# modified by Christian Herrmann for the use in HWP
##########################################################################

##########################################################################
# The toolchain requires the variable AVR_MCU set:
#
# AVR_MCU (default: atmega1280)
#     the type of AVR the application is built for
#
# These variables are optional:
# AVR_TOOLCHAIN_PATH (auto-detected if not given)
#     the base path of the AVR toolchain (compiler, linker, ...)
#     Can be given as environment variable or cmake command line option -D.
# AVR_LIBC_PATH (auto-detected if not given)
#     path of the "avr" directory containing the AVR libc (libs and headers).
#     Usually, this path is found as ${AVR_TOOLCHAIN_PATH}/avr or /usr/lib/avr.
#     Can be given as environment variable or cmake command line option -D.
# AVR_L_FUSE (NO DEFAULT)
#     the LOW fuse value for the MCU used
# AVR_H_FUSE (NO DEFAULT)
#     the HIGH fuse value for the MCU used
# AVR_E_FUSE (NO DEFAULT)
#     the EXTENDED fuse value for the MCU used
# AVR_UPLOADTOOL (default: avrdude)
#     the application used to upload to the MCU
# AVR_UPLOADTOOL_PORT (default: usb)
#     the port used for the upload tool, e.g. usb
# AVR_PROGRAMMER (default: avr109)
#     the programmer hardware used, e.g. avrispmkII
# AVR_SIZE_ARGS (default: --format=avr;--mcu=${AVR_MCU})
#     arguments to avr-size
##########################################################################

##########################################################################
# options
##########################################################################
option(WITH_MCU "Add the MCU type to the target file name." OFF)

##########################################################################
# auto-detect location of base directory of AVR toolchain on Windows
# For Linux and macOS it is assumed that it is located in a path from
# the system environment variable PATH.
##########################################################################
if (DEFINED ENV{AVR_TOOLCHAIN_PATH})
	set(AVR_TOOLCHAIN_PATH $ENV{AVR_TOOLCHAIN_PATH})
endif()

if (DEFINED ENV{AVR_LIBC_PATH})
	set(AVR_LIBC_PATH $ENV{AVR_LIBC_PATH})
endif()

# propagate the variables AVR_TOOLCHAIN_PATH and AVR_LIBC_PATH to the
# try-compile runs
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES AVR_TOOLCHAIN_PATH AVR_LIBC_PATH)

if (DEFINED AVR_TOOLCHAIN_PATH)
	if(NOT EXISTS "${AVR_TOOLCHAIN_PATH}")
		message(FATAL_ERROR "Directory ${AVR_TOOLCHAIN_PATH} does not exist. Please specify a correct AVR_TOOLCHAIN_PATH as environment variable or by cmake command line option -D.")
	endif()
	if(NOT DEFINED AVR_LIBC_PATH)
		if(EXISTS "${AVR_TOOLCHAIN_PATH}/avr")
			set(AVR_LIBC_PATH "${AVR_TOOLCHAIN_PATH}/avr")
		endif()
	endif()
else()
	if (CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
		if (EXISTS "C:/Program Files/avr8-gnu-toolchain-win32_x86")
			set(AVR_TOOLCHAIN_PATH "C:/Program Files/avr8-gnu-toolchain-win32_x86")
		elseif (EXISTS "C:/Program Files/avr8-gnu-toolchain-win32_x86_64")
			set(AVR_TOOLCHAIN_PATH "C:/Program Files/avr8-gnu-toolchain-win32_x86_64")
		elseif (EXISTS "C:/Program Files (x86)/avr8-gnu-toolchain-win32_x86")
			set(AVR_TOOLCHAIN_PATH "C:/Program Files (x86)/avr8-gnu-toolchain-win32_x86")
		elseif (EXISTS "C:/Program Files (x86)/avr8-gnu-toolchain-win32_x86_64")
			set(AVR_TOOLCHAIN_PATH "C:/Program Files (x86)/avr8-gnu-toolchain-win32_x86_64")
		elseif (EXISTS "C:/avr8-gnu-toolchain-win32_x86")
			set(AVR_TOOLCHAIN_PATH "C:/avr8-gnu-toolchain-win32_x86")
		elseif (EXISTS "C:/avr8-gnu-toolchain-win32_x86_64")
			set(AVR_TOOLCHAIN_PATH "C:/avr8-gnu-toolchain-win32_x86_64")
		elseif (EXISTS "C:/Program Files/avr8-gnu-toolchain")
			set(AVR_TOOLCHAIN_PATH "C:/Program Files/avr8-gnu-toolchain")
		elseif (EXISTS "C:/Program Files (x86)/avr8-gnu-toolchain")
			set(AVR_TOOLCHAIN_PATH "C:/Program Files (x86)/avr8-gnu-toolchain")
		elseif (EXISTS "C:/avr8-gnu-toolchain")
			set(AVR_TOOLCHAIN_PATH "C:/avr8-gnu-toolchain")
		else()
			message(FATAL_ERROR "No AVR toolchain detected. Run cmake with -DAVR_TOOLCHAIN_PATH=<PATH_TO_AVR_TOOLCHAIN>.")
		endif()

		if(NOT DEFINED AVR_LIBC_PATH)
			if(EXISTS "${AVR_TOOLCHAIN_PATH}/avr")
				set(AVR_LIBC_PATH "${AVR_TOOLCHAIN_PATH}/avr")
			endif()
		endif()
	endif()
endif()

##########################################################################
# detect AVR libc, usually in the path "${AVR_TOOLCHAIN_PATH}/avr" or /usr/lib/avr
##########################################################################

if(DEFINED AVR_LIBC_PATH)
	if(NOT EXISTS "${AVR_LIBC_PATH}")
		message(FATAL_ERROR "Directory ${AVR_LIBC_PATH} does not exist. Please specify a correct AVR_LIBC_PATH as environment variable or by cmake command line option -D.")
	endif()
	set(CMAKE_FIND_ROOT_PATH "${AVR_LIBC_PATH}")
else()
	if (CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
		if(EXISTS "${AVR_TOOLCHAIN_PATH}/avr")
			set(CMAKE_FIND_ROOT_PATH "${AVR_TOOLCHAIN_PATH}/avr")
		else()
			message(FATAL_ERROR "Directory ${AVR_TOOLCHAIN_PATH}/avr does not exist. Please specify a correct AVR_LIBC_PATH as environment variable or by cmake command line option -D.")
		endif()
	else()
		if(EXISTS "/usr/lib/avr")
			set(CMAKE_FIND_ROOT_PATH "/usr/lib/avr")
		elseif(EXISTS "/usr/local/lib/avr")
			set(CMAKE_FIND_ROOT_PATH "/usr/local/lib/avr")
		elseif(EXISTS "/usr/avr")
			set(CMAKE_FIND_ROOT_PATH "/usr/avr")
		elseif(EXISTS "/usr/local/CrossPack-AVR")
			set(CMAKE_FIND_ROOT_PATH "/usr/local/CrossPack-AVR")
		elseif(EXISTS "/opt/local/avr")
			set(CMAKE_FIND_ROOT_PATH "/opt/local/avr")
		else()
			message(FATAL_ERROR "Please specify a correct AVR_LIBC_PATH as environment variable or by cmake command line option -D.")
		endif()
	endif()
endif()

##########################################################################
# detect compiler and auxiliary binaries
##########################################################################
find_program(AVR_CC avr-gcc REQUIRED HINTS "${AVR_TOOLCHAIN_PATH}/bin")
find_program(AVR_CXX avr-g++ REQUIRED HINTS "${AVR_TOOLCHAIN_PATH}/bin")
find_program(AVR_OBJCOPY avr-objcopy REQUIRED HINTS "${AVR_TOOLCHAIN_PATH}/bin")
find_program(AVR_OBJDUMP avr-objdump REQUIRED HINTS "${AVR_TOOLCHAIN_PATH}/bin")
find_program(AVR_SIZE_TOOL avr-size REQUIRED HINTS "${AVR_TOOLCHAIN_PATH}/bin")

##########################################################################
# toolchain starts with defining mandatory variables
##########################################################################
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)
set(CMAKE_C_COMPILER ${AVR_CC})
set(CMAKE_CXX_COMPILER ${AVR_CXX})
set(CMAKE_ASM_COMPILER ${AVR_CC})

# add compile option "-x assembler-with-cpp" only for assembler files *.S
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

add_compile_options(-c)

##########################################################################
# set the way how CMake detects programs, libs and headers
##########################################################################
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

##########################################################################
# Identification
##########################################################################
set(AVR 1)

##########################################################################
# some optional tools and variables for AVR builds, which may not be
# defined yet
# - AVR_UPLOADTOOL
# - AVR_UPLOADTOOL_PORT
# - AVR_PROGRAMMER
# - AVR_MCU
# - AVR_SIZE_ARGS
##########################################################################

# default upload tool
if(NOT DEFINED AVR_UPLOADTOOL)
	set(
			AVR_UPLOADTOOL avrdude
			CACHE STRING "Set default upload tool: avrdude"
	)
	find_program(AVR_UPLOADTOOL avrdude HINTS "${AVRDUDE_PATH}")
endif()

# default upload tool port
if(NOT DEFINED AVR_UPLOADTOOL_PORT)
	set(
			AVR_UPLOADTOOL_PORT usb
			CACHE STRING "Set default upload tool port: usb"
	)
endif()

# default programmer (hardware)
if(NOT DEFINED AVR_PROGRAMMER)
	set(
			AVR_PROGRAMMER avr109
			CACHE STRING "Set default programmer hardware model: avr109"
	)
endif()

# default MCU (chip)
if(NOT DEFINED AVR_MCU)
	set(
			AVR_MCU atmega1280
			CACHE STRING "Set default MCU: atmega8 (see 'avr-gcc --target-help' for valid values)"
	)
endif()
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES AVR_MCU)

#default avr-size args
if(NOT DEFINED AVR_SIZE_ARGS)
	set(AVR_SIZE_ARGS)
endif()

# prepare base flags for upload tool
set(AVR_UPLOADTOOL_BASE_OPTIONS -p ${AVR_MCU} -c ${AVR_PROGRAMMER})

# use AVR_UPLOADTOOL_BAUDRATE as baudrate for upload tool (if defined)
if(DEFINED AVR_UPLOADTOOL_BAUDRATE)
	set(AVR_UPLOADTOOL_BASE_OPTIONS ${AVR_UPLOADTOOL_BASE_OPTIONS} -b ${AVR_UPLOADTOOL_BAUDRATE})
endif()

##########################################################################
# check build types:
# - Debug
# - Release
# - RelWithDebInfo
# - MinSizeRel
#
# If none is given, "Release" is chosen, because of some optimized
# functions in the AVR toolchain, e.g. _delay_ms().
##########################################################################
if(NOT ((CMAKE_BUILD_TYPE MATCHES Release) OR
(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo) OR
(CMAKE_BUILD_TYPE MATCHES Debug) OR
(CMAKE_BUILD_TYPE MATCHES MinSizeRel)))
	message(STATUS "Warning: No build type specified by CMAKE_BUILD_TYPE. Using build type 'Release'.")
	set(
			CMAKE_BUILD_TYPE Release
			CACHE STRING "Choose cmake build type: Debug Release RelWithDebInfo MinSizeRel"
			FORCE
	)
endif()

##########################################################################

##########################################################################
# target file name add-on
##########################################################################
if(WITH_MCU)
	set(MCU_TYPE_FOR_FILENAME "-${AVR_MCU}")
else()
	set(MCU_TYPE_FOR_FILENAME "")
endif()

##########################################################################
# add_avr_executable
# - IN_VAR: EXECUTABLE_NAME
#
# Creates targets and dependencies for AVR toolchain, building an
# executable. Calls add_executable with ELF file as target name, so
# any link dependencies need to be using that target, e.g. for
# target_link_libraries(<EXECUTABLE_NAME>-${AVR_MCU}.elf ...).
##########################################################################
function(add_avr_executable EXECUTABLE_NAME)

	if(NOT ARGN)
		message(FATAL_ERROR "No source files given for ${EXECUTABLE_NAME}.")
	endif()

	# set file names
	set(elf_file ${EXECUTABLE_NAME}${MCU_TYPE_FOR_FILENAME}.elf)
	set(hex_file ${EXECUTABLE_NAME}${MCU_TYPE_FOR_FILENAME}.hex)
	set(lss_file ${EXECUTABLE_NAME}${MCU_TYPE_FOR_FILENAME}.lss)
	set(map_file ${EXECUTABLE_NAME}${MCU_TYPE_FOR_FILENAME}.map)
	set(eeprom_image ${EXECUTABLE_NAME}${MCU_TYPE_FOR_FILENAME}.eep)

	set (${EXECUTABLE_NAME}_ELF_TARGET ${elf_file} PARENT_SCOPE)
	set (${EXECUTABLE_NAME}_HEX_TARGET ${hex_file} PARENT_SCOPE)
	set (${EXECUTABLE_NAME}_LSS_TARGET ${lss_file} PARENT_SCOPE)
	set (${EXECUTABLE_NAME}_MAP_TARGET ${map_file} PARENT_SCOPE)
	set (${EXECUTABLE_NAME}_EEPROM_TARGET ${eeprom_file} PARENT_SCOPE)
	# elf file
	add_executable(${elf_file} EXCLUDE_FROM_ALL ${ARGN})

	set_target_properties(
			${elf_file}
			PROPERTIES
			COMPILE_FLAGS "-mmcu=${AVR_MCU}"
			LINK_FLAGS "-mmcu=${AVR_MCU} -Wl,-Map,${map_file}"
	)

	add_custom_command(
			OUTPUT ${hex_file}
			COMMAND
			${AVR_OBJCOPY} -j .text -j .data -O ihex ${elf_file} ${hex_file}
			COMMAND
			${AVR_SIZE_TOOL} ${AVR_SIZE_ARGS} ${elf_file}
			DEPENDS ${elf_file}
	)

	add_custom_command(
			OUTPUT ${lss_file}
			COMMAND
			${AVR_OBJDUMP} -h -S ${elf_file} > ${lss_file}
			DEPENDS ${elf_file}
	)

	# eeprom
	add_custom_command(
			OUTPUT ${eeprom_image}
			COMMAND
			${AVR_OBJCOPY} -j .eeprom --set-section-flags=.eeprom=alloc,load
			--change-section-lma .eeprom=0 --no-change-warnings
			-O ihex ${elf_file} ${eeprom_image}
			DEPENDS ${elf_file}
	)

	add_custom_target(
			${EXECUTABLE_NAME}
			ALL
			DEPENDS ${hex_file} ${lss_file} ${eeprom_image}
	)

	set_target_properties(
			${EXECUTABLE_NAME}
			PROPERTIES
			OUTPUT_NAME "${elf_file}"
	)

	# clean
	get_directory_property(clean_files ADDITIONAL_MAKE_CLEAN_FILES)
	set_directory_properties(
			PROPERTIES
			ADDITIONAL_MAKE_CLEAN_FILES "${map_file}"
	)

	# upload - with avrdude
	add_custom_target(
			upload_${EXECUTABLE_NAME}
			${AVR_UPLOADTOOL} ${AVR_UPLOADTOOL_BASE_OPTIONS} ${AVR_UPLOADTOOL_OPTIONS}
			-U flash:w:${hex_file}
			-P ${AVR_UPLOADTOOL_PORT}
			DEPENDS ${hex_file}
			COMMENT "Uploading ${hex_file} to ${AVR_MCU} using ${AVR_PROGRAMMER}"
	)

	# upload eeprom only - with avrdude
	# see also bug http://savannah.nongnu.org/bugs/?40142
	add_custom_target(
			upload_${EXECUTABLE_NAME}_eeprom
			${AVR_UPLOADTOOL} ${AVR_UPLOADTOOL_BASE_OPTIONS} ${AVR_UPLOADTOOL_OPTIONS}
			-U eeprom:w:${eeprom_image}
			-P ${AVR_UPLOADTOOL_PORT}
			DEPENDS ${eeprom_image}
			COMMENT "Uploading ${eeprom_image} to ${AVR_MCU} using ${AVR_PROGRAMMER}"
	)

	# disassemble
	add_custom_target(
			disassemble_${EXECUTABLE_NAME}
			${AVR_OBJDUMP} -h -S ${elf_file} > ${EXECUTABLE_NAME}.lst
			DEPENDS ${elf_file}
	)
endfunction()


##########################################################################
# add_avr_library
# - IN_VAR: LIBRARY_NAME
#
# Calls add_library with an optionally concatenated name
# <LIBRARY_NAME>${MCU_TYPE_FOR_FILENAME}.
# This needs to be used for linking against the library, e.g. calling
# target_link_libraries(...).
##########################################################################
function(add_avr_library LIBRARY_NAME)
	if(NOT ARGN)
		message(FATAL_ERROR "No source files given for ${LIBRARY_NAME}.")
	endif()

	set(lib_file ${LIBRARY_NAME}${MCU_TYPE_FOR_FILENAME})
	set (${LIBRARY_NAME}_LIB_TARGET ${elf_file} PARENT_SCOPE)

	add_library(${lib_file} STATIC ${ARGN})

	set_target_properties(
			${lib_file}
			PROPERTIES
			COMPILE_FLAGS "-mmcu=${AVR_MCU}"
			OUTPUT_NAME "${lib_file}"
	)

	if(NOT TARGET ${LIBRARY_NAME})
		add_custom_target(
				${LIBRARY_NAME}
				ALL
				DEPENDS ${lib_file}
		)

		set_target_properties(
				${LIBRARY_NAME}
				PROPERTIES
				OUTPUT_NAME "${lib_file}"
		)
	endif()

endfunction()

##########################################################################
# avr_target_link_libraries
# - IN_VAR: EXECUTABLE_TARGET
# - ARGN  : targets and files to link to
#
# Calls target_link_libraries with AVR target names (concatenation,
# extensions and so on.
##########################################################################
function(avr_target_link_libraries EXECUTABLE_TARGET)
	if(NOT ARGN)
		message(FATAL_ERROR "Nothing to link to ${EXECUTABLE_TARGET}.")
	endif()

	get_target_property(TARGET_LIST ${EXECUTABLE_TARGET} OUTPUT_NAME)

	foreach(TGT ${ARGN})
		if(TARGET ${TGT})
			get_target_property(ARG_NAME ${TGT} OUTPUT_NAME)
			list(APPEND NON_TARGET_LIST ${ARG_NAME})
		else()
			list(APPEND NON_TARGET_LIST ${TGT})
		endif()
	endforeach()

	target_link_libraries(${TARGET_LIST} ${NON_TARGET_LIST})
endfunction()

##########################################################################
# avr_target_link_options
#
# Calls target_link_options with AVR target names
##########################################################################

function(avr_target_link_options EXECUTABLE_TARGET)
	if(NOT ARGN)
		message(FATAL_ERROR "No link options to add to ${EXECUTABLE_TARGET}.")
	endif()

	get_target_property(TARGET_LIST ${EXECUTABLE_TARGET} OUTPUT_NAME)
	set(extra_args ${ARGN})

	target_link_options(${TARGET_LIST} PRIVATE ${extra_args})
endfunction()

##########################################################################
# avr_target_include_directories
#
# Calls target_include_directories with AVR target names
##########################################################################

function(avr_target_include_directories EXECUTABLE_TARGET)
	if(NOT ARGN)
		message(FATAL_ERROR "No include directories to add to ${EXECUTABLE_TARGET}.")
	endif()

	get_target_property(TARGET_LIST ${EXECUTABLE_TARGET} OUTPUT_NAME)
	set(extra_args ${ARGN})

	target_include_directories(${TARGET_LIST} PRIVATE ${extra_args})
endfunction()

##########################################################################
# avr_target_compile_definitions
#
# Calls target_compile_definitions with AVR target names
##########################################################################

function(avr_target_compile_definitions EXECUTABLE_TARGET)
	if(NOT ARGN)
		message(FATAL_ERROR "No compile definitions to add to ${EXECUTABLE_TARGET}.")
	endif()

	get_target_property(TARGET_LIST ${EXECUTABLE_TARGET} OUTPUT_NAME)
	set(extra_args ${ARGN})

	target_compile_definitions(${TARGET_LIST} PRIVATE ${extra_args})
endfunction()

##########################################################################
# avr_target_compile_options
#
# Calls target_compile_options with AVR target names
##########################################################################

function(avr_target_compile_options EXECUTABLE_TARGET)
	if(NOT ARGN)
		message(FATAL_ERROR "No compile options to add to ${EXECUTABLE_TARGET}.")
	endif()

	get_target_property(TARGET_LIST ${EXECUTABLE_TARGET} OUTPUT_NAME)
	set(extra_args ${ARGN})

	target_compile_options(${TARGET_LIST} PRIVATE ${extra_args})
endfunction()

function(avr_generate_fixed_targets)
	# get status
	add_custom_target(
			get_status
			${AVR_UPLOADTOOL} ${AVR_UPLOADTOOL_BASE_OPTIONS} -P ${AVR_UPLOADTOOL_PORT} -n -v
			COMMENT "Get status from ${AVR_MCU}"
	)

	# get fuses
	add_custom_target(
			get_fuses
			${AVR_UPLOADTOOL} ${AVR_UPLOADTOOL_BASE_OPTIONS} -P ${AVR_UPLOADTOOL_PORT} -n
			-U lfuse:r:-:b
			-U hfuse:r:-:b
			-U efuse:r:-:b
			COMMENT "Get fuses from ${AVR_MCU}"
	)

	# set fuses
	add_custom_target(
			set_fuses
			${AVR_UPLOADTOOL} ${AVR_UPLOADTOOL_BASE_OPTIONS} -P ${AVR_UPLOADTOOL_PORT}
			-U lfuse:w:${AVR_L_FUSE}:m
			-U hfuse:w:${AVR_H_FUSE}:m
			-U efuse:w:${AVR_E_FUSE}:m
			COMMENT "Setup: Extended Fuse: ${AVR_E_FUSE} High Fuse: ${AVR_H_FUSE} Low Fuse: ${AVR_L_FUSE}"
	)

	# get oscillator calibration
	add_custom_target(
			get_calibration
			${AVR_UPLOADTOOL} ${AVR_UPLOADTOOL_BASE_OPTIONS} -P ${AVR_UPLOADTOOL_PORT}
			-U calibration:r:${AVR_MCU}_calib.tmp:r
			COMMENT "Write calibration status of internal oscillator to ${AVR_MCU}_calib.tmp."
	)

	# set oscillator calibration
	add_custom_target(
			set_calibration
			${AVR_UPLOADTOOL} ${AVR_UPLOADTOOL_BASE_OPTIONS} -P ${AVR_UPLOADTOOL_PORT}
			-U calibration:w:${AVR_MCU}_calib.hex
			COMMENT "Program calibration status of internal oscillator from ${AVR_MCU}_calib.hex."
	)
endfunction()

##########################################################################
# Bypass the link step in CMake's "compiler sanity test" check
#
# CMake throws in a try_compile() target test in some generators, but does
# not know that this is a cross compiler so the executable can't link.
# Change the target type:
#
# https://stackoverflow.com/q/53633705
##########################################################################

set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
