cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME iFOC-AxDr)
add_definitions(-DAXDR_DRIVER)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

### USER DEFINED AREA

MACRO(HEADER_DIRECTORIES return_list)
    FILE(GLOB_RECURSE new_list *.h *.hpp)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

# CPU
set(CPU "-mcpu=cortex-m4")
# FPU
set(FPU "-mfpu=fpv4-sp-d16")
# float-abi
set(FLOAT-ABI "-mfloat-abi=hard")
# MCU
set(MCU "${CPU} -mthumb ${FPU} ${FLOAT-ABI}")

# Optimization (switch -Ox at CMAKE PROFILES!)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU} -flto")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MCU} -flto")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU} -fsingle-precision-constant")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MCU} -fsingle-precision-constant")

# Add additional flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -specs=nosys.specs")

# User Sources and headers
file(GLOB_RECURSE src User/*.cpp User/*.c)
file(GLOB_RECURSE headers User/*.h User/*.hpp)

HEADER_DIRECTORIES(header_dir_list)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${header_dir_list})

# elf -> bin with arm-none-eabi-objcopy
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND arm-none-eabi-size --format=berkeley "${CMAKE_PROJECT_NAME}.elf"
        COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin)

### USER DEFINED AREA

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
        User/iFOC/ThirdParty/CMSIS-DSP
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ${src}
        ${headers}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
        arm_cortexM4lf_math
)
