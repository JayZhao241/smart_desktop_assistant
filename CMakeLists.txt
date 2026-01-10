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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME smart_desktop_assistant)

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

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
# 1. 定义 BSP 和 SYSTEM 路径
set(BSP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/BSP)
set(SYSTEM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/SYSTEM)

# 2. 自动搜索 BSP 和 SYSTEM 下所有子目录的 .c 文件
file(GLOB_RECURSE BSP_SOURCES 
    "${BSP_DIR}/*.c"
    "${SYSTEM_DIR}/*.c"
)

# 3. 自动搜索所有包含 .h 的目录并加入包含路径
file(GLOB_RECURSE BSP_HEADERS "${BSP_DIR}/*.h" "${SYSTEM_DIR}/*.h")
set(BSP_INCLUDE_DIRS "")
foreach(_headerFile ${BSP_HEADERS})
    get_filename_component(_dir ${_headerFile} DIRECTORY)
    list(APPEND BSP_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES BSP_INCLUDE_DIRS)
list(FILTER BSP_SOURCES EXCLUDE REGEX "lcd_ex.c")

# 4. 将搜索到的文件和路径关联到当前工程
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${BSP_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${BSP_INCLUDE_DIRS})

message(STATUS "Auto-added BSP sources: ${BSP_SOURCES}")
