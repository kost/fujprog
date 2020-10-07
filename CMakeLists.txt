cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(fujprog VERSION 4.8)

# control where the static and shared libraries are built so that on windows
# we don't need to tinker with the path to run the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

option(BUILD_STATIC "Build completely static binary" OFF)
option(BUILD_STATICLIB "Build binary with non default static binaries" OFF)

if (BUILD_STATIC)
	 set(CMAKE_EXE_LINKER_FLAGS "-static")
	 set(BUILD_SHARED_LIBS OFF)
endif()

if (BUILD_STATICLIB)
	find_library(LIBFTDISTATIC libftdi1.a REQUIRED)
	find_library(LIBUSBSTATIC libusb-1.0.a REQUIRED)
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -framework CoreFoundation -framework IOKit")
endif()



set(FUJPROG_HEADERS
	fujprog.h
)

set(FUJPROG_SOURCE
	fujprog.c
)

if (MSVC)
	set(FUJPROG_SOURCE "${FUJPROG_SOURCE}" "getopt.c")
	add_compile_options(
	$<$<CONFIG:>:/MT> #---------|
	$<$<CONFIG:Debug>:/MTd> #---|-- Statically link the runtime libraries
	$<$<CONFIG:Release>:/MT> #--|
	)
else()
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(LIBFTDI REQUIRED libftdi1)
	pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
	link_directories(${LIBUSB_LIBRARY_DIRS} ${LIBFTDI_LIBRARY_DIRS})
endif()

set(fujprog_git_version "unknown")

find_package(Git)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY "${local_dir}"
    OUTPUT_VARIABLE fujprog_git_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message( STATUS "GIT hash: ${fujprog_git_version}")
else()
  message(STATUS "GIT not found")
endif()

string(TIMESTAMP _time_stamp)

add_executable(fujprog
	${FUJPROG_SOURCE}
	${FUJPROG_HEADERS}
)

if (WIN32)
	link_directories(${PROJECT_SOURCE_DIR})
	add_definitions(-DFTD2XX_STATIC=1)
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")
	find_library(LIBFTDISTATIC ftd2xx.lib PATHS ${CMAKE_CURRENT_SOURCE_DIR} REQUIRED)
	include_directories(${CMAKE_CURRENT_SOURCE_DIR})
	target_link_libraries(fujprog PUBLIC ${LIBFTDISTATIC})
else()
if (BUILD_STATICLIB)
	set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
	set_target_properties(fujprog PROPERTIES LINK_SEARCH_START_STATIC 1)
	set_target_properties(fujprog PROPERTIES LINK_SEARCH_END_STATIC 1)
	target_link_libraries(fujprog PUBLIC ${LIBFTDISTATIC} ${LIBUSBSTATIC})
else()
	target_link_libraries(fujprog
		${LIBUSB_LIBRARIES}
		${LIBFTDI_LIBRARIES}
	)
endif()
endif()

# configure a header file to pass the version number only
configure_file(fujprog.h.in fujprog.h)

include_directories(
	${LIBUSB_INCLUDE_DIRS}
	${LIBFTDI_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}
)


if (BUILD_STATIC)
	 set_target_properties(fujprog PROPERTIES LINK_SEARCH_END_STATIC 1)
endif()

# install
install(TARGETS fujprog DESTINATION bin)

