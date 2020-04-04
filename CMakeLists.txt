cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(ujprog VERSION 1.0)

# control where the static and shared libraries are built so that on windows
# we don't need to tinker with the path to run the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

option(BUILD_STATIC "Build completely static binary" OFF)
option(BUILD_STATICLIB "Build binary with non default static binaries" OFF)

if (BUILD_STATIC)
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")
	set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
	find_library(LIBFTDISTATIC libftdi.a REQUIRED)
	find_library(LIBUSB0STATIC libusb.a REQUIRED)
endif()
if (BUILD_STATICLIB)
	if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -framework CoreFoundation -framework IOKit")
		find_library(LIBFTDISTATIC libftdi1.a REQUIRED)
		find_library(LIBUSB0STATIC libusb-1.0.a REQUIRED)
	else()
		find_library(LIBFTDISTATIC libftdi.a REQUIRED)
		find_library(LIBUSB0STATIC libusb.a REQUIRED)
	endif()
	# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc")
endif()
find_package(LibFTDI1 CONFIG REQUIRED)

link_directories(/usr/local/lib)

# configure a header file to pass the version number only
configure_file(ujprog.h.in ujprog.h)

# add the executable
add_executable(ujprog ujprog.c)
target_include_directories(ujprog PRIVATE ${LIBFTDI_INCLUDE_DIR})
if (BUILD_STATICLIB)
	set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
	# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc")
	set_target_properties(ujprog PROPERTIES LINK_SEARCH_START_STATIC 1)
	set_target_properties(ujprog PROPERTIES LINK_SEARCH_END_STATIC 1)
	target_link_libraries(ujprog PUBLIC ${LIBFTDISTATIC} ${LIBUSB0STATIC})
else()
	if (BUILD_STATIC)
		target_link_libraries(ujprog PUBLIC ${LIBFTDISTATIC} ${LIBUSB0STATIC})
	else()
		target_link_libraries(ujprog PUBLIC ${LIBFTDI_LIBRARY} usb)
	endif()
endif()

# install
install(TARGETS ujprog DESTINATION bin)

