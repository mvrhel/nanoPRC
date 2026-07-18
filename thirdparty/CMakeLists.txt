cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(thirdparty)

option(NANOPRC_BUILD_SDL "Build SDL dependency" ON)

include(zlib.cmake)

if (NANOPRC_BUILD_SDL)
	add_subdirectory(SDL)
endif()
