cmake_minimum_required(VERSION 3.12)
project(tetra-rtlsdr C)

# ---------------------------------------------------------------------------
# tetra-rtlsdr - RTL-SDR front-end for the telive / osmo-tetra ecosystem
#
# Replaces the GnuRadio + simdemod3.py chain with a single C binary.
# Outputs raw bits (1 byte per bit, 0x00/0x01) to stdout, compatible with:
#   tetra-rx -r -s /dev/stdin   (from osmo-tetra-sq5bpf-2 by SQ5BPF)
#
# Credits:
#   telive / osmo-tetra-sq5bpf-2 by Jacek Lipkowski SQ5BPF
#   https://github.com/sq5bpf/telive
#   https://github.com/sq5bpf/osmo-tetra-sq5bpf-2
# ---------------------------------------------------------------------------

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-parameter -std=c99")

if(UNIX AND NOT APPLE)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|ARM|aarch64|AARCH64")
        # ARM/AArch64: no -msse4.1
    else()
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msse4.1")
    endif()
endif()

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -ggdb3")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
        "Build type: Debug Release RelWithDebInfo MinSizeRel" FORCE)
endif()

# --- Dependencies ---

# pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# librtlsdr
find_package(PkgConfig REQUIRED)
pkg_check_modules(RTLSDR REQUIRED librtlsdr)

# liquid-dsp (no pkg-config; find manually)
find_path(LIQUID_INCLUDE_DIR liquid/liquid.h
    PATHS /usr/include /usr/local/include)
find_library(LIQUID_LIBRARY
    NAMES liquid
    PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)

if(NOT LIQUID_INCLUDE_DIR OR NOT LIQUID_LIBRARY)
    message(FATAL_ERROR "liquid-dsp not found. Install: sudo apt install libliquid-dev")
endif()
message(STATUS "liquid-dsp: ${LIQUID_LIBRARY}")

# --- Sources ---
set(SOURCES
    main.c
    rtlsdr_source.c
    rtltcp_source.c
    dqpsk_demod.c
    spectrum.c
    web_server.c
    scanner.c
    tetra_decode.c
    ascii_display.c
    channel_mgr.c
)

add_executable(tetra-rtlsdr ${SOURCES})

target_include_directories(tetra-rtlsdr PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${RTLSDR_INCLUDE_DIRS}
    ${LIQUID_INCLUDE_DIR}
)

target_link_libraries(tetra-rtlsdr PRIVATE
    Threads::Threads
    m
    ${RTLSDR_LIBRARIES}
    ${LIQUID_LIBRARY}
)

set_property(TARGET tetra-rtlsdr PROPERTY C_STANDARD 99)

install(TARGETS tetra-rtlsdr DESTINATION bin)
