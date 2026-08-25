# CMake toolchain file: cross-compile for the Bela (BeagleBone Black, Cortex-A8,
# armv7-a + NEON, hard float) using the official Bela linaro/crosstool-NG
# toolchain installed by SyncBelaSysroot.sh.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(BELA_TC /usr/local/linaro/arm-bela-linux-gnueabihf)
set(CMAKE_C_COMPILER   ${BELA_TC}/bin/arm-bela-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER ${BELA_TC}/bin/arm-bela-linux-gnueabihf-g++)
set(CMAKE_SYSROOT      ${BELA_TC}/arm-bela-linux-gnueabihf/sysroot)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(_bela_arch "-march=armv7-a -mtune=cortex-a8 -mfpu=neon -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT   "${_bela_arch}")
set(CMAKE_CXX_FLAGS_INIT "${_bela_arch}")
