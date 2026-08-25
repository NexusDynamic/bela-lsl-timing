#!/usr/bin/env bash
# Cross-compile liblsl for the Bela (BeagleBone Black, armv7-a/NEON, hard float)
# on the host, using the same linaro/crosstool-NG toolchain that Bela's own
# cross-compilation uses. No packages are installed on the board.
#
# Usage:
#   cross/build_liblsl.sh [path-to-liblsl-source] [--install]
#
# Without --install the result is left in cross/build/ for inspection; with it,
# the stripped library and the matching public headers are copied into lib/ and
# include/ of this project.
#
# The toolchain's g++ is 6.3.1, which is a C++11/14 front-end with partial
# C++17. liblsl 1.17 uses three things it does not have, so cross/gcc6-compat.patch
# is applied to a scratch copy of the source (the source tree itself is never
# modified):
#   * `inline` static data members       -> definitions moved to api_config.cpp
#   * `if constexpr`                     -> tag dispatch on std::is_same
#   * std::string_view for pugixml 1.15  -> falls back to the const char* overload
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(dirname "$HERE")"

LSL_SRC="${1:-$HOME/github_projects/liblsl.dart/packages/liblsl/src/liblsl-dart_main}"
INSTALL=0
for arg in "$@"; do [ "$arg" = "--install" ] && INSTALL=1; done

TC=/usr/local/linaro/arm-bela-linux-gnueabihf
STRIP="$TC/bin/arm-bela-linux-gnueabihf-strip"

[ -d "$TC" ] || { echo "Bela toolchain not found at $TC (run SyncBelaSysroot.sh)"; exit 1; }
[ -f "$LSL_SRC/CMakeLists.txt" ] || { echo "No liblsl source at $LSL_SRC"; exit 1; }

WORK="$HERE/build"
SRC="$WORK/src"
BUILD="$WORK/obj"
rm -rf "$WORK"
mkdir -p "$SRC"

echo "==> staging source from $LSL_SRC"
# NB: the exclude patterns are anchored so they do not swallow src/buildinfo.cpp.
rsync -a --exclude '/build/' --exclude '.git/' "$LSL_SRC/" "$SRC/"

echo "==> applying GCC 6 compatibility patch"
patch -p1 -d "$SRC" < "$HERE/gcc6-compat.patch"

# The staged copy has no .git, so pull the revision from the real source tree and
# hand it to CMake -- otherwise lsl_library_info() reports an empty git string and
# you cannot tell which build is on the board.
GITREV="$(git -C "$LSL_SRC" describe --tags HEAD 2>/dev/null || echo unknown)"
GITBRANCH="$(git -C "$LSL_SRC" rev-parse --symbolic-full-name --abbrev-ref @ 2>/dev/null || echo unknown)"

echo "==> configuring ($GITBRANCH/$GITREV)"
# CMAKE_CXX_STANDARD_LIBRARIES appends -ldl at the end of the link line: loguru
# calls dladdr for stack traces, and on the board's glibc (2.24) dladdr still
# lives in libdl rather than libc. liblsl only links dl when LSL_DEBUGLOG is on.
cmake -S "$SRC" -B "$BUILD" -G Ninja \
	-DCMAKE_TOOLCHAIN_FILE="$HERE/bela-armhf.cmake" \
	-DCMAKE_BUILD_TYPE=Release \
	-Dlslgitrevision="$GITREV" \
	-Dlslgitbranch="$GITBRANCH" \
	-DCMAKE_CXX_STANDARD_LIBRARIES=-ldl \
	-DLSL_OPTIMIZATIONS=OFF \
	-DLSL_UNITTESTS=OFF \
	-DLSL_TOOLS=OFF \
	-DLSL_INSTALL=OFF

echo "==> building"
cmake --build "$BUILD" -j"$(getconf _NPROCESSORS_ONLN)"

SO="$(ls "$BUILD"/liblsl.so.*.*.* | head -1)"
cp "$SO" "$WORK/liblsl.so"
"$STRIP" --strip-unneeded "$WORK/liblsl.so"

echo
echo "Built: $WORK/liblsl.so"
file "$WORK/liblsl.so"

if [ "$INSTALL" = "1" ]; then
	echo "==> installing into $PROJECT/lib and $PROJECT/include"
	cp "$WORK/liblsl.so" "$PROJECT/lib/liblsl.so"
	# The library carries SONAME liblsl.so.2, so the runtime loader looks for
	# that name rather than the path passed to the linker. Ship both names and
	# make sure lib/ is on the rpath (see build.sh).
	cp "$WORK/liblsl.so" "$PROJECT/lib/liblsl.so.2"
	cp "$SRC"/include/lsl_c.h "$SRC"/include/lsl_cpp.h "$PROJECT/include/"
	rm -rf "$PROJECT/include/lsl" && cp -R "$SRC/include/lsl" "$PROJECT/include/lsl"
	echo "Installed liblsl $GITBRANCH/$GITREV into lib/ and include/"
fi
