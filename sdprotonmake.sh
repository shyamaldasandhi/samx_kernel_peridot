#!/bin/bash

# A script to build a custom Android kernel with Proton Clang.
#
# Copyright (C) 2025 Your Name
#
# Licensed under the GNU General Public License, version 2.0 (the "License")

# --- ADJUST THESE VARIABLES ---
# Path to your Proton Clang toolchain
export CLANG_PATH="$HOME/tc/protonclang"

# The name of your device's defconfig file
export DEFCONFIG="peridot_defconfig"

# The name for the output ZIP file
export ZIP_NAME="SamXKernel-PocoF6-$(date +%Y%m%d-%H%M).zip"
# ------------------------------


# --- DO NOT EDIT BELOW THIS LINE (unless you know what you're doing) ---

# Set up environment variables
export ARCH="arm64"
export SUBARCH="arm64"
export KBUILD_BUILD_USER="SAM" # Change to your name/username
export KBUILD_BUILD_HOST="i5" # Change to your PC's hostname
export TZ=":Asia/Kolkata" # Set your timezone

# --- IMPORTANT: Set HOSTCC and HOSTLD to use your system's native compiler/linker ---
# Get absolute paths for host tools to ensure they are used.
# **YOU MUST VERIFY THESE PATHS ON YOUR SYSTEM FIRST!**
# Run: which gcc, which g++, which ld, which ar, which strip, which objcopy, which objdump
# in your terminal and update the paths below accordingly.

export SYSTEM_GCC="$(which gcc)"
export SYSTEM_GPP="$(which g++)"
export SYSTEM_LD="$(which ld)"
export SYSTEM_AR="$(which ar)"
export SYSTEM_STRIP="$(which strip)"
export SYSTEM_OBJCOPY="$(which objcopy)"
export SYSTEM_OBJDUMP="$(which objdump)"

# Verify the paths are found
if [ -z "$SYSTEM_GCC" ] || [ -z "$SYSTEM_LD" ]; then
    echo "ERROR: Could not find system 'gcc' or 'ld' in your PATH."
    echo "Please ensure build-essential (or equivalent) is installed."
    exit 1
fi

export HOSTCC="${SYSTEM_GCC}"
export HOSTCXX="${SYSTEM_GPP}"
export HOSTLD="${SYSTEM_LD}"
export HOSTAR="${SYSTEM_AR}"
export HOSTSTRIP="${SYSTEM_STRIP}"
export HOSTOBJCOPY="${SYSTEM_OBJCOPY}"
export HOSTOBJDUMP="${SYSTEM_OBJDUMP}"

# Compiler and Linker flags for the TARGET (Android kernel)
# These will be set LATER, after HOST tools are compiled.
export CC_TARGET="clang"
export CLANG_TRIPLE="aarch64-linux-gnu-"
export CXX_TARGET="clang++"
export LD_TARGET="ld.lld"
export AR_TARGET="llvm-ar"
export NM_TARGET="llvm-nm"
export STRIP_TARGET="llvm-strip"
export OBJCOPY_TARGET="llvm-objcopy"
export OBJDUMP_TARGET="llvm-objdump"
export READELF_TARGET="llvm-readelf"

# Clean up previous builds
echo "========================================"
echo "         CLEANING BUILD"
echo "========================================"
make mrproper
rm -rf out

# Start the timer
BUILD_START=$(date +"%s")
echo "========================================"
echo "         STARTING BUILD"
echo "========================================"

# Step 1: Configure and build host tools *before* adding clang to PATH
echo "Building host tools..."
# Temporarily remove clang from PATH if it was added earlier in a shell session
ORIGINAL_PATH="$PATH"
export PATH=$(echo $PATH | sed -e "s|${CLANG_PATH}/bin:||g") # Remove clang path if present

make O=out ${DEFCONFIG} \
    HOSTCC="${HOSTCC}" \
    HOSTLD="${HOSTLD}" \
    HOSTCXX="${HOSTCXX}" \
    HOSTAR="${HOSTAR}" \
    HOSTSTRIP="${HOSTSTRIP}" \
    HOSTOBJCOPY="${HOSTOBJCOPY}" \
    HOSTOBJDUMP="${HOSTOBJDUMP}" \
    scripts_basic # Only build scripts_basic first to test host tool compilation

# Check if host tools built successfully
if [ ! -f "out/scripts/basic/fixdep" ]; then
    echo "!!! HOST TOOL BUILD FAILED. Check the error log."
    exit 1
fi
echo "Host tools compiled successfully."

# Step 2: Add Clang path to the front of the PATH for kernel compilation
export PATH="${CLANG_PATH}/bin:${ORIGINAL_PATH}" # Restore original path and prepend clang

# Now set the target compiler variables for the main kernel build
export CC="${CC_TARGET}"
export CXX="${CXX_TARGET}"
export LD="${LD_TARGET}"
export AR="${AR_TARGET}"
export NM="${NM_TARGET}"
export STRIP="${STRIP_TARGET}"
export OBJCOPY="${OBJCOPY_TARGET}"
export OBJDUMP="${OBJDUMP_TARGET}"
export READELF="${READELF_TARGET}"

# Continue with the full kernel build
echo "Continuing with full kernel build..."
make -j$(nproc --all) \
    O=out \
    HOSTCC="${HOSTCC}" \
    HOSTLD="${HOSTLD}" \
    HOSTCXX="${HOSTCXX}" \
    HOSTAR="${HOSTAR}" \
    HOSTSTRIP="${HOSTSTRIP}" \
    HOSTOBJCOPY="${HOSTOBJCOPY}" \
    HOSTOBJDUMP="${HOSTOBJDUMP}"

# Check if the build was successful
if [ ! -f "out/arch/arm64/boot/Image" ]; then
    echo "!!! KERNEL BUILD FAILED. Check the error log."
    exit 1
fi

echo "========================================"
echo "         BUILD SUCCESSFUL"
echo "========================================"

# Package into a flashable ZIP using AnyKernel3 (Recommended)
echo "Packaging into a flashable ZIP..."
git clone https://github.com/osm0sis/AnyKernel3.git --depth=1 AnyKernel
cp out/arch/arm64/boot/Image AnyKernel/
cd AnyKernel
zip -r9 ../${ZIP_NAME} * -x .git README.md *zip

# Go back to root
cd ..
rm -rf AnyKernel

# End the timer and calculate build time
BUILD_END=$(date +"%s")
DIFF=$(($BUILD_END - $BUILD_START))

echo "========================================"
echo "         FLASHABLE ZIP CREATED"
echo "========================================"
echo "ZIP Location: $(pwd)/${ZIP_NAME}"
echo "Build completed in $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) second(s)."
