#!/usr/bin/env bash

# PWD="/home/sd/tc"
# LINERO13="linero13"
# LINERO14="linero14"
# DEFCONFIG="peridot_defconfig"

# export TOOLCHAIN_PATH=$PWD/$LINERO14/bin
# export PATH=$TOOLCHAIN_PATH:$PATH
# export CROSS_COMPILE=aarch64-none-linux-gnu-
# export ARCH=arm64

# make clean && make mrproper

# make ARCH=arm64 CROSS_COMPILE=aarch64-none-linux-gnu- $DEFCONFIG

# make -j$(nproc) ARCH=arm64 CROSS_COMPILE=aarch64-none-linux-gnu-



# Configurations
PWD="/home/sd/tc"
LINERO13="linero13" #outdated does not support -mcpu=a720 (peridot cortex architecture use for old cortex)
LINERO14="linero14"
DEFCONFIG="peridot_defconfig"
OUT_DIR="$PWD/out"  # Output directory

# Toolchain setup
export TOOLCHAIN_PATH="$PWD/$LINERO14/bin"
export PATH="$TOOLCHAIN_PATH:$PATH"
export CROSS_COMPILE="aarch64-none-linux-gnu-"
export ARCH="arm64"

# Clean previous builds
make clean && make mrproper

# Create output directory
mkdir -p "$OUT_DIR"

# Configure kernel (output to out/)
make O="$OUT_DIR" ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE" "$DEFCONFIG"

# Build kernel (output to out/)
make -j$(nproc) O="$OUT_DIR" ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE"