#!/bin/bash
###############################################################################
#
#                           Kernel Build Script 
#
###############################################################################
# 2011-10-24 effectivesky : modified
# 2010-12-29 allydrop     : created
###############################################################################
##############################################################################
# set toolchain
##############################################################################
export ARCH=arm
export CCACHE_COMPRESS=1
export CCACHE_MAXSIZE=500M
export CCACHE_DIR="$PWD/obj/ccache"
mkdir -p "$CCACHE_DIR"
export CROSS_COMPILE="$PWD/../android-4.4.4_r1-prebuilt/prebuilts/misc/linux-x86/ccache/ccache $PWD/../android-4.4.4_r1-prebuilt/prebuilts/gcc/linux-x86/arm/arm-eabi-4.7/bin/arm-eabi-"
# export LINUX_BIN_PATH=$PWD/obj
# rm -rf $LINUX_BIN_PATH
CMD_V_LOG_FILE=$PWD/obj/KERNEL_build.log
rm -rf $CMD_V_LOG_FILE

##############################################################################
# make zImage
##############################################################################
KERNEL_CONFIG=cyanogenmod_presto_defconfig
if ! [ -z "$1" ]; then
  KERNEL_CONFIG="$1"
fi
mkdir -p ./obj/KERNEL_OBJ/
make O=./obj/KERNEL_OBJ "$KERNEL_CONFIG"
#make O=./obj/KERNEL_OBJ menuconfig
make -j2 O=./obj/KERNEL_OBJ 2>&1 | tee $CMD_V_LOG_FILE
