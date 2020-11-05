#!/usr/bin/env bash

# from basalt
MYPWD=$(pwd)

set -x
set -e

BUILD_TYPE=RelWithDebInfo

if [ -n "$1" ]; then
BUILD_TYPE=$1
fi

BASEDIR="$PWD"
CERES_VERSION="1.14.0"

# https://stackoverflow.com/a/45181694
NUM_CORES=`getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu || echo 1`
NUM_PROC=2 

NUM_PARALLEL_BUILDS=$NUM_CORES

BUILD_PANGOLIN=3rdParty/build-pangolin

rm -rf "$BUILD_PANGOLIN"

mkdir -p "$BUILD_PANGOLIN"
pushd "$BUILD_PANGOLIN"
cmake -D CMAKE_INSTALL_PREFIX=$MYPWD/3rdParty ../Pangolin
make -j$NUM_PARALLEL_BUILDS install
popd

rm -rf "$BUILD_PANGOLIN"

cd "$BASEDIR/3rdParty"
rm -rf ceres-solver

# fast access for mainland user
git clone https://gitee.com/wlt027/ceres-solver.git

cd "$BASEDIR/3rdParty/ceres-solver"
rm -rf build
git checkout tags/${CERES_VERSION}
mkdir build
cd build
cmake -D CMAKE_INSTALL_PREFIX=$BASEDIR/3rdParty -DBUILD_SHARED_LIBS=ON ..
make -j$NUM_PROC install
cd "$BASEDIR/3rdParty/ceres-solver"
rm -rf build