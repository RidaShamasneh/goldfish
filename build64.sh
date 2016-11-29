export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
make ranchu_defconfig
make -j4
