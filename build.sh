#!/bin/bash

# Written by antdking <anthonydking@gmail.com>
# credits to Rashed for the base of zip making
# credits to the internet for filling in else where

echo "this is an open source script, feel free to use and share it"

daytime=$(date +%d"-"%m"-"%Y"_"%H":"%M)

location=.
vendor=lge

if [ -z $target ]; then
echo "choose your target device"
echo "1) p350"
echo "2) p500"
echo "3) p505"
echo "4) p506"
echo "5) p509"
read -p "1/2/3/4/5: " choice
case "$choice" in
1 ) export target=p350 ; export defconfig=cyanogenmod_p350_defconfig;;
2 ) export target=p500 ; export defconfig=cyanogenmod_p500_p509_defconfig;;
3 ) export target=p505 ; export defconfig=cyanogenmod_p505_p506_defconfig;;
4 ) export target=p506 ; export defconfig=cyanogenmod_p505_p506_defconfig;;
5 ) export target=p509 ; export defconfig=cyanogenmod_p500_p509_defconfig;;
* ) echo "invalid choice"; sleep 2 ; ./build.sh;;
esac
fi # [ -z $target ]

if [ -z $compiler ]; then
if [ -f ../arm-eabi-4.6/bin/arm-eabi-* ]; then
export compiler=../arm-eabi-4.6/bin/arm-eabi-
elif [ -f arm-eabi-4.6/bin/arm-eabi-* ]; then # [ -f ../arm-eabi-4.6/bin/arm-eabi-* ]
export compiler=arm-eabi-4.6/bin/arm-eabi-
else # [ -f arm-eabi-4.6/bin/arm-eabi-* ]
echo "please specify a location, including the '/bin/arm-eabi-' at the end "
read compiler
fi # [ -z $compiler ]
fi # [ -f ../arm-eabi-4.6/bin/arm-eabi-* ]

cd $location
export ARCH=arm
export CROSS_COMPILE=$compiler
if [ -z "$clean" ]; then
read -p "do make clean mrproper?(y/n)" clean
fi # [ -z "$clean" ]
case "$clean" in
y|Y ) echo "cleaning..."; make clean mrproper;;
n|N ) echo "continuing...";;
* ) echo "invalid option"; sleep 2 ; build.sh;;
esac

echo "now building the kernel"

make $defconfig
make -j `cat /proc/cpuinfo | grep "^processor" | wc -l` "$@"

## the zip creation
if [ -f arch/arm/boot/zImage ]; then

rm -f zip-creator/kernel/zImage
rm -rf zip-creator/system/

mkdir zip-creator/system
mkdir zip-creator/system/lib
mkdir zip-creator/system/lib/modules

cp arch/arm/boot/zImage zip-creator/kernel
case "$target" in
p500|p505|p506|p509 ) cp drivers/net/wireless/bcm4325/wireless.ko zip-creator/system/lib/modules/;;
p350 ) cp drivers/net/wireless/bcm4329/wireless.ko zip-creator/system/lib/modules/;;
* ) echo "your device isn't supported for wireless. why are you here!"
esac
cp drivers/scsi/scsi_wait_scan.ko zip-creator/system/lib/modules
cp arch/arm/common/cpaccess.ko zip-creator/system/lib/modules
cp drivers/mmc/card/mmc_test.ko zip-creator/system/lib/modules

zipfile="lge-3.0.x-$target-$daytime.zip"
cd zip-creator
rm -f *.zip
zip -r $zipfile *

echo "zip saved to $zipfile"

else # [ -f arch/arm/boot/zImage ]
echo "the build failed so a zip won't be created"
fi # [ -f arch/arm/boot/zImage ]

