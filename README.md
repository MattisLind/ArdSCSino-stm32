# ArdSCSino-stm32

The idea and the base code was taken from a project created by Tambo (TNB Seisakusho) (https://twitter.com/h_koma2) ArdSCSino is hardware that reproduces SCSI devices (hard disks) with arduino.

### Setup

I am using Arduino Software (IDE) V1.8.13.
Tools-> Boards-> Board Manager-> Search Filters Search and install "Arduino SAM board (32-bit ARM Cortex-M3)" Select the board "Generic STM32F103C series"

I am using the following as a library.

SDFAT (https://github.com/greiman/SdFat/tree/1.1.4) 

* Make sure to use V1 of SdFat (for example V1.1.4 )
* Make sure to set the #define ENABLE_EXTENDED_TRANSFER_CLASS 1 in SdFatConfig.h

Arduino_STM32 (https://github.com/rogerclarkmelbourne/Arduino_STM32/releases/tag/v1.0.0)

## Enhancements

In the STM32SCSISD folder there is a new PCB which integrates the following features:
* Floppy / AMP / TE connectivity connector.
* Pin headers for serial port, USB, SWD connectors. The use of these are for debugging only and is purely optional.
* Jumpers for RESET and BOOT. Also only needed for debuging.
* Jumpers for and image select. The image select feature allow for selecting one out of four images on the card when the SCSI device is selected.
* Jumpers for unit select. Allow the settig of the SCSI ID. The ID is also used to select one of seven sets of images as defined by the image select above.
* Activity LED pin header.
* Mounting holes.



The firmware (in STM32SCSISD_FW) has been improved by optimizing how data is transfered to the SCSI port. One SCSI transaction is now 1.2 microsseconds and the innerloop consists of 30 instructions. Please make sure that you use -O3 optimizations when compiling the project.

```
arm-none-eabi-g++ -mcpu=cortex-m3 -mthumb ArdSCSinoV2.cpp ../../SdFat/src/FatLib/FatFile.cpp  -I../../Arduino_STM32/STM32F1/libraries/SPI/src/ -I../../Arduino_STM32/STM32F1/system/libmaple/include/ -I../../Arduino_STM32/STM32F1/system/libmaple/ -I../../Arduino_STM32/STM32F1/cores/maple/ -I../../Arduino_STM32/STM32F1/variants/generic_stm32f103c -DMCU_STM32F103C8 -I../../Arduino_STM32/STM32F1/system/libmaple/stm32f1/include/ -I../../SdFat/src/ -DARDUINO -DF_CPU=72000000 -D__arm__ -D__STM32F1__ -Dnullptr=NULL -O3
```

Real life copying of 5 Mbyte file under RT-11 operating system running on a PDP-11/23 CPU takes 23 seconds which is equivalent of 218kbytes/s copying speed. Comparing this with the SCSI2SD v5 adapter gives a 305kbytes/s copying speed. 

The design has also been tested successfully with a Luxor ABC1600 computer.
