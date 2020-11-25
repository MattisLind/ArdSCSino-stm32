# ArdSCSino-stm32

Translation stolen from Eric Helgeson.

ArdSCSino-stm32 is the STM32 version of ArdSCSino created by Tambo (TNB Seisakusho) (https://twitter.com/h_koma2) ArdSCSino is hardware that reproduces SCSI devices (hard disks) with arduino. We will publish it with permission.

#Setup

I am using Arduino Software (IDE) V1.8.8.
Tools-> Boards-> Board Manager-> Search Filters Search and install "Arduino SAM board (32-bit ARM Cortex-M3)" Select the board "Generic STM32F103C series"

I am using the following as a library.

SDFAT (https://github.com/greiman/SdFat/tree/1.1.4) 

## Make sure to use V1 of SdFat (for example V1.1.4 )
## Make sure to set the #define ENABLE_EXTENDED_TRANSFER_CLASS 1 in SdFatConfig.h

Arduino_STM32 (https://github.com/rogerclarkmelbourne/Arduino_STM32/releases/tag/v1.0.0)

I am using the following as a micro SD CARD adapter.

ArdSCSIno V1
Arduino SPI Micro SD Adapter 6PIN Compatible Micro SD TF Card Memory Shield Module

ArdSCSIno V2
Hirose DM3AT-SF-PEJM5


ArdSCSino-stm32 とは たんぼ（TNB製作所）(https://twitter.com/h_koma2) さんが作成した ArdSCSino のSTM32版です<br>
ArdSCSino とは SCSIデバイス（ハードディスク）を arduino で再現するハードウエアです。<br>
許可を頂いて公開することになりました。<br>

# Setup
* Arduino Software (IDE) V1.8.8 を使用しています。<br>
 ライブラリとしてSDFAT (https://github.com/greiman/SdFat) を使用しています。 <br>

