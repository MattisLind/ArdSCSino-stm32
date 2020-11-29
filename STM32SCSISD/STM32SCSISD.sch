EESchema Schematic File Version 4
LIBS:STM32SCSISD-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_ST_STM32F1:STM32F103C8Tx U1
U 1 1 5FC2BBEF
P 5750 3100
F 0 "U1" H 5200 1650 50  0000 C CNN
F 1 "STM32F103C8Tx" H 5150 4550 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 5150 1700 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00161566.pdf" H 5750 3100 50  0001 C CNN
	1    5750 3100
	1    0    0    -1  
$EndComp
$Comp
L Connector:Micro_SD_Card_Det_Hirose_DM3AT J7
U 1 1 5FC2BC71
P 9150 4400
F 0 "J7" H 9100 5217 50  0000 C CNN
F 1 "Micro_SD_Card_Det_Hirose_DM3AT" H 9100 5126 50  0000 C CNN
F 2 "" H 11200 5100 50  0001 C CNN
F 3 "https://www.hirose.com/product/en/download_file/key_name/DM3/category/Catalog/doc_file_id/49662/?file_category_id=4&item_id=195&is_series=1" H 9150 4500 50  0001 C CNN
	1    9150 4400
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x25_Odd_Even J4
U 1 1 5FC2C959
P 1250 2200
F 0 "J4" H 1300 3617 50  0000 C CNN
F 1 "SCSI" H 1300 3526 50  0000 C CNN
F 2 "" H 1250 2200 50  0001 C CNN
F 3 "~" H 1250 2200 50  0001 C CNN
	1    1250 2200
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J1
U 1 1 5FC2CA34
P 1350 5050
F 0 "J1" H 1456 5328 50  0000 C CNN
F 1 "SERIAL_DEBUG" H 1456 5237 50  0000 C CNN
F 2 "" H 1350 5050 50  0001 C CNN
F 3 "~" H 1350 5050 50  0001 C CNN
	1    1350 5050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J2
U 1 1 5FC2CACC
P 1450 5900
F 0 "J2" H 1556 6178 50  0000 C CNN
F 1 "SWD" H 1556 6087 50  0000 C CNN
F 2 "" H 1450 5900 50  0001 C CNN
F 3 "~" H 1450 5900 50  0001 C CNN
	1    1450 5900
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J3
U 1 1 5FC2CB36
P 1500 6850
F 0 "J3" H 1606 7128 50  0000 C CNN
F 1 "USB" H 1606 7037 50  0000 C CNN
F 2 "" H 1500 6850 50  0001 C CNN
F 3 "~" H 1500 6850 50  0001 C CNN
	1    1500 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:Jumper JP5
U 1 1 5FC2CC3D
P 3750 2700
F 0 "JP5" H 3750 2964 50  0000 C CNN
F 1 "BOOT" H 3750 2873 50  0000 C CNN
F 2 "" H 3750 2700 50  0001 C CNN
F 3 "~" H 3750 2700 50  0001 C CNN
	1    3750 2700
	0    -1   -1   0   
$EndComp
$Comp
L Device:Jumper JP1
U 1 1 5FC2CEC5
P 6450 5450
F 0 "JP1" H 6450 5400 50  0000 C CNN
F 1 "UNITSEL0" H 6450 5623 50  0000 C CNN
F 2 "" H 6450 5450 50  0001 C CNN
F 3 "~" H 6450 5450 50  0001 C CNN
	1    6450 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:Jumper JP2
U 1 1 5FC2D01B
P 6450 5750
F 0 "JP2" H 6450 5700 50  0000 C CNN
F 1 "UNITSEL1" H 6450 5923 50  0000 C CNN
F 2 "" H 6450 5750 50  0001 C CNN
F 3 "~" H 6450 5750 50  0001 C CNN
	1    6450 5750
	1    0    0    -1  
$EndComp
$Comp
L Device:Jumper JP3
U 1 1 5FC2D04B
P 6450 6050
F 0 "JP3" H 6450 6000 50  0000 C CNN
F 1 "UNITSEL2" H 6450 6223 50  0000 C CNN
F 2 "" H 6450 6050 50  0001 C CNN
F 3 "~" H 6450 6050 50  0001 C CNN
	1    6450 6050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J5
U 1 1 5FC2D24F
P 3850 5800
F 0 "J5" H 3956 5978 50  0000 C CNN
F 1 "ACTIVITYLED" H 3956 5887 50  0000 C CNN
F 2 "" H 3850 5800 50  0001 C CNN
F 3 "~" H 3850 5800 50  0001 C CNN
	1    3850 5800
	1    0    0    -1  
$EndComp
$Comp
L Device:Jumper JP4
U 1 1 5FC2D504
P 3850 1100
F 0 "JP4" H 3850 1364 50  0000 C CNN
F 1 "TERMPWR_SUPPLY" H 3850 1273 50  0000 C CNN
F 2 "" H 3850 1100 50  0001 C CNN
F 3 "~" H 3850 1100 50  0001 C CNN
	1    3850 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5FC2D694
P 800 3500
F 0 "#PWR01" H 800 3250 50  0001 C CNN
F 1 "GND" H 805 3327 50  0000 C CNN
F 2 "" H 800 3500 50  0001 C CNN
F 3 "" H 800 3500 50  0001 C CNN
	1    800  3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  3500 800  3400
Wire Wire Line
	800  3400 1050 3400
Wire Wire Line
	800  3400 800  3300
Wire Wire Line
	800  3300 1050 3300
Connection ~ 800  3400
Wire Wire Line
	800  3200 1050 3200
Connection ~ 800  3300
Wire Wire Line
	1050 3100 800  3100
Wire Wire Line
	800  3100 800  3200
Connection ~ 800  3200
Wire Wire Line
	800  3200 800  3300
Wire Wire Line
	1050 3000 800  3000
Connection ~ 800  3100
Wire Wire Line
	1050 2900 800  2900
Wire Wire Line
	800  2900 800  3000
Connection ~ 800  3000
Wire Wire Line
	800  3000 800  3100
Wire Wire Line
	1050 1000 800  1000
Wire Wire Line
	800  1000 800  1100
Connection ~ 800  2900
Wire Wire Line
	1050 2800 800  2800
Connection ~ 800  2800
Wire Wire Line
	800  2800 800  2900
Wire Wire Line
	1050 2700 800  2700
Connection ~ 800  2700
Wire Wire Line
	800  2700 800  2800
Wire Wire Line
	1050 2600 800  2600
Connection ~ 800  2600
Wire Wire Line
	800  2600 800  2700
Wire Wire Line
	1050 2500 800  2500
Connection ~ 800  2500
Wire Wire Line
	800  2500 800  2600
Wire Wire Line
	1050 2400 800  2400
Connection ~ 800  2400
Wire Wire Line
	800  2400 800  2500
Wire Wire Line
	1050 2300 800  2300
Connection ~ 800  2300
Wire Wire Line
	800  2300 800  2400
Wire Wire Line
	1050 2100 800  2100
Connection ~ 800  2100
Wire Wire Line
	800  2100 800  2200
Wire Wire Line
	1050 2000 800  2000
Connection ~ 800  2000
Wire Wire Line
	800  2000 800  2100
Wire Wire Line
	1050 1900 800  1900
Connection ~ 800  1900
Wire Wire Line
	800  1900 800  2000
Wire Wire Line
	1050 1800 800  1800
Connection ~ 800  1800
Wire Wire Line
	800  1800 800  1900
Wire Wire Line
	1050 1700 800  1700
Connection ~ 800  1700
Wire Wire Line
	800  1700 800  1800
Wire Wire Line
	1050 1600 800  1600
Connection ~ 800  1600
Wire Wire Line
	800  1600 800  1700
Wire Wire Line
	1050 1500 800  1500
Connection ~ 800  1500
Wire Wire Line
	800  1500 800  1600
Wire Wire Line
	1050 1100 800  1100
Connection ~ 800  1100
Wire Wire Line
	800  1100 800  1200
Wire Wire Line
	1050 1200 800  1200
Connection ~ 800  1200
Wire Wire Line
	800  1200 800  1300
Wire Wire Line
	1050 1300 800  1300
Connection ~ 800  1300
Wire Wire Line
	800  1300 800  1400
Wire Wire Line
	1050 1400 800  1400
Connection ~ 800  1400
Wire Wire Line
	800  1400 800  1500
Wire Wire Line
	1550 1000 2000 1000
Text Label 1550 1000 0    50   ~ 0
SCSI_DB0
Wire Wire Line
	1550 1100 2000 1100
Wire Wire Line
	1550 1200 2000 1200
Wire Wire Line
	1550 1300 2000 1300
Wire Wire Line
	1550 1400 2000 1400
Wire Wire Line
	1550 1600 2000 1600
Wire Wire Line
	1550 1700 2000 1700
Wire Wire Line
	1550 1800 2000 1800
Text Label 1550 1100 0    50   ~ 0
SCSI_DB1
Text Label 1550 1200 0    50   ~ 0
SCSI_DB2
Text Label 1550 1300 0    50   ~ 0
SCSI_DB3
Text Label 1550 1400 0    50   ~ 0
SCSI_DB4
Text Label 1550 1500 0    50   ~ 0
SCSI_DB5
Wire Wire Line
	1550 1500 2000 1500
Text Label 1550 1600 0    50   ~ 0
SCSI_DB6
Text Label 1550 1700 0    50   ~ 0
SCSI_DB7
Text Label 1550 1800 0    50   ~ 0
SCSI_DBP
Text Label 1600 2200 0    50   ~ 0
TERMPWR
Wire Wire Line
	1550 2500 2000 2500
Wire Wire Line
	1550 2700 2000 2700
Wire Wire Line
	1550 2800 2000 2800
Wire Wire Line
	1550 2900 2000 2900
Wire Wire Line
	1550 3000 2000 3000
Wire Wire Line
	1550 3100 2000 3100
Wire Wire Line
	1550 3200 2000 3200
Wire Wire Line
	1550 3300 2000 3300
Wire Wire Line
	1550 3400 2000 3400
Text Label 1600 2500 0    50   ~ 0
SCSI_ATN
Text Label 1600 2700 0    50   ~ 0
SCSI_BSY
Text Label 1600 2800 0    50   ~ 0
SCSI_ACK
Text Label 1600 2900 0    50   ~ 0
SCSI_RST
Text Label 1600 3000 0    50   ~ 0
SCSI_MSG
Text Label 1600 3100 0    50   ~ 0
SCSI_SEL
Text Label 1600 3200 0    50   ~ 0
SCSI_CD
Text Label 1600 3300 0    50   ~ 0
SCSI_REQ
Text Label 1600 3400 0    50   ~ 0
SCSI_IO
$Comp
L power:GND #PWR02
U 1 1 5FC4747B
P 2250 3500
F 0 "#PWR02" H 2250 3250 50  0001 C CNN
F 1 "GND" H 2255 3327 50  0000 C CNN
F 2 "" H 2250 3500 50  0001 C CNN
F 3 "" H 2250 3500 50  0001 C CNN
	1    2250 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 3500 2250 2600
Wire Wire Line
	2250 1900 1550 1900
Wire Wire Line
	1550 2000 2250 2000
Connection ~ 2250 2000
Wire Wire Line
	2250 2000 2250 1900
Wire Wire Line
	1550 2100 2250 2100
Connection ~ 2250 2100
Wire Wire Line
	2250 2100 2250 2000
Wire Wire Line
	1550 2300 2250 2300
Connection ~ 2250 2300
Wire Wire Line
	2250 2300 2250 2100
Wire Wire Line
	1550 2400 2250 2400
Connection ~ 2250 2400
Wire Wire Line
	2250 2400 2250 2300
Wire Wire Line
	1550 2600 2250 2600
Connection ~ 2250 2600
Wire Wire Line
	2250 2600 2250 2400
Wire Wire Line
	2800 2200 2800 1100
Wire Wire Line
	2800 1100 3550 1100
Wire Wire Line
	1550 2200 2800 2200
$Comp
L power:GND #PWR03
U 1 1 5FC518BB
P 5650 4750
F 0 "#PWR03" H 5650 4500 50  0001 C CNN
F 1 "GND" H 5655 4577 50  0000 C CNN
F 2 "" H 5650 4750 50  0001 C CNN
F 3 "" H 5650 4750 50  0001 C CNN
	1    5650 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 4750 5650 4700
Wire Wire Line
	5650 4700 5550 4700
Wire Wire Line
	5550 4700 5550 4600
Wire Wire Line
	5650 4700 5650 4600
Connection ~ 5650 4700
Wire Wire Line
	5650 4700 5750 4700
Wire Wire Line
	5750 4700 5750 4600
Wire Wire Line
	5750 4700 5850 4700
Wire Wire Line
	5850 4700 5850 4600
Connection ~ 5750 4700
$Comp
L Regulator_Linear:MIC5504-3.3YM5 U2
U 1 1 5FC5911B
P 9050 2450
F 0 "U2" H 9050 2817 50  0000 C CNN
F 1 "MIC5504-3.3YM5" H 9050 2726 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 9050 2050 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/MIC550X.pdf" H 8800 2700 50  0001 C CNN
	1    9050 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:Jumper JP6
U 1 1 5FC5923D
P 3100 2100
F 0 "JP6" H 3100 2364 50  0000 C CNN
F 1 "RESET" H 3100 2273 50  0000 C CNN
F 2 "" H 3100 2100 50  0001 C CNN
F 3 "~" H 3100 2100 50  0001 C CNN
	1    3100 2100
	0    -1   -1   0   
$EndComp
$Comp
L Device:Crystal_Small Y1
U 1 1 5FC59341
P 4650 2250
F 0 "Y1" V 4604 2338 50  0000 L CNN
F 1 "8MHz" V 4695 2338 50  0000 L CNN
F 2 "" H 4650 2250 50  0001 C CNN
F 3 "~" H 4650 2250 50  0001 C CNN
	1    4650 2250
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR012
U 1 1 5FC5954C
P 5700 1400
F 0 "#PWR012" H 5700 1250 50  0001 C CNN
F 1 "+3.3V" H 5715 1573 50  0000 C CNN
F 2 "" H 5700 1400 50  0001 C CNN
F 3 "" H 5700 1400 50  0001 C CNN
	1    5700 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 1600 5550 1500
Wire Wire Line
	5550 1500 5650 1500
Wire Wire Line
	5700 1500 5700 1400
Wire Wire Line
	5700 1500 5750 1500
Wire Wire Line
	5950 1500 5950 1600
Connection ~ 5700 1500
Wire Wire Line
	5850 1600 5850 1500
Connection ~ 5850 1500
Wire Wire Line
	5850 1500 5950 1500
Wire Wire Line
	5750 1600 5750 1500
Connection ~ 5750 1500
Wire Wire Line
	5750 1500 5850 1500
Wire Wire Line
	5650 1600 5650 1500
Connection ~ 5650 1500
Wire Wire Line
	5650 1500 5700 1500
$Comp
L Device:R_Small R4
U 1 1 5FC63E34
P 4350 2250
F 0 "R4" H 4409 2296 50  0000 L CNN
F 1 "1M" H 4409 2205 50  0000 L CNN
F 2 "" H 4350 2250 50  0001 C CNN
F 3 "~" H 4350 2250 50  0001 C CNN
	1    4350 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5FC63FB9
P 4350 2500
F 0 "C2" H 4442 2546 50  0000 L CNN
F 1 "22p" H 4400 2400 50  0000 L CNN
F 2 "" H 4350 2500 50  0001 C CNN
F 3 "~" H 4350 2500 50  0001 C CNN
	1    4350 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5FC64045
P 4100 2500
F 0 "C1" H 4192 2546 50  0000 L CNN
F 1 "22p" H 4150 2400 50  0000 L CNN
F 2 "" H 4100 2500 50  0001 C CNN
F 3 "~" H 4100 2500 50  0001 C CNN
	1    4100 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 2300 5050 2350
Wire Wire Line
	5050 2350 4650 2350
Connection ~ 4650 2350
Wire Wire Line
	4650 2350 4350 2350
Wire Wire Line
	4350 2350 4350 2400
Connection ~ 4350 2350
Wire Wire Line
	4100 2400 4100 2150
Wire Wire Line
	4100 2150 4350 2150
Wire Wire Line
	5050 2150 5050 2200
Connection ~ 4350 2150
Wire Wire Line
	4350 2150 4650 2150
Connection ~ 4650 2150
Wire Wire Line
	4650 2150 5050 2150
$Comp
L power:GND #PWR010
U 1 1 5FC6B7CC
P 4350 2700
F 0 "#PWR010" H 4350 2450 50  0001 C CNN
F 1 "GND" H 4355 2527 50  0000 C CNN
F 2 "" H 4350 2700 50  0001 C CNN
F 3 "" H 4350 2700 50  0001 C CNN
	1    4350 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5FC6B7FD
P 4100 2700
F 0 "#PWR09" H 4100 2450 50  0001 C CNN
F 1 "GND" H 4105 2527 50  0000 C CNN
F 2 "" H 4100 2700 50  0001 C CNN
F 3 "" H 4100 2700 50  0001 C CNN
	1    4100 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 2700 4350 2600
Wire Wire Line
	4100 2700 4100 2600
Wire Wire Line
	4550 2500 5050 2500
Text Label 4550 2500 0    50   ~ 0
ACTIVITY_LED
Wire Wire Line
	4550 2600 5050 2600
Text Label 4550 2600 0    50   ~ 0
UNIT_SEL0
Wire Wire Line
	4550 2700 5050 2700
Text Label 4550 2700 0    50   ~ 0
UNIT_SEL1
Wire Wire Line
	5050 3200 4550 3200
Wire Wire Line
	5050 3300 4550 3300
Wire Wire Line
	5050 3400 4550 3400
Wire Wire Line
	5050 3500 4550 3500
Wire Wire Line
	5050 3600 4550 3600
Wire Wire Line
	5050 3700 4550 3700
Wire Wire Line
	5050 3800 4550 3800
Wire Wire Line
	5050 3900 4550 3900
Wire Wire Line
	5050 4000 4550 4000
Wire Wire Line
	5050 4100 4550 4100
Wire Wire Line
	5050 4200 4550 4200
Wire Wire Line
	5050 4300 4550 4300
Wire Wire Line
	5050 4400 4550 4400
Wire Wire Line
	6350 4400 6800 4400
Wire Wire Line
	6350 4300 6800 4300
Wire Wire Line
	6350 4200 6800 4200
Wire Wire Line
	6350 4100 6800 4100
Wire Wire Line
	6350 4000 6800 4000
Wire Wire Line
	6350 3900 6800 3900
Wire Wire Line
	6350 3800 6800 3800
Wire Wire Line
	6350 3700 6800 3700
Wire Wire Line
	6350 3200 6800 3200
Wire Wire Line
	6350 3100 6800 3100
Wire Wire Line
	6350 3000 6800 3000
Wire Wire Line
	6350 2900 6800 2900
Wire Wire Line
	8650 2550 8500 2550
Wire Wire Line
	8500 2550 8500 2350
Wire Wire Line
	8500 2350 8650 2350
Connection ~ 8500 2350
$Comp
L power:GND #PWR017
U 1 1 5FCB70F5
P 9050 2800
F 0 "#PWR017" H 9050 2550 50  0001 C CNN
F 1 "GND" H 9055 2627 50  0000 C CNN
F 2 "" H 9050 2800 50  0001 C CNN
F 3 "" H 9050 2800 50  0001 C CNN
	1    9050 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 2800 9050 2750
$Comp
L power:+3.3V #PWR019
U 1 1 5FCC15F5
P 10000 2350
F 0 "#PWR019" H 10000 2200 50  0001 C CNN
F 1 "+3.3V" H 10015 2523 50  0000 C CNN
F 2 "" H 10000 2350 50  0001 C CNN
F 3 "" H 10000 2350 50  0001 C CNN
	1    10000 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 4300 8200 4300
Wire Wire Line
	8200 4300 8200 3800
$Comp
L power:+3.3V #PWR015
U 1 1 5FCC6B20
P 8200 3800
F 0 "#PWR015" H 8200 3650 50  0001 C CNN
F 1 "+3.3V" H 8215 3973 50  0000 C CNN
F 2 "" H 8200 3800 50  0001 C CNN
F 3 "" H 8200 3800 50  0001 C CNN
	1    8200 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 4500 8200 4500
Wire Wire Line
	8200 4500 8200 5150
$Comp
L power:GND #PWR016
U 1 1 5FCCC445
P 8200 5150
F 0 "#PWR016" H 8200 4900 50  0001 C CNN
F 1 "GND" H 8205 4977 50  0000 C CNN
F 2 "" H 8200 5150 50  0001 C CNN
F 3 "" H 8200 5150 50  0001 C CNN
	1    8200 5150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5FCCC489
P 10000 4950
F 0 "#PWR020" H 10000 4700 50  0001 C CNN
F 1 "GND" H 10005 4777 50  0000 C CNN
F 2 "" H 10000 4950 50  0001 C CNN
F 3 "" H 10000 4950 50  0001 C CNN
	1    10000 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 4950 10000 4900
Wire Wire Line
	10000 4900 9950 4900
$Comp
L Device:C_Small C3
U 1 1 5FCD1E45
P 8200 2600
F 0 "C3" H 8292 2646 50  0000 L CNN
F 1 "1u" H 8292 2555 50  0000 L CNN
F 2 "" H 8200 2600 50  0001 C CNN
F 3 "~" H 8200 2600 50  0001 C CNN
	1    8200 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5FCD1F05
P 9850 2600
F 0 "C4" H 9942 2646 50  0000 L CNN
F 1 "1u" H 9942 2555 50  0000 L CNN
F 2 "" H 9850 2600 50  0001 C CNN
F 3 "~" H 9850 2600 50  0001 C CNN
	1    9850 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5FCD1F79
P 8200 2800
F 0 "#PWR014" H 8200 2550 50  0001 C CNN
F 1 "GND" H 8205 2627 50  0000 C CNN
F 2 "" H 8200 2800 50  0001 C CNN
F 3 "" H 8200 2800 50  0001 C CNN
	1    8200 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5FCD1FAE
P 9850 2800
F 0 "#PWR018" H 9850 2550 50  0001 C CNN
F 1 "GND" H 9855 2627 50  0000 C CNN
F 2 "" H 9850 2800 50  0001 C CNN
F 3 "" H 9850 2800 50  0001 C CNN
	1    9850 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 2500 9850 2350
Connection ~ 9850 2350
Wire Wire Line
	9850 2350 10000 2350
Wire Wire Line
	9850 2800 9850 2700
Wire Wire Line
	8200 2800 8200 2700
Wire Wire Line
	8200 2500 8200 2350
Connection ~ 8200 2350
Wire Wire Line
	1550 4950 2150 4950
Wire Wire Line
	1550 5050 2150 5050
$Comp
L power:GND #PWR04
U 1 1 5FCF502C
P 1650 5250
F 0 "#PWR04" H 1650 5000 50  0001 C CNN
F 1 "GND" H 1655 5077 50  0000 C CNN
F 2 "" H 1650 5250 50  0001 C CNN
F 3 "" H 1650 5250 50  0001 C CNN
	1    1650 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 5250 1650 5150
Wire Wire Line
	1650 5150 1550 5150
$Comp
L power:GND #PWR06
U 1 1 5FCFB4BF
P 1750 6200
F 0 "#PWR06" H 1750 5950 50  0001 C CNN
F 1 "GND" H 1755 6027 50  0000 C CNN
F 2 "" H 1750 6200 50  0001 C CNN
F 3 "" H 1750 6200 50  0001 C CNN
	1    1750 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 6100 1750 6100
Wire Wire Line
	1750 6100 1750 6200
$Comp
L power:+3.3V #PWR05
U 1 1 5FD01D60
P 1750 5700
F 0 "#PWR05" H 1750 5550 50  0001 C CNN
F 1 "+3.3V" H 1765 5873 50  0000 C CNN
F 2 "" H 1750 5700 50  0001 C CNN
F 3 "" H 1750 5700 50  0001 C CNN
	1    1750 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 5800 1750 5800
Wire Wire Line
	1750 5800 1750 5700
$Comp
L power:GND #PWR07
U 1 1 5FD08877
P 1800 7150
F 0 "#PWR07" H 1800 6900 50  0001 C CNN
F 1 "GND" H 1805 6977 50  0000 C CNN
F 2 "" H 1800 7150 50  0001 C CNN
F 3 "" H 1800 7150 50  0001 C CNN
	1    1800 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 6750 1950 6750
Wire Wire Line
	1700 7050 1800 7050
Wire Wire Line
	1800 7050 1800 7150
Wire Wire Line
	1650 6000 2200 6000
Wire Wire Line
	1650 5900 2200 5900
Text Label 1850 5900 0    50   ~ 0
SWDIO
Text Label 1850 6000 0    50   ~ 0
SWDCLK
$Comp
L Device:R_Small R1
U 1 1 5FD437C7
P 2150 6850
F 0 "R1" V 2050 7000 50  0000 C CNN
F 1 "22" V 2045 6850 50  0000 C CNN
F 2 "" H 2150 6850 50  0001 C CNN
F 3 "~" H 2150 6850 50  0001 C CNN
	1    2150 6850
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5FD43929
P 2150 6950
F 0 "R2" V 2250 7100 50  0000 C CNN
F 1 "22" V 2250 6950 50  0000 C CNN
F 2 "" H 2150 6950 50  0001 C CNN
F 3 "~" H 2150 6950 50  0001 C CNN
	1    2150 6950
	0    1    1    0   
$EndComp
Wire Wire Line
	2250 6850 2800 6850
Wire Wire Line
	2250 6950 2400 6950
$Comp
L Device:R_Small R3
U 1 1 5FD6272E
P 2400 6600
F 0 "R3" H 2250 6500 50  0000 C CNN
F 1 "4.7k" H 2250 6600 50  0000 C CNN
F 2 "" H 2400 6600 50  0001 C CNN
F 3 "~" H 2400 6600 50  0001 C CNN
	1    2400 6600
	-1   0    0    1   
$EndComp
Wire Wire Line
	1950 6450 2400 6450
Wire Wire Line
	2400 6450 2400 6500
Wire Wire Line
	1950 6450 1950 6750
Wire Wire Line
	2400 6700 2400 6950
Connection ~ 2400 6950
Wire Wire Line
	2400 6950 2800 6950
Text Label 2500 6850 0    50   ~ 0
USB-
Text Label 2500 6950 0    50   ~ 0
USB+
Wire Wire Line
	1700 6850 2050 6850
Wire Wire Line
	1700 6950 2050 6950
$Comp
L power:+5V #PWR08
U 1 1 5FD93F14
P 2400 6400
F 0 "#PWR08" H 2400 6250 50  0001 C CNN
F 1 "+5V" H 2415 6573 50  0000 C CNN
F 2 "" H 2400 6400 50  0001 C CNN
F 3 "" H 2400 6400 50  0001 C CNN
	1    2400 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 6450 2400 6400
Connection ~ 2400 6450
Text Label 1800 4950 0    50   ~ 0
SERIAL_TX
Text Label 1800 5050 0    50   ~ 0
SERIAL_RX
$Comp
L power:+3.3V #PWR011
U 1 1 5FD9C6B0
P 4350 5700
F 0 "#PWR011" H 4350 5550 50  0001 C CNN
F 1 "+3.3V" H 4365 5873 50  0000 C CNN
F 2 "" H 4350 5700 50  0001 C CNN
F 3 "" H 4350 5700 50  0001 C CNN
	1    4350 5700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5FD9C6EB
P 4400 5900
F 0 "R5" V 4300 5900 50  0000 C CNN
F 1 "470" V 4500 5900 50  0000 C CNN
F 2 "" H 4400 5900 50  0001 C CNN
F 3 "~" H 4400 5900 50  0001 C CNN
	1    4400 5900
	0    1    1    0   
$EndComp
Wire Wire Line
	4050 5800 4350 5800
Wire Wire Line
	4350 5800 4350 5700
Wire Wire Line
	4050 5900 4300 5900
Wire Wire Line
	4500 5900 5100 5900
Text Label 4650 5900 0    50   ~ 0
ACTIVITY_LED
$Comp
L Device:C_Small C5
U 1 1 5FDBF454
P 3350 2100
F 0 "C5" H 3442 2146 50  0000 L CNN
F 1 "1u" H 3400 2000 50  0000 L CNN
F 2 "" H 3350 2100 50  0001 C CNN
F 3 "~" H 3350 2100 50  0001 C CNN
	1    3350 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5FDBF4BE
P 3100 2500
F 0 "#PWR021" H 3100 2250 50  0001 C CNN
F 1 "GND" H 3105 2327 50  0000 C CNN
F 2 "" H 3100 2500 50  0001 C CNN
F 3 "" H 3100 2500 50  0001 C CNN
	1    3100 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R6
U 1 1 5FDBF4FD
P 3100 1600
F 0 "R6" H 3159 1646 50  0000 L CNN
F 1 "10k" H 3159 1555 50  0000 L CNN
F 2 "" H 3100 1600 50  0001 C CNN
F 3 "~" H 3100 1600 50  0001 C CNN
	1    3100 1600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR013
U 1 1 5FDBF59F
P 3100 1450
F 0 "#PWR013" H 3100 1300 50  0001 C CNN
F 1 "+3.3V" H 3115 1623 50  0000 C CNN
F 2 "" H 3100 1450 50  0001 C CNN
F 3 "" H 3100 1450 50  0001 C CNN
	1    3100 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 2500 3100 2400
Wire Wire Line
	3100 2400 3350 2400
Wire Wire Line
	3350 2400 3350 2200
Connection ~ 3100 2400
Wire Wire Line
	3350 2000 3350 1800
Wire Wire Line
	3350 1800 3100 1800
Wire Wire Line
	3100 1850 3100 1800
Connection ~ 3100 1800
Wire Wire Line
	3100 1800 3100 1700
Wire Wire Line
	3100 1500 3100 1450
Connection ~ 3350 1800
Text Label 3300 1800 0    50   ~ 0
RESET
Text Label 4450 1800 0    50   ~ 0
RESET
$Comp
L power:GND #PWR023
U 1 1 5FE1D6E9
P 3750 3250
F 0 "#PWR023" H 3750 3000 50  0001 C CNN
F 1 "GND" H 3755 3077 50  0000 C CNN
F 2 "" H 3750 3250 50  0001 C CNN
F 3 "" H 3750 3250 50  0001 C CNN
	1    3750 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R7
U 1 1 5FE1D72A
P 3750 2250
F 0 "R7" H 3809 2296 50  0000 L CNN
F 1 "47k" H 3809 2205 50  0000 L CNN
F 2 "" H 3750 2250 50  0001 C CNN
F 3 "~" H 3750 2250 50  0001 C CNN
	1    3750 2250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR022
U 1 1 5FE1D88E
P 3750 2100
F 0 "#PWR022" H 3750 1950 50  0001 C CNN
F 1 "+3.3V" H 3765 2273 50  0000 C CNN
F 2 "" H 3750 2100 50  0001 C CNN
F 3 "" H 3750 2100 50  0001 C CNN
	1    3750 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 1800 5050 1800
Wire Wire Line
	3750 2400 4000 2400
Wire Wire Line
	4000 2400 4000 2000
Wire Wire Line
	4000 2000 5050 2000
Wire Wire Line
	3750 2400 3750 2350
Connection ~ 3750 2400
Wire Wire Line
	3750 2150 3750 2100
Wire Wire Line
	3750 3250 3750 3100
$Comp
L Device:R_Small R8
U 1 1 5FE76C77
P 4200 3100
F 0 "R8" V 4300 3200 50  0000 L CNN
F 1 "47k" V 4300 3000 50  0000 L CNN
F 2 "" H 4200 3100 50  0001 C CNN
F 3 "~" H 4200 3100 50  0001 C CNN
	1    4200 3100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5050 3100 4300 3100
Wire Wire Line
	4100 3100 3750 3100
Connection ~ 3750 3100
Wire Wire Line
	3750 3100 3750 3000
Text Label 4550 3700 0    50   ~ 0
SCSI_DB0
Text Label 4550 3800 0    50   ~ 0
SCSI_DB1
Text Label 4550 3900 0    50   ~ 0
SCSI_DB2
Text Label 4550 4000 0    50   ~ 0
SCSI_DB3
Text Label 4550 4100 0    50   ~ 0
SCSI_DB4
Text Label 4550 4200 0    50   ~ 0
SCSI_DB5
Text Label 4550 4300 0    50   ~ 0
SCSI_DB6
Text Label 4550 4400 0    50   ~ 0
SCSI_DB7
Text Label 6450 3300 0    50   ~ 0
SD_CS
Text Label 6450 3600 0    50   ~ 0
SD_MOSI
Text Label 6450 3500 0    50   ~ 0
SD_MISO
Text Label 6450 3400 0    50   ~ 0
SD_CLK
Text Label 6450 3700 0    50   ~ 0
SCSI_DBP
Text Label 6450 3800 0    50   ~ 0
SERIAL_TX
Text Label 6450 3900 0    50   ~ 0
SERIAL_RX
Text Label 6500 4000 0    50   ~ 0
USB-
Text Label 6500 4100 0    50   ~ 0
USB+
Text Label 6500 4200 0    50   ~ 0
SWDIO
Text Label 6500 4300 0    50   ~ 0
SWDCLK
Text Label 6400 4400 0    50   ~ 0
SCSI_ATN
Text Label 4550 3200 0    50   ~ 0
SCSI_BSY
Text Label 4550 3300 0    50   ~ 0
SCSI_ACK
Text Label 4550 3400 0    50   ~ 0
SCSI_RST
Text Label 4550 3500 0    50   ~ 0
SCSI_MSG
Text Label 4550 3600 0    50   ~ 0
SCSI_SEL
Text Label 6450 2900 0    50   ~ 0
SCSI_CD
Text Label 6450 3000 0    50   ~ 0
SCSI_REQ
Text Label 6450 3100 0    50   ~ 0
SCSI_IO
Text Label 6450 3200 0    50   ~ 0
UNIT_SEL2
NoConn ~ 5050 3000
NoConn ~ 5050 2900
Wire Wire Line
	8250 4100 7800 4100
Wire Wire Line
	7800 4100 7800 3300
Wire Wire Line
	6350 3300 7800 3300
Wire Wire Line
	8250 4200 7700 4200
Wire Wire Line
	7700 4200 7700 3600
Wire Wire Line
	6350 3600 7700 3600
Wire Wire Line
	8250 4400 7600 4400
Wire Wire Line
	7600 4400 7600 3400
Wire Wire Line
	6350 3400 7600 3400
Wire Wire Line
	8250 4600 7500 4600
Wire Wire Line
	7500 4600 7500 3500
Wire Wire Line
	6350 3500 7500 3500
Wire Wire Line
	1050 2200 800  2200
Connection ~ 800  2200
Wire Wire Line
	800  2200 800  2300
Wire Wire Line
	7800 1950 7800 1100
Wire Wire Line
	7800 1950 8200 1950
Wire Wire Line
	4150 1100 7800 1100
$Comp
L Device:C_Small C6
U 1 1 5FF51D06
P 6450 1650
F 0 "C6" H 6542 1696 50  0000 L CNN
F 1 "0.1u" H 6450 1500 50  0000 L CNN
F 2 "" H 6450 1650 50  0001 C CNN
F 3 "~" H 6450 1650 50  0001 C CNN
	1    6450 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5FF51DE6
P 6700 1650
F 0 "C7" H 6792 1696 50  0000 L CNN
F 1 "0.1u" H 6700 1500 50  0000 L CNN
F 2 "" H 6700 1650 50  0001 C CNN
F 3 "~" H 6700 1650 50  0001 C CNN
	1    6700 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5FF51E46
P 6950 1650
F 0 "C8" H 7042 1696 50  0000 L CNN
F 1 "0.1u" H 6950 1500 50  0000 L CNN
F 2 "" H 6950 1650 50  0001 C CNN
F 3 "~" H 6950 1650 50  0001 C CNN
	1    6950 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 5FF51EA0
P 7200 1650
F 0 "C9" H 7292 1696 50  0000 L CNN
F 1 "0.1u" H 7200 1500 50  0000 L CNN
F 2 "" H 7200 1650 50  0001 C CNN
F 3 "~" H 7200 1650 50  0001 C CNN
	1    7200 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 5FF6A4A0
P 7450 1650
F 0 "C10" H 7542 1696 50  0000 L CNN
F 1 "0.1u" H 7450 1500 50  0000 L CNN
F 2 "" H 7450 1650 50  0001 C CNN
F 3 "~" H 7450 1650 50  0001 C CNN
	1    7450 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 1450 9150 1450
NoConn ~ 9150 1150
Wire Wire Line
	8850 1250 9150 1250
Wire Wire Line
	8850 1350 8850 1250
Wire Wire Line
	8850 1350 9150 1350
Connection ~ 8850 1350
Wire Wire Line
	8850 1550 8850 1350
$Comp
L power:GND #PWR024
U 1 1 5FEF00DD
P 8850 1550
F 0 "#PWR024" H 8850 1300 50  0001 C CNN
F 1 "GND" H 8855 1377 50  0000 C CNN
F 2 "" H 8850 1550 50  0001 C CNN
F 3 "" H 8850 1550 50  0001 C CNN
	1    8850 1550
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J6
U 1 1 5FC2C72B
P 9350 1250
F 0 "J6" H 9456 1528 50  0000 C CNN
F 1 "AMP FLOPPY PWR" H 9456 1437 50  0000 C CNN
F 2 "" H 9350 1250 50  0001 C CNN
F 3 "~" H 9350 1250 50  0001 C CNN
	1    9350 1250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8200 2350 8500 2350
Wire Wire Line
	8200 1950 8200 2350
Wire Wire Line
	6450 1750 6700 1750
Connection ~ 6700 1750
Wire Wire Line
	6700 1750 6950 1750
Connection ~ 6950 1750
Wire Wire Line
	6950 1750 7200 1750
Connection ~ 7200 1750
Wire Wire Line
	7200 1750 7450 1750
Wire Wire Line
	7450 1550 7200 1550
Connection ~ 6700 1550
Wire Wire Line
	6700 1550 6450 1550
Connection ~ 6950 1550
Wire Wire Line
	6950 1550 6700 1550
Connection ~ 7200 1550
Wire Wire Line
	7200 1550 6950 1550
Wire Wire Line
	5950 1500 6950 1500
Wire Wire Line
	6950 1500 6950 1550
Connection ~ 5950 1500
$Comp
L power:GND #PWR0101
U 1 1 5FFCD1BA
P 6950 1850
F 0 "#PWR0101" H 6950 1600 50  0001 C CNN
F 1 "GND" H 6955 1677 50  0000 C CNN
F 2 "" H 6950 1850 50  0001 C CNN
F 3 "" H 6950 1850 50  0001 C CNN
	1    6950 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 1750 6950 1850
Wire Wire Line
	8200 1950 8950 1950
Wire Wire Line
	8950 1450 8950 1950
Connection ~ 8200 1950
NoConn ~ 8250 4700
NoConn ~ 8250 4800
NoConn ~ 8250 4900
NoConn ~ 8250 4000
Wire Wire Line
	6150 5450 6150 5750
Connection ~ 6150 5750
Wire Wire Line
	6150 5750 6150 6050
Connection ~ 6150 6050
Wire Wire Line
	6150 6050 6150 6250
$Comp
L power:GND #PWR0102
U 1 1 60061896
P 6150 6250
F 0 "#PWR0102" H 6150 6000 50  0001 C CNN
F 1 "GND" H 6155 6077 50  0000 C CNN
F 2 "" H 6150 6250 50  0001 C CNN
F 3 "" H 6150 6250 50  0001 C CNN
	1    6150 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R9
U 1 1 600618E5
P 6900 5200
F 0 "R9" V 6800 5200 50  0000 C CNN
F 1 "10k" V 7000 5200 50  0000 C CNN
F 2 "" H 6900 5200 50  0001 C CNN
F 3 "~" H 6900 5200 50  0001 C CNN
	1    6900 5200
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R10
U 1 1 600619C0
P 7250 5200
F 0 "R10" V 7150 5200 50  0000 C CNN
F 1 "10k" V 7350 5200 50  0000 C CNN
F 2 "" H 7250 5200 50  0001 C CNN
F 3 "~" H 7250 5200 50  0001 C CNN
	1    7250 5200
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R11
U 1 1 60061A26
P 7550 5200
F 0 "R11" V 7450 5200 50  0000 C CNN
F 1 "10k" V 7650 5200 50  0000 C CNN
F 2 "" H 7550 5200 50  0001 C CNN
F 3 "~" H 7550 5200 50  0001 C CNN
	1    7550 5200
	-1   0    0    1   
$EndComp
Wire Wire Line
	6750 5450 6900 5450
Wire Wire Line
	6750 5750 7250 5750
Wire Wire Line
	6750 6050 7550 6050
Wire Wire Line
	6900 5300 6900 5450
Connection ~ 6900 5450
Wire Wire Line
	6900 5450 8150 5450
Wire Wire Line
	7250 5300 7250 5750
Connection ~ 7250 5750
Wire Wire Line
	7250 5750 8150 5750
Wire Wire Line
	7550 5300 7550 6050
Connection ~ 7550 6050
Wire Wire Line
	7550 6050 8150 6050
$Comp
L power:+3.3V #PWR0103
U 1 1 600D4204
P 6900 5050
F 0 "#PWR0103" H 6900 4900 50  0001 C CNN
F 1 "+3.3V" H 6915 5223 50  0000 C CNN
F 2 "" H 6900 5050 50  0001 C CNN
F 3 "" H 6900 5050 50  0001 C CNN
	1    6900 5050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0104
U 1 1 600D4259
P 7250 5050
F 0 "#PWR0104" H 7250 4900 50  0001 C CNN
F 1 "+3.3V" H 7265 5223 50  0000 C CNN
F 2 "" H 7250 5050 50  0001 C CNN
F 3 "" H 7250 5050 50  0001 C CNN
	1    7250 5050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0105
U 1 1 600D42AE
P 7550 5050
F 0 "#PWR0105" H 7550 4900 50  0001 C CNN
F 1 "+3.3V" H 7565 5223 50  0000 C CNN
F 2 "" H 7550 5050 50  0001 C CNN
F 3 "" H 7550 5050 50  0001 C CNN
	1    7550 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 5100 7550 5050
Wire Wire Line
	7250 5100 7250 5050
Wire Wire Line
	6900 5100 6900 5050
Text Label 7750 6050 0    50   ~ 0
UNIT_SEL2
Text Label 7750 5750 0    50   ~ 0
UNIT_SEL1
Text Label 7750 5450 0    50   ~ 0
UNIT_SEL0
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 601012A7
P 8500 2350
F 0 "#FLG0102" H 8500 2425 50  0001 C CNN
F 1 "PWR_FLAG" H 8500 2524 50  0000 C CNN
F 2 "" H 8500 2350 50  0001 C CNN
F 3 "~" H 8500 2350 50  0001 C CNN
	1    8500 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 2350 9850 2350
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 60101632
P 8650 1250
F 0 "#FLG0101" H 8650 1325 50  0001 C CNN
F 1 "PWR_FLAG" H 8650 1424 50  0000 C CNN
F 2 "" H 8650 1250 50  0001 C CNN
F 3 "~" H 8650 1250 50  0001 C CNN
	1    8650 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 1250 8650 1350
Wire Wire Line
	8650 1350 8850 1350
$Comp
L power:+5V #PWR0106
U 1 1 60110AEA
P 7800 1100
F 0 "#PWR0106" H 7800 950 50  0001 C CNN
F 1 "+5V" H 7815 1273 50  0000 C CNN
F 2 "" H 7800 1100 50  0001 C CNN
F 3 "" H 7800 1100 50  0001 C CNN
	1    7800 1100
	1    0    0    -1  
$EndComp
Connection ~ 7800 1100
$EndSCHEMATC
