EESchema Schematic File Version 2
LIBS:ArduProMiniTKB
LIBS:reg_204-5
LIBS:arduinopromini-cache
EELAYER 25 0
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
L ARDUPROMINI-6 uP1
U 1 1 55E8DADD
P 8400 3800
F 0 "uP1" H 8400 4650 60  0000 C CNN
F 1 "ARDUPROMINI-6" H 8400 4550 60  0000 C CNN
F 2 "Bibliotecas:ArduProMinicdr2" H 8450 3800 60  0001 C CNN
F 3 "" H 8450 3800 60  0000 C CNN
	1    8400 3800
	1    0    0    -1  
$EndComp
$Comp
L ATMEGA328-A IC1
U 1 1 55E8DB83
P 5300 4100
F 0 "IC1" H 4550 5350 40  0000 L BNN
F 1 "ATMEGA328-A" H 5700 2700 40  0000 L BNN
F 2 "Housings_QFP:TQFP-32_7x7mm_Pitch0.8mm" H 5300 4100 30  0000 C CIN
F 3 "" H 5300 4100 60  0000 C CNN
	1    5300 4100
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR01
U 1 1 55EA04DC
P 4250 2900
F 0 "#PWR01" H 4250 2750 50  0001 C CNN
F 1 "VCC" H 4250 3050 50  0000 C CNN
F 2 "" H 4250 2900 60  0000 C CNN
F 3 "" H 4250 2900 60  0000 C CNN
	1    4250 2900
	1    0    0    -1  
$EndComp
Text Label 6550 4450 2    60   ~ 0
reset
$Comp
L VCC #PWR02
U 1 1 55EA0539
P 1850 3250
F 0 "#PWR02" H 1850 3100 50  0001 C CNN
F 1 "VCC" H 1850 3400 50  0000 C CNN
F 2 "" H 1850 3250 60  0000 C CNN
F 3 "" H 1850 3250 60  0000 C CNN
	1    1850 3250
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 55EA0623
P 1850 4450
F 0 "SW1" H 2000 4560 50  0000 C CNN
F 1 "SW_PUSH" H 1850 4370 50  0000 C CNN
F 2 "Capacitors_SMD:C_0805" H 1850 4450 60  0001 C CNN
F 3 "" H 1850 4450 60  0000 C CNN
	1    1850 4450
	0    1    1    0   
$EndComp
$Comp
L GND #PWR03
U 1 1 55EA0675
P 1850 4800
F 0 "#PWR03" H 1850 4550 50  0001 C CNN
F 1 "GND" H 1850 4650 50  0000 C CNN
F 2 "" H 1850 4800 60  0000 C CNN
F 3 "" H 1850 4800 60  0000 C CNN
	1    1850 4800
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 55EA06C3
P 1850 3550
F 0 "R1" V 1930 3550 50  0000 C CNN
F 1 "10 k" V 1850 3550 50  0000 C CNN
F 2 "LEDs:LED-0805" V 1780 3550 30  0001 C CNN
F 3 "" H 1850 3550 30  0000 C CNN
	1    1850 3550
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 55EA0776
P 3900 3900
F 0 "C8" H 3925 4000 50  0000 L CNN
F 1 "0.1 uF" H 3925 3800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3938 3750 30  0001 C CNN
F 3 "" H 3900 3900 60  0000 C CNN
	1    3900 3900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 55EA07B9
P 3900 4100
F 0 "#PWR04" H 3900 3850 50  0001 C CNN
F 1 "GND" H 3900 3950 50  0000 C CNN
F 2 "" H 3900 4100 60  0000 C CNN
F 3 "" H 3900 4100 60  0000 C CNN
	1    3900 4100
	1    0    0    -1  
$EndComp
Text Label 6550 3600 2    60   ~ 0
xtal1
Text Label 6550 3700 2    60   ~ 0
xtal2
$Comp
L Crystal Y1
U 1 1 55EA0965
P 1800 2200
F 0 "Y1" H 1800 2350 50  0000 C CNN
F 1 "16 MHz" H 1800 2050 50  0000 C CNN
F 2 "Crystals:Crystal_HC49-U_Vertical" H 1800 2200 60  0001 C CNN
F 3 "" H 1800 2200 60  0000 C CNN
	1    1800 2200
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 55EA09AF
P 1600 2450
F 0 "C3" H 1625 2550 50  0000 L CNN
F 1 "15 pF" H 1625 2350 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1638 2300 30  0001 C CNN
F 3 "" H 1600 2450 60  0000 C CNN
	1    1600 2450
	0    1    1    0   
$EndComp
$Comp
L C C2
U 1 1 55EA0A08
P 1600 1950
F 0 "C2" H 1625 2050 50  0000 L CNN
F 1 "15 pF" H 1625 1850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1638 1800 30  0001 C CNN
F 3 "" H 1600 1950 60  0000 C CNN
	1    1600 1950
	0    1    1    0   
$EndComp
$Comp
L GND #PWR05
U 1 1 55EA0AA7
P 1100 2300
F 0 "#PWR05" H 1100 2050 50  0001 C CNN
F 1 "GND" H 1100 2150 50  0000 C CNN
F 2 "" H 1100 2300 60  0000 C CNN
F 3 "" H 1100 2300 60  0000 C CNN
	1    1100 2300
	1    0    0    -1  
$EndComp
Text Label 2250 1950 2    60   ~ 0
xtal1
Text Label 2250 2450 2    60   ~ 0
xtal2
$Comp
L VCC #PWR06
U 1 1 55EA0C96
P 3250 1800
F 0 "#PWR06" H 3250 1650 50  0001 C CNN
F 1 "VCC" H 3250 1950 50  0000 C CNN
F 2 "" H 3250 1800 60  0000 C CNN
F 3 "" H 3250 1800 60  0000 C CNN
	1    3250 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 55EA0CC5
P 3250 2200
F 0 "#PWR07" H 3250 1950 50  0001 C CNN
F 1 "GND" H 3250 2050 50  0000 C CNN
F 2 "" H 3250 2200 60  0000 C CNN
F 3 "" H 3250 2200 60  0000 C CNN
	1    3250 2200
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 55EA0CF4
P 3250 2000
F 0 "C6" H 3275 2100 50  0000 L CNN
F 1 "0.1 uF" H 3275 1900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3288 1850 30  0001 C CNN
F 3 "" H 3250 2000 60  0000 C CNN
	1    3250 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 55EA0F6C
P 4400 5450
F 0 "#PWR08" H 4400 5200 50  0001 C CNN
F 1 "GND" H 4400 5300 50  0000 C CNN
F 2 "" H 4400 5450 60  0000 C CNN
F 3 "" H 4400 5450 60  0000 C CNN
	1    4400 5450
	1    0    0    -1  
$EndComp
Text Label 6500 3500 2    60   ~ 0
led
$Comp
L R R2
U 1 1 55EA114A
P 2000 6050
F 0 "R2" V 2080 6050 50  0000 C CNN
F 1 "330" V 2000 6050 50  0000 C CNN
F 2 "LEDs:LED-0805" V 1930 6050 30  0001 C CNN
F 3 "" H 2000 6050 30  0000 C CNN
	1    2000 6050
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 55EA1180
P 2000 6450
F 0 "D1" H 2000 6550 50  0000 C CNN
F 1 "Verde" H 2000 6350 50  0000 C CNN
F 2 "LEDs:LED-0805" H 2000 6450 60  0001 C CNN
F 3 "" H 2000 6450 60  0000 C CNN
	1    2000 6450
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR09
U 1 1 55EA11CD
P 2000 6700
F 0 "#PWR09" H 2000 6450 50  0001 C CNN
F 1 "GND" H 2000 6550 50  0000 C CNN
F 2 "" H 2000 6700 60  0000 C CNN
F 3 "" H 2000 6700 60  0000 C CNN
	1    2000 6700
	1    0    0    -1  
$EndComp
Text Label 2000 5800 2    60   ~ 0
led
Text GLabel 2100 5850 2    60   Input ~ 0
sck
$Comp
L REG_104-5 U1
U 1 1 55EA1805
P 3950 6500
F 0 "U1" H 3700 6700 60  0000 C CNN
F 1 "REG_104-5" H 3950 6800 60  0000 C CNN
F 2 "SOT223-5:SOT223-5" H 3950 6500 60  0001 C CNN
F 3 "" H 3950 6500 60  0000 C CNN
	1    3950 6500
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 55EA1BD0
P 2950 6950
F 0 "C4" H 2975 7050 50  0000 L CNN
F 1 "0.1 uF" H 2975 6850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 2988 6800 30  0001 C CNN
F 3 "" H 2950 6950 60  0000 C CNN
	1    2950 6950
	-1   0    0    1   
$EndComp
Text GLabel 3550 6000 2    60   Input ~ 0
raw
Wire Wire Line
	4400 3000 4400 3300
Connection ~ 4400 3100
Wire Wire Line
	4400 3000 4250 3000
Wire Wire Line
	4250 3000 4250 2900
Connection ~ 4400 3000
Wire Wire Line
	6300 4450 6650 4450
Wire Wire Line
	1850 3700 1850 4150
Wire Wire Line
	1850 4750 1850 4800
Wire Wire Line
	1850 3400 1850 3250
Wire Wire Line
	4400 3600 3900 3600
Wire Wire Line
	3900 3600 3900 3750
Wire Wire Line
	3900 4100 3900 4050
Wire Wire Line
	6550 3600 6300 3600
Wire Wire Line
	6300 3700 6550 3700
Wire Wire Line
	1800 2050 1800 1950
Wire Wire Line
	1750 1950 2250 1950
Wire Wire Line
	1750 2450 2250 2450
Wire Wire Line
	1800 2450 1800 2350
Wire Wire Line
	1450 2450 1350 2450
Wire Wire Line
	1350 2450 1350 1950
Wire Wire Line
	1350 1950 1450 1950
Wire Wire Line
	1350 2200 1100 2200
Wire Wire Line
	1100 2200 1100 2300
Connection ~ 1350 2200
Connection ~ 1800 1950
Connection ~ 1800 2450
Wire Wire Line
	3250 1850 3250 1800
Wire Wire Line
	3250 2150 3250 2200
Wire Wire Line
	4400 5100 4400 5450
Connection ~ 4400 5200
Connection ~ 4400 5300
Wire Wire Line
	6300 3500 6500 3500
Wire Wire Line
	2000 6700 2000 6650
Wire Wire Line
	2000 6250 2000 6200
Wire Wire Line
	2000 5900 2000 5800
Wire Wire Line
	2100 5850 2000 5850
Connection ~ 2000 5850
Text GLabel 4550 6600 2    60   Input ~ 0
raw
Wire Wire Line
	4550 6600 4450 6600
$Comp
L GND #PWR010
U 1 1 55EA1DAA
P 3050 6400
F 0 "#PWR010" H 3050 6150 50  0001 C CNN
F 1 "GND" H 3050 6250 50  0000 C CNN
F 2 "" H 3050 6400 60  0000 C CNN
F 3 "" H 3050 6400 60  0000 C CNN
	1    3050 6400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 55EA1E78
P 3950 7000
F 0 "#PWR011" H 3950 6750 50  0001 C CNN
F 1 "GND" H 3950 6850 50  0000 C CNN
F 2 "" H 3950 7000 60  0000 C CNN
F 3 "" H 3950 7000 60  0000 C CNN
	1    3950 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 6950 3950 7000
Wire Wire Line
	3050 6400 3050 6300
Wire Wire Line
	3050 6000 3550 6000
Wire Wire Line
	3450 6000 3450 6400
Connection ~ 3450 6000
Wire Wire Line
	4450 6400 5050 6400
$Comp
L CP C7
U 1 1 55EA232F
P 3250 6950
F 0 "C7" H 3275 7050 50  0000 L CNN
F 1 "10 uF" H 3275 6850 50  0000 L CNN
F 2 "SMD_Packages:SMD-1206_Pol" H 3288 6800 30  0001 C CNN
F 3 "" H 3250 6950 60  0000 C CNN
	1    3250 6950
	1    0    0    -1  
$EndComp
$Comp
L CP C5
U 1 1 55EA25C4
P 3050 6150
F 0 "C5" H 3075 6250 50  0000 L CNN
F 1 "10 uF" H 3075 6050 50  0000 L CNN
F 2 "SMD_Packages:SMD-1206_Pol" H 3088 6000 30  0001 C CNN
F 3 "" H 3050 6150 60  0000 C CNN
	1    3050 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 6800 3450 6600
Wire Wire Line
	2550 6800 3450 6800
Connection ~ 3250 6800
Wire Wire Line
	2950 7100 3250 7100
Wire Wire Line
	3100 7100 3100 7250
Connection ~ 3100 7100
$Comp
L GND #PWR012
U 1 1 55EA2778
P 3100 7250
F 0 "#PWR012" H 3100 7000 50  0001 C CNN
F 1 "GND" H 3100 7100 50  0000 C CNN
F 2 "" H 3100 7250 60  0000 C CNN
F 3 "" H 3100 7250 60  0000 C CNN
	1    3100 7250
	1    0    0    -1  
$EndComp
Connection ~ 2950 6800
$Comp
L VCC #PWR013
U 1 1 55EA2835
P 2550 6750
F 0 "#PWR013" H 2550 6600 50  0001 C CNN
F 1 "VCC" H 2550 6900 50  0000 C CNN
F 2 "" H 2550 6750 60  0000 C CNN
F 3 "" H 2550 6750 60  0000 C CNN
	1    2550 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 6800 2550 6750
Wire Wire Line
	6300 3950 6650 3950
Wire Wire Line
	6300 4050 6650 4050
Wire Wire Line
	6300 4150 6650 4150
Wire Wire Line
	6300 3850 6650 3850
Text GLabel 6650 3850 2    60   Input ~ 0
A0
Text GLabel 6650 3950 2    60   Input ~ 0
A1
Text GLabel 6650 4050 2    60   Input ~ 0
A2
Text GLabel 6650 4150 2    60   Input ~ 0
A3
Text GLabel 6650 4600 2    60   Input ~ 0
RXI
Text GLabel 6650 4700 2    60   Input ~ 0
TX0
Wire Wire Line
	6300 4600 6650 4600
Wire Wire Line
	6300 4700 6650 4700
Text GLabel 6650 4800 2    60   Input ~ 0
D2
Wire Wire Line
	6300 4800 6650 4800
Text GLabel 6650 4900 2    60   Input ~ 0
D3
Text GLabel 6650 5000 2    60   Input ~ 0
D4
Text GLabel 6650 5100 2    60   Input ~ 0
D5
Text GLabel 6650 5200 2    60   Input ~ 0
D6
Text GLabel 6650 5300 2    60   Input ~ 0
D7
Wire Wire Line
	6300 4900 6650 4900
Wire Wire Line
	6650 5000 6300 5000
Wire Wire Line
	6300 5100 6650 5100
Wire Wire Line
	6650 5200 6300 5200
Wire Wire Line
	6300 5300 6650 5300
Text GLabel 6650 3000 2    60   Input ~ 0
D8
Text GLabel 6650 3100 2    60   Input ~ 0
D9
Wire Wire Line
	6300 3000 6650 3000
Wire Wire Line
	6650 3100 6300 3100
Text GLabel 6650 3200 2    60   Input ~ 0
D10
Wire Wire Line
	6650 3200 6300 3200
Text GLabel 6650 3300 2    60   Input ~ 0
MOSI
Text GLabel 6650 3400 2    60   Input ~ 0
MISO
Wire Wire Line
	6650 3300 6300 3300
Wire Wire Line
	6300 3400 6650 3400
Text GLabel 6650 4250 2    60   Input ~ 0
A4
Text GLabel 6650 4350 2    60   Input ~ 0
A5
Wire Wire Line
	6650 4350 6300 4350
Wire Wire Line
	6300 4250 6650 4250
Text GLabel 9050 3900 2    60   Input ~ 0
A0
Text GLabel 9050 3800 2    60   Input ~ 0
A1
Text GLabel 9050 3700 2    60   Input ~ 0
A2
Text GLabel 9050 3600 2    60   Input ~ 0
A3
Text GLabel 7750 3600 0    60   Input ~ 0
D2
Text GLabel 7750 3700 0    60   Input ~ 0
D3
Text GLabel 7750 3800 0    60   Input ~ 0
D4
Text GLabel 7750 3900 0    60   Input ~ 0
D5
Text GLabel 7750 4000 0    60   Input ~ 0
D6
Text GLabel 7750 4100 0    60   Input ~ 0
D7
Text GLabel 7750 4200 0    60   Input ~ 0
D8
Text GLabel 7750 4300 0    60   Input ~ 0
D9
Wire Wire Line
	7800 4300 7750 4300
Wire Wire Line
	7750 4200 7800 4200
Wire Wire Line
	7800 4100 7750 4100
Wire Wire Line
	7800 4000 7750 4000
Wire Wire Line
	7750 3900 7800 3900
Wire Wire Line
	7800 3800 7750 3800
Wire Wire Line
	7750 3700 7800 3700
Wire Wire Line
	7800 3600 7750 3600
Text GLabel 8250 4850 3    60   Input ~ 0
A4
Text GLabel 8350 4850 3    60   Input ~ 0
A5
Wire Wire Line
	8550 4850 8550 4800
Wire Wire Line
	8450 4850 8450 4800
Wire Wire Line
	8350 4850 8350 4800
Wire Wire Line
	8250 4850 8250 4800
Text GLabel 7750 3200 0    60   Input ~ 0
TX0
Text GLabel 7750 3300 0    60   Input ~ 0
RXI
Wire Wire Line
	7750 3300 7800 3300
Wire Wire Line
	7800 3200 7750 3200
Text GLabel 9050 4300 2    60   Input ~ 0
D10
Wire Wire Line
	9050 4300 9000 4300
Wire Wire Line
	9050 3900 9000 3900
Wire Wire Line
	9000 3800 9050 3800
Wire Wire Line
	9050 3700 9000 3700
Wire Wire Line
	9000 3600 9050 3600
Wire Wire Line
	9000 3500 9350 3500
Text GLabel 9100 3200 2    60   Input ~ 0
raw
Wire Wire Line
	9100 3200 9000 3200
Text GLabel 9050 4200 2    60   Input ~ 0
MOSI
Text GLabel 9050 4100 2    60   Input ~ 0
MISO
Wire Wire Line
	9050 4200 9000 4200
Wire Wire Line
	9000 4100 9050 4100
Text GLabel 9050 4000 2    60   Input ~ 0
sck
Wire Wire Line
	9050 4000 9000 4000
Wire Wire Line
	9000 3300 9550 3300
Wire Wire Line
	7450 3500 7800 3500
Text GLabel 7750 3400 0    60   Input ~ 0
RST
Text GLabel 9050 3400 2    60   Input ~ 0
RST
Wire Wire Line
	9050 3400 9000 3400
Wire Wire Line
	7800 3400 7750 3400
$Comp
L GND #PWR014
U 1 1 55EDC218
P 7450 3550
F 0 "#PWR014" H 7450 3300 50  0001 C CNN
F 1 "GND" H 7450 3400 50  0000 C CNN
F 2 "" H 7450 3550 60  0000 C CNN
F 3 "" H 7450 3550 60  0000 C CNN
	1    7450 3550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 55EDC2CC
P 9550 3350
F 0 "#PWR015" H 9550 3100 50  0001 C CNN
F 1 "GND" H 9550 3200 50  0000 C CNN
F 2 "" H 9550 3350 60  0000 C CNN
F 3 "" H 9550 3350 60  0000 C CNN
	1    9550 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 3500 7450 3550
Text GLabel 6650 4450 2    60   Input ~ 0
RST
$Comp
L PWR_FLAG #FLG016
U 1 1 55EDCED5
P 9050 3150
F 0 "#FLG016" H 9050 3245 50  0001 C CNN
F 1 "PWR_FLAG" H 9050 3330 50  0000 C CNN
F 2 "" H 9050 3150 60  0000 C CNN
F 3 "" H 9050 3150 60  0000 C CNN
	1    9050 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 3150 9050 3200
Connection ~ 9050 3200
$Comp
L PWR_FLAG #FLG017
U 1 1 55EDD2A0
P 4150 5350
F 0 "#FLG017" H 4150 5445 50  0001 C CNN
F 1 "PWR_FLAG" H 4150 5530 50  0000 C CNN
F 2 "" H 4150 5350 60  0000 C CNN
F 3 "" H 4150 5350 60  0000 C CNN
	1    4150 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 5350 4400 5350
Connection ~ 4400 5350
$Comp
L PWR_FLAG #FLG018
U 1 1 55EDD8F3
P 2700 6650
F 0 "#FLG018" H 2700 6745 50  0001 C CNN
F 1 "PWR_FLAG" H 2700 6830 50  0000 C CNN
F 2 "" H 2700 6650 60  0000 C CNN
F 3 "" H 2700 6650 60  0000 C CNN
	1    2700 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 6650 2700 6800
Connection ~ 2700 6800
$Comp
L VCC #PWR019
U 1 1 55EDDAF5
P 9350 3500
F 0 "#PWR019" H 9350 3350 50  0001 C CNN
F 1 "VCC" H 9350 3650 50  0000 C CNN
F 2 "" H 9350 3500 60  0000 C CNN
F 3 "" H 9350 3500 60  0000 C CNN
	1    9350 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 3300 9550 3350
$Comp
L GND #PWR020
U 1 1 55EDE382
P 5050 6450
F 0 "#PWR020" H 5050 6200 50  0001 C CNN
F 1 "GND" H 5050 6300 50  0000 C CNN
F 2 "" H 5050 6450 60  0000 C CNN
F 3 "" H 5050 6450 60  0000 C CNN
	1    5050 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 6400 5050 6450
NoConn ~ 4400 4450
NoConn ~ 8550 4850
NoConn ~ 4400 4350
NoConn ~ 8450 4850
Connection ~ 1850 3900
Text Label 1600 3900 0    60   ~ 0
reset
Wire Wire Line
	1600 3900 1850 3900
$EndSCHEMATC
