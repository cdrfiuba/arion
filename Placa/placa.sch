EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:tcrt1000
LIBS:reg_204-5
LIBS:DVR8833
LIBS:placa-cache
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
L ATMEGA328-A IC?
U 1 1 56A2C86A
P 2750 5550
F 0 "IC?" H 2000 6800 50  0000 L BNN
F 1 "ATMEGA328-A" H 3150 4150 50  0000 L BNN
F 2 "TQFP32" H 2750 5550 50  0000 C CIN
F 3 "" H 2750 5550 50  0000 C CNN
	1    2750 5550
	1    0    0    -1  
$EndComp
$Comp
L TCRT1000 U?
U 1 1 56A2C9DC
P 4700 2450
F 0 "U?" H 4700 2450 60  0000 C CNN
F 1 "TCRT1000" H 4950 2650 60  0000 C CNN
F 2 "TCRT1000" H 4450 2650 60  0000 C CNN
F 3 "" H 4700 2450 60  0000 C CNN
	1    4700 2450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 56A2CA2B
P 4050 2700
F 0 "#PWR?" H 4050 2550 50  0001 C CNN
F 1 "+5V" H 4050 2840 50  0000 C CNN
F 2 "" H 4050 2700 50  0000 C CNN
F 3 "" H 4050 2700 50  0000 C CNN
	1    4050 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 2700 4050 2850
Wire Wire Line
	4050 2850 4250 2850
$Comp
L R R?
U 1 1 56A2CA53
P 4550 3050
F 0 "R?" V 4630 3050 50  0000 C CNN
F 1 "180" V 4550 3050 50  0000 C CNN
F 2 "" V 4480 3050 50  0000 C CNN
F 3 "" H 4550 3050 50  0000 C CNN
	1    4550 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 2850 4550 2900
Wire Wire Line
	4550 3200 4550 3250
$Comp
L GND #PWR?
U 1 1 56A2CA88
P 4550 3250
F 0 "#PWR?" H 4550 3000 50  0001 C CNN
F 1 "GND" H 4550 3100 50  0000 C CNN
F 2 "" H 4550 3250 50  0000 C CNN
F 3 "" H 4550 3250 50  0000 C CNN
	1    4550 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 2850 4850 3250
$Comp
L GND #PWR?
U 1 1 56A2CAAF
P 4850 3250
F 0 "#PWR?" H 4850 3000 50  0001 C CNN
F 1 "GND" H 4850 3100 50  0000 C CNN
F 2 "" H 4850 3250 50  0000 C CNN
F 3 "" H 4850 3250 50  0000 C CNN
	1    4850 3250
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 56A2CACC
P 5150 3100
F 0 "R?" V 5230 3100 50  0000 C CNN
F 1 "33k" V 5150 3100 50  0000 C CNN
F 2 "" V 5080 3100 50  0000 C CNN
F 3 "" H 5150 3100 50  0000 C CNN
	1    5150 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 2850 5150 2950
Wire Wire Line
	5150 3250 5150 3300
Wire Wire Line
	5150 3300 5400 3300
Wire Wire Line
	5400 3300 5400 3200
$Comp
L +5V #PWR?
U 1 1 56A2CB02
P 5400 3200
F 0 "#PWR?" H 5400 3050 50  0001 C CNN
F 1 "+5V" H 5400 3340 50  0000 C CNN
F 2 "" H 5400 3200 50  0000 C CNN
F 3 "" H 5400 3200 50  0000 C CNN
	1    5400 3200
	1    0    0    -1  
$EndComp
Connection ~ 5150 2900
Wire Wire Line
	5150 2900 5300 2900
Text Label 5300 2900 2    60   ~ 0
A
$Comp
L TCRT1000 U?
U 1 1 56A2CC86
P 6200 2450
F 0 "U?" H 6200 2450 60  0000 C CNN
F 1 "TCRT1000" H 6450 2650 60  0000 C CNN
F 2 "TCRT1000" H 5950 2650 60  0000 C CNN
F 3 "" H 6200 2450 60  0000 C CNN
	1    6200 2450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 56A2CC8C
P 5550 2700
F 0 "#PWR?" H 5550 2550 50  0001 C CNN
F 1 "+5V" H 5550 2840 50  0000 C CNN
F 2 "" H 5550 2700 50  0000 C CNN
F 3 "" H 5550 2700 50  0000 C CNN
	1    5550 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2700 5550 2850
Wire Wire Line
	5550 2850 5750 2850
$Comp
L R R?
U 1 1 56A2CC94
P 6050 3050
F 0 "R?" V 6130 3050 50  0000 C CNN
F 1 "180" V 6050 3050 50  0000 C CNN
F 2 "" V 5980 3050 50  0000 C CNN
F 3 "" H 6050 3050 50  0000 C CNN
	1    6050 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 2850 6050 2900
Wire Wire Line
	6050 3200 6050 3250
$Comp
L GND #PWR?
U 1 1 56A2CC9C
P 6050 3250
F 0 "#PWR?" H 6050 3000 50  0001 C CNN
F 1 "GND" H 6050 3100 50  0000 C CNN
F 2 "" H 6050 3250 50  0000 C CNN
F 3 "" H 6050 3250 50  0000 C CNN
	1    6050 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 2850 6350 3250
$Comp
L GND #PWR?
U 1 1 56A2CCA3
P 6350 3250
F 0 "#PWR?" H 6350 3000 50  0001 C CNN
F 1 "GND" H 6350 3100 50  0000 C CNN
F 2 "" H 6350 3250 50  0000 C CNN
F 3 "" H 6350 3250 50  0000 C CNN
	1    6350 3250
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 56A2CCA9
P 6650 3100
F 0 "R?" V 6730 3100 50  0000 C CNN
F 1 "33k" V 6650 3100 50  0000 C CNN
F 2 "" V 6580 3100 50  0000 C CNN
F 3 "" H 6650 3100 50  0000 C CNN
	1    6650 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 2850 6650 2950
Wire Wire Line
	6650 3250 6650 3300
Wire Wire Line
	6650 3300 6900 3300
Wire Wire Line
	6900 3300 6900 3200
$Comp
L +5V #PWR?
U 1 1 56A2CCB4
P 6900 3200
F 0 "#PWR?" H 6900 3050 50  0001 C CNN
F 1 "+5V" H 6900 3340 50  0000 C CNN
F 2 "" H 6900 3200 50  0000 C CNN
F 3 "" H 6900 3200 50  0000 C CNN
	1    6900 3200
	1    0    0    -1  
$EndComp
Connection ~ 6650 2900
Wire Wire Line
	6650 2900 6800 2900
Text Label 6800 2900 2    60   ~ 0
A
$Comp
L TCRT1000 U?
U 1 1 56A2CE7D
P 7750 2450
F 0 "U?" H 7750 2450 60  0000 C CNN
F 1 "TCRT1000" H 8000 2650 60  0000 C CNN
F 2 "TCRT1000" H 7500 2650 60  0000 C CNN
F 3 "" H 7750 2450 60  0000 C CNN
	1    7750 2450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 56A2CE83
P 7100 2700
F 0 "#PWR?" H 7100 2550 50  0001 C CNN
F 1 "+5V" H 7100 2840 50  0000 C CNN
F 2 "" H 7100 2700 50  0000 C CNN
F 3 "" H 7100 2700 50  0000 C CNN
	1    7100 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 2700 7100 2850
Wire Wire Line
	7100 2850 7300 2850
$Comp
L R R?
U 1 1 56A2CE8B
P 7600 3050
F 0 "R?" V 7680 3050 50  0000 C CNN
F 1 "180" V 7600 3050 50  0000 C CNN
F 2 "" V 7530 3050 50  0000 C CNN
F 3 "" H 7600 3050 50  0000 C CNN
	1    7600 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 2850 7600 2900
Wire Wire Line
	7600 3200 7600 3250
$Comp
L GND #PWR?
U 1 1 56A2CE93
P 7600 3250
F 0 "#PWR?" H 7600 3000 50  0001 C CNN
F 1 "GND" H 7600 3100 50  0000 C CNN
F 2 "" H 7600 3250 50  0000 C CNN
F 3 "" H 7600 3250 50  0000 C CNN
	1    7600 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 2850 7900 3250
$Comp
L GND #PWR?
U 1 1 56A2CE9A
P 7900 3250
F 0 "#PWR?" H 7900 3000 50  0001 C CNN
F 1 "GND" H 7900 3100 50  0000 C CNN
F 2 "" H 7900 3250 50  0000 C CNN
F 3 "" H 7900 3250 50  0000 C CNN
	1    7900 3250
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 56A2CEA0
P 8200 3100
F 0 "R?" V 8280 3100 50  0000 C CNN
F 1 "33k" V 8200 3100 50  0000 C CNN
F 2 "" V 8130 3100 50  0000 C CNN
F 3 "" H 8200 3100 50  0000 C CNN
	1    8200 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 2850 8200 2950
Wire Wire Line
	8200 3250 8200 3300
Wire Wire Line
	8200 3300 8450 3300
Wire Wire Line
	8450 3300 8450 3200
$Comp
L +5V #PWR?
U 1 1 56A2CEAB
P 8450 3200
F 0 "#PWR?" H 8450 3050 50  0001 C CNN
F 1 "+5V" H 8450 3340 50  0000 C CNN
F 2 "" H 8450 3200 50  0000 C CNN
F 3 "" H 8450 3200 50  0000 C CNN
	1    8450 3200
	1    0    0    -1  
$EndComp
Connection ~ 8200 2900
Wire Wire Line
	8200 2900 8350 2900
Text Label 8350 2900 2    60   ~ 0
A
$Comp
L TCRT1000 U?
U 1 1 56A2CEB4
P 4650 1000
F 0 "U?" H 4650 1000 60  0000 C CNN
F 1 "TCRT1000" H 4900 1200 60  0000 C CNN
F 2 "TCRT1000" H 4400 1200 60  0000 C CNN
F 3 "" H 4650 1000 60  0000 C CNN
	1    4650 1000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 56A2CEBA
P 4000 1250
F 0 "#PWR?" H 4000 1100 50  0001 C CNN
F 1 "+5V" H 4000 1390 50  0000 C CNN
F 2 "" H 4000 1250 50  0000 C CNN
F 3 "" H 4000 1250 50  0000 C CNN
	1    4000 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 1250 4000 1400
Wire Wire Line
	4000 1400 4200 1400
$Comp
L R R?
U 1 1 56A2CEC2
P 4500 1600
F 0 "R?" V 4580 1600 50  0000 C CNN
F 1 "180" V 4500 1600 50  0000 C CNN
F 2 "" V 4430 1600 50  0000 C CNN
F 3 "" H 4500 1600 50  0000 C CNN
	1    4500 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 1400 4500 1450
Wire Wire Line
	4500 1750 4500 1800
$Comp
L GND #PWR?
U 1 1 56A2CECA
P 4500 1800
F 0 "#PWR?" H 4500 1550 50  0001 C CNN
F 1 "GND" H 4500 1650 50  0000 C CNN
F 2 "" H 4500 1800 50  0000 C CNN
F 3 "" H 4500 1800 50  0000 C CNN
	1    4500 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 1400 4800 1800
$Comp
L GND #PWR?
U 1 1 56A2CED1
P 4800 1800
F 0 "#PWR?" H 4800 1550 50  0001 C CNN
F 1 "GND" H 4800 1650 50  0000 C CNN
F 2 "" H 4800 1800 50  0000 C CNN
F 3 "" H 4800 1800 50  0000 C CNN
	1    4800 1800
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 56A2CED7
P 5100 1650
F 0 "R?" V 5180 1650 50  0000 C CNN
F 1 "33k" V 5100 1650 50  0000 C CNN
F 2 "" V 5030 1650 50  0000 C CNN
F 3 "" H 5100 1650 50  0000 C CNN
	1    5100 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 1400 5100 1500
Wire Wire Line
	5100 1800 5100 1850
Wire Wire Line
	5100 1850 5350 1850
Wire Wire Line
	5350 1850 5350 1750
$Comp
L +5V #PWR?
U 1 1 56A2CEE2
P 5350 1750
F 0 "#PWR?" H 5350 1600 50  0001 C CNN
F 1 "+5V" H 5350 1890 50  0000 C CNN
F 2 "" H 5350 1750 50  0000 C CNN
F 3 "" H 5350 1750 50  0000 C CNN
	1    5350 1750
	1    0    0    -1  
$EndComp
Connection ~ 5100 1450
Wire Wire Line
	5100 1450 5250 1450
Text Label 5250 1450 2    60   ~ 0
A
$Comp
L TCRT1000 U?
U 1 1 56A2D0B4
P 6250 1000
F 0 "U?" H 6250 1000 60  0000 C CNN
F 1 "TCRT1000" H 6500 1200 60  0000 C CNN
F 2 "TCRT1000" H 6000 1200 60  0000 C CNN
F 3 "" H 6250 1000 60  0000 C CNN
	1    6250 1000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 56A2D0BA
P 5600 1250
F 0 "#PWR?" H 5600 1100 50  0001 C CNN
F 1 "+5V" H 5600 1390 50  0000 C CNN
F 2 "" H 5600 1250 50  0000 C CNN
F 3 "" H 5600 1250 50  0000 C CNN
	1    5600 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 1250 5600 1400
Wire Wire Line
	5600 1400 5800 1400
$Comp
L R R?
U 1 1 56A2D0C2
P 6100 1600
F 0 "R?" V 6180 1600 50  0000 C CNN
F 1 "180" V 6100 1600 50  0000 C CNN
F 2 "" V 6030 1600 50  0000 C CNN
F 3 "" H 6100 1600 50  0000 C CNN
	1    6100 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 1400 6100 1450
Wire Wire Line
	6100 1750 6100 1800
$Comp
L GND #PWR?
U 1 1 56A2D0CA
P 6100 1800
F 0 "#PWR?" H 6100 1550 50  0001 C CNN
F 1 "GND" H 6100 1650 50  0000 C CNN
F 2 "" H 6100 1800 50  0000 C CNN
F 3 "" H 6100 1800 50  0000 C CNN
	1    6100 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 1400 6400 1800
$Comp
L GND #PWR?
U 1 1 56A2D0D1
P 6400 1800
F 0 "#PWR?" H 6400 1550 50  0001 C CNN
F 1 "GND" H 6400 1650 50  0000 C CNN
F 2 "" H 6400 1800 50  0000 C CNN
F 3 "" H 6400 1800 50  0000 C CNN
	1    6400 1800
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 56A2D0D7
P 6700 1650
F 0 "R?" V 6780 1650 50  0000 C CNN
F 1 "33k" V 6700 1650 50  0000 C CNN
F 2 "" V 6630 1650 50  0000 C CNN
F 3 "" H 6700 1650 50  0000 C CNN
	1    6700 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 1400 6700 1500
Wire Wire Line
	6700 1800 6700 1850
Wire Wire Line
	6700 1850 6950 1850
Wire Wire Line
	6950 1850 6950 1750
$Comp
L +5V #PWR?
U 1 1 56A2D0E2
P 6950 1750
F 0 "#PWR?" H 6950 1600 50  0001 C CNN
F 1 "+5V" H 6950 1890 50  0000 C CNN
F 2 "" H 6950 1750 50  0000 C CNN
F 3 "" H 6950 1750 50  0000 C CNN
	1    6950 1750
	1    0    0    -1  
$EndComp
Connection ~ 6700 1450
Wire Wire Line
	6700 1450 6850 1450
Text Label 6850 1450 2    60   ~ 0
A
$Comp
L TCRT1000 U?
U 1 1 56A2D0EB
P 7750 1000
F 0 "U?" H 7750 1000 60  0000 C CNN
F 1 "TCRT1000" H 8000 1200 60  0000 C CNN
F 2 "TCRT1000" H 7500 1200 60  0000 C CNN
F 3 "" H 7750 1000 60  0000 C CNN
	1    7750 1000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 56A2D0F1
P 7100 1250
F 0 "#PWR?" H 7100 1100 50  0001 C CNN
F 1 "+5V" H 7100 1390 50  0000 C CNN
F 2 "" H 7100 1250 50  0000 C CNN
F 3 "" H 7100 1250 50  0000 C CNN
	1    7100 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1250 7100 1400
Wire Wire Line
	7100 1400 7300 1400
$Comp
L R R?
U 1 1 56A2D0F9
P 7600 1600
F 0 "R?" V 7680 1600 50  0000 C CNN
F 1 "180" V 7600 1600 50  0000 C CNN
F 2 "" V 7530 1600 50  0000 C CNN
F 3 "" H 7600 1600 50  0000 C CNN
	1    7600 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 1400 7600 1450
Wire Wire Line
	7600 1750 7600 1800
$Comp
L GND #PWR?
U 1 1 56A2D101
P 7600 1800
F 0 "#PWR?" H 7600 1550 50  0001 C CNN
F 1 "GND" H 7600 1650 50  0000 C CNN
F 2 "" H 7600 1800 50  0000 C CNN
F 3 "" H 7600 1800 50  0000 C CNN
	1    7600 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 1400 7900 1800
$Comp
L GND #PWR?
U 1 1 56A2D108
P 7900 1800
F 0 "#PWR?" H 7900 1550 50  0001 C CNN
F 1 "GND" H 7900 1650 50  0000 C CNN
F 2 "" H 7900 1800 50  0000 C CNN
F 3 "" H 7900 1800 50  0000 C CNN
	1    7900 1800
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 56A2D10E
P 8200 1650
F 0 "R?" V 8280 1650 50  0000 C CNN
F 1 "33k" V 8200 1650 50  0000 C CNN
F 2 "" V 8130 1650 50  0000 C CNN
F 3 "" H 8200 1650 50  0000 C CNN
	1    8200 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 1400 8200 1500
Wire Wire Line
	8200 1800 8200 1850
Wire Wire Line
	8200 1850 8450 1850
Wire Wire Line
	8450 1850 8450 1750
$Comp
L +5V #PWR?
U 1 1 56A2D119
P 8450 1750
F 0 "#PWR?" H 8450 1600 50  0001 C CNN
F 1 "+5V" H 8450 1890 50  0000 C CNN
F 2 "" H 8450 1750 50  0000 C CNN
F 3 "" H 8450 1750 50  0000 C CNN
	1    8450 1750
	1    0    0    -1  
$EndComp
Connection ~ 8200 1450
Wire Wire Line
	8200 1450 8350 1450
Text Label 8350 1450 2    60   ~ 0
A
$Comp
L R R?
U 1 1 56A2D2A8
P 4900 5300
F 0 "R?" V 4980 5300 50  0000 C CNN
F 1 "33k" V 4900 5300 50  0000 C CNN
F 2 "" V 4830 5300 50  0000 C CNN
F 3 "" H 4900 5300 50  0000 C CNN
	1    4900 5300
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 56A2D313
P 4900 5700
F 0 "R?" V 4980 5700 50  0000 C CNN
F 1 "33k" V 4900 5700 50  0000 C CNN
F 2 "" V 4830 5700 50  0000 C CNN
F 3 "" H 4900 5700 50  0000 C CNN
	1    4900 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 5150 4900 5100
$Comp
L +8V #PWR?
U 1 1 56A2D61F
P 4900 5100
F 0 "#PWR?" H 4900 4950 50  0001 C CNN
F 1 "+8V" H 4900 5240 50  0000 C CNN
F 2 "" H 4900 5100 50  0000 C CNN
F 3 "" H 4900 5100 50  0000 C CNN
	1    4900 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 5450 4900 5550
Wire Wire Line
	4900 5500 5550 5500
Connection ~ 4900 5500
Text Label 5550 5500 2    60   ~ 0
BatteryControl
Wire Wire Line
	4900 5850 4900 5900
$Comp
L GND #PWR?
U 1 1 56A2D85E
P 4900 5900
F 0 "#PWR?" H 4900 5650 50  0001 C CNN
F 1 "GND" H 4900 5750 50  0000 C CNN
F 2 "" H 4900 5900 50  0000 C CNN
F 3 "" H 4900 5900 50  0000 C CNN
	1    4900 5900
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P?
U 1 1 56A2D8DB
P 6550 5200
F 0 "P?" H 6550 5400 50  0000 C CNN
F 1 "CONN_01X03" V 6650 5200 50  0000 C CNN
F 2 "" H 6550 5200 50  0000 C CNN
F 3 "" H 6550 5200 50  0000 C CNN
	1    6550 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 5200 6350 5200
Wire Wire Line
	6300 5100 6300 5200
Wire Wire Line
	6300 5100 6350 5100
Wire Wire Line
	6300 5150 6200 5150
Wire Wire Line
	6200 5150 6200 5000
Connection ~ 6300 5150
$Comp
L +BATT #PWR?
U 1 1 56A2D9FC
P 6200 5000
F 0 "#PWR?" H 6200 4850 50  0001 C CNN
F 1 "+BATT" H 6200 5140 50  0000 C CNN
F 2 "" H 6200 5000 50  0000 C CNN
F 3 "" H 6200 5000 50  0000 C CNN
	1    6200 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 5300 6350 5300
Wire Wire Line
	6050 5300 6050 5200
$Comp
L +8V #PWR?
U 1 1 56A2DAA2
P 6050 5200
F 0 "#PWR?" H 6050 5050 50  0001 C CNN
F 1 "+8V" H 6050 5340 50  0000 C CNN
F 2 "" H 6050 5200 50  0000 C CNN
F 3 "" H 6050 5200 50  0000 C CNN
	1    6050 5200
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG?
U 1 1 56A2DAE2
P 6200 5350
F 0 "#FLG?" H 6200 5445 50  0001 C CNN
F 1 "PWR_FLAG" H 6200 5530 50  0000 C CNN
F 2 "" H 6200 5350 50  0000 C CNN
F 3 "" H 6200 5350 50  0000 C CNN
	1    6200 5350
	-1   0    0    1   
$EndComp
Wire Wire Line
	6200 5350 6200 5300
Connection ~ 6200 5300
$Comp
L CONN_01X02 P?
U 1 1 56A2DC3B
P 5400 6700
F 0 "P?" H 5400 6850 50  0000 C CNN
F 1 "CONN_01X02" V 5500 6700 50  0000 C CNN
F 2 "" H 5400 6700 50  0000 C CNN
F 3 "" H 5400 6700 50  0000 C CNN
	1    5400 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 6650 5200 6650
Wire Wire Line
	5100 6650 5100 6600
$Comp
L +BATT #PWR?
U 1 1 56A2DDA6
P 5100 6600
F 0 "#PWR?" H 5100 6450 50  0001 C CNN
F 1 "+BATT" H 5100 6740 50  0000 C CNN
F 2 "" H 5100 6600 50  0000 C CNN
F 3 "" H 5100 6600 50  0000 C CNN
	1    5100 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 6750 5200 6750
Wire Wire Line
	5100 6750 5100 6800
$Comp
L GND #PWR?
U 1 1 56A2DE56
P 5100 6800
F 0 "#PWR?" H 5100 6550 50  0001 C CNN
F 1 "GND" H 5100 6650 50  0000 C CNN
F 2 "" H 5100 6800 50  0000 C CNN
F 3 "" H 5100 6800 50  0000 C CNN
	1    5100 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 6750 4900 6850
Connection ~ 5100 6750
Wire Wire Line
	4900 6550 4900 6650
Connection ~ 5100 6650
$Comp
L PWR_FLAG #FLG?
U 1 1 56A2DFBD
P 4900 6550
F 0 "#FLG?" H 4900 6645 50  0001 C CNN
F 1 "PWR_FLAG" H 4900 6730 50  0000 C CNN
F 2 "" H 4900 6550 50  0000 C CNN
F 3 "" H 4900 6550 50  0000 C CNN
	1    4900 6550
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG?
U 1 1 56A2E096
P 4900 6850
F 0 "#FLG?" H 4900 6945 50  0001 C CNN
F 1 "PWR_FLAG" H 4900 7030 50  0000 C CNN
F 2 "" H 4900 6850 50  0000 C CNN
F 3 "" H 4900 6850 50  0000 C CNN
	1    4900 6850
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR?
U 1 1 56E34B56
P 7050 4800
F 0 "#PWR?" H 7050 4650 50  0001 C CNN
F 1 "+5V" H 7050 4940 50  0000 C CNN
F 2 "" H 7050 4800 50  0000 C CNN
F 3 "" H 7050 4800 50  0000 C CNN
	1    7050 4800
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 56E34BF4
P 7050 5050
F 0 "R?" V 7130 5050 50  0000 C CNN
F 1 "10k" V 7050 5050 50  0000 C CNN
F 2 "" V 6980 5050 50  0000 C CNN
F 3 "" H 7050 5050 50  0000 C CNN
	1    7050 5050
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH_SMALL SW?
U 1 1 56E34C3B
P 7150 5450
F 0 "SW?" H 7300 5560 50  0000 C CNN
F 1 "SW_PUSH_SMALL" H 7150 5371 50  0000 C CNN
F 2 "" H 7150 5450 50  0000 C CNN
F 3 "" H 7150 5450 50  0000 C CNN
	1    7150 5450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56E34CC6
P 7250 5700
F 0 "#PWR?" H 7250 5450 50  0001 C CNN
F 1 "GND" H 7250 5550 50  0000 C CNN
F 2 "" H 7250 5700 50  0000 C CNN
F 3 "" H 7250 5700 50  0000 C CNN
	1    7250 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 5700 7250 5550
Wire Wire Line
	7050 5350 7050 5200
Wire Wire Line
	7050 4900 7050 4800
$Comp
L +5V #PWR?
U 1 1 56E34F4A
P 7550 4800
F 0 "#PWR?" H 7550 4650 50  0001 C CNN
F 1 "+5V" H 7550 4940 50  0000 C CNN
F 2 "" H 7550 4800 50  0000 C CNN
F 3 "" H 7550 4800 50  0000 C CNN
	1    7550 4800
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 56E34F50
P 7550 5050
F 0 "R?" V 7630 5050 50  0000 C CNN
F 1 "10k" V 7550 5050 50  0000 C CNN
F 2 "" V 7480 5050 50  0000 C CNN
F 3 "" H 7550 5050 50  0000 C CNN
	1    7550 5050
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH_SMALL SW?
U 1 1 56E34F56
P 7650 5450
F 0 "SW?" H 7800 5560 50  0000 C CNN
F 1 "SW_PUSH_SMALL" H 7650 5371 50  0000 C CNN
F 2 "" H 7650 5450 50  0000 C CNN
F 3 "" H 7650 5450 50  0000 C CNN
	1    7650 5450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56E34F5C
P 7750 5700
F 0 "#PWR?" H 7750 5450 50  0001 C CNN
F 1 "GND" H 7750 5550 50  0000 C CNN
F 2 "" H 7750 5700 50  0000 C CNN
F 3 "" H 7750 5700 50  0000 C CNN
	1    7750 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 5700 7750 5550
Wire Wire Line
	7550 5350 7550 5200
Wire Wire Line
	7550 4900 7550 4800
$Comp
L +5V #PWR?
U 1 1 56E34FD1
P 8000 4800
F 0 "#PWR?" H 8000 4650 50  0001 C CNN
F 1 "+5V" H 8000 4940 50  0000 C CNN
F 2 "" H 8000 4800 50  0000 C CNN
F 3 "" H 8000 4800 50  0000 C CNN
	1    8000 4800
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 56E34FD7
P 8000 5050
F 0 "R?" V 8080 5050 50  0000 C CNN
F 1 "10k" V 8000 5050 50  0000 C CNN
F 2 "" V 7930 5050 50  0000 C CNN
F 3 "" H 8000 5050 50  0000 C CNN
	1    8000 5050
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH_SMALL SW?
U 1 1 56E34FDD
P 8100 5450
F 0 "SW?" H 8250 5560 50  0000 C CNN
F 1 "SW_PUSH_SMALL" H 8100 5371 50  0000 C CNN
F 2 "" H 8100 5450 50  0000 C CNN
F 3 "" H 8100 5450 50  0000 C CNN
	1    8100 5450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56E34FE3
P 8200 5700
F 0 "#PWR?" H 8200 5450 50  0001 C CNN
F 1 "GND" H 8200 5550 50  0000 C CNN
F 2 "" H 8200 5700 50  0000 C CNN
F 3 "" H 8200 5700 50  0000 C CNN
	1    8200 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 5700 8200 5550
Wire Wire Line
	8000 5350 8000 5200
Wire Wire Line
	8000 4900 8000 4800
$Comp
L REG_104-5 U?
U 1 1 56E35B60
P 9800 1100
F 0 "U?" H 9550 1300 60  0000 C CNN
F 1 "REG_104-5" H 9800 1400 60  0000 C CNN
F 2 "" H 9800 1100 60  0000 C CNN
F 3 "" H 9800 1100 60  0000 C CNN
	1    9800 1100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56E35C2B
P 9800 1650
F 0 "#PWR?" H 9800 1400 50  0001 C CNN
F 1 "GND" H 9800 1500 50  0000 C CNN
F 2 "" H 9800 1650 50  0000 C CNN
F 3 "" H 9800 1650 50  0000 C CNN
	1    9800 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 1650 9800 1550
Wire Wire Line
	9300 1000 9100 1000
$Comp
L +8V #PWR?
U 1 1 56E35FD9
P 9100 950
F 0 "#PWR?" H 9100 800 50  0001 C CNN
F 1 "+8V" H 9100 1090 50  0000 C CNN
F 2 "" H 9100 950 50  0000 C CNN
F 3 "" H 9100 950 50  0000 C CNN
	1    9100 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 1200 8900 1200
Wire Wire Line
	8900 1200 8900 950 
$Comp
L +5V #PWR?
U 1 1 56E36133
P 8900 950
F 0 "#PWR?" H 8900 800 50  0001 C CNN
F 1 "+5V" H 8900 1090 50  0000 C CNN
F 2 "" H 8900 950 50  0000 C CNN
F 3 "" H 8900 950 50  0000 C CNN
	1    8900 950 
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P?
U 1 1 56E363B9
P 6600 6250
F 0 "P?" H 6600 6400 50  0000 C CNN
F 1 "CONN_01X02" V 6700 6250 50  0000 C CNN
F 2 "" H 6600 6250 50  0000 C CNN
F 3 "" H 6600 6250 50  0000 C CNN
	1    6600 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 6300 6050 6300
Wire Wire Line
	6050 6300 6050 6150
$Comp
L +8V #PWR?
U 1 1 56E364C3
P 6050 6150
F 0 "#PWR?" H 6050 6000 50  0001 C CNN
F 1 "+8V" H 6050 6290 50  0000 C CNN
F 2 "" H 6050 6150 50  0000 C CNN
F 3 "" H 6050 6150 50  0000 C CNN
	1    6050 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 6200 6250 6200
Wire Wire Line
	6250 6200 6250 6150
$Comp
L +12V #PWR?
U 1 1 56E365A1
P 6250 6150
F 0 "#PWR?" H 6250 6000 50  0001 C CNN
F 1 "+12V" H 6250 6290 50  0000 C CNN
F 2 "" H 6250 6150 50  0000 C CNN
F 3 "" H 6250 6150 50  0000 C CNN
	1    6250 6150
	1    0    0    -1  
$EndComp
$Comp
L LM1084IT-ADJ/NOPB U?
U 1 1 56E365F3
P 9650 2550
F 0 "U?" H 9850 2350 50  0000 C CNN
F 1 "LM1084IT-ADJ/NOPB" H 9350 2750 50  0000 L CNN
F 2 "TO-220" H 9650 2650 50  0000 C CIN
F 3 "" H 9650 2550 50  0000 C CNN
	1    9650 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 2500 9000 2500
Wire Wire Line
	9000 2500 9000 2400
$Comp
L +12V #PWR?
U 1 1 56E36A35
P 9000 2400
F 0 "#PWR?" H 9000 2250 50  0001 C CNN
F 1 "+12V" H 9000 2540 50  0000 C CNN
F 2 "" H 9000 2400 50  0000 C CNN
F 3 "" H 9000 2400 50  0000 C CNN
	1    9000 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 2500 10750 2500
Wire Wire Line
	10550 2500 10550 2400
$Comp
L +9V #PWR?
U 1 1 56E36B99
P 10550 2400
F 0 "#PWR?" H 10550 2250 50  0001 C CNN
F 1 "+9V" H 10550 2540 50  0000 C CNN
F 2 "" H 10550 2400 50  0000 C CNN
F 3 "" H 10550 2400 50  0000 C CNN
	1    10550 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 2500 10300 2700
Connection ~ 10300 2500
$Comp
L R R?
U 1 1 56E36C80
P 10300 2850
F 0 "R?" V 10380 2850 50  0000 C CNN
F 1 "120" V 10300 2850 50  0000 C CNN
F 2 "" V 10230 2850 50  0000 C CNN
F 3 "" H 10300 2850 50  0000 C CNN
	1    10300 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 3000 10300 3300
Wire Wire Line
	10300 3100 9650 3100
Wire Wire Line
	9650 3100 9650 2800
Connection ~ 10300 3100
$Comp
L R R?
U 1 1 56E36EE6
P 10300 3450
F 0 "R?" V 10380 3450 50  0000 C CNN
F 1 "470" V 10300 3450 50  0000 C CNN
F 2 "" V 10230 3450 50  0000 C CNN
F 3 "" H 10300 3450 50  0000 C CNN
	1    10300 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 3600 10300 3700
$Comp
L R R?
U 1 1 56E370F0
P 10300 3850
F 0 "R?" V 10380 3850 50  0000 C CNN
F 1 "270" V 10300 3850 50  0000 C CNN
F 2 "" V 10230 3850 50  0000 C CNN
F 3 "" H 10300 3850 50  0000 C CNN
	1    10300 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 4000 10300 4100
$Comp
L GND #PWR?
U 1 1 56E372E2
P 10300 4100
F 0 "#PWR?" H 10300 3850 50  0001 C CNN
F 1 "GND" H 10300 3950 50  0000 C CNN
F 2 "" H 10300 4100 50  0000 C CNN
F 3 "" H 10300 4100 50  0000 C CNN
	1    10300 4100
	1    0    0    -1  
$EndComp
Connection ~ 10300 4050
$Comp
L JUMPER JP?
U 1 1 56E373DA
P 10800 3850
F 0 "JP?" H 10800 4000 50  0000 C CNN
F 1 "JUMPER" H 10800 3770 50  0000 C CNN
F 2 "" H 10800 3850 50  0000 C CNN
F 3 "" H 10800 3850 50  0000 C CNN
	1    10800 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	10300 4050 10600 4050
Wire Wire Line
	10600 3650 10300 3650
Connection ~ 10300 3650
Wire Wire Line
	10600 3650 10600 3550
Wire Wire Line
	10600 3550 10800 3550
Wire Wire Line
	10800 4150 10600 4150
Wire Wire Line
	10600 4150 10600 4050
Wire Wire Line
	10300 1000 10850 1000
Wire Wire Line
	10850 1000 10850 1100
$Comp
L GND #PWR?
U 1 1 56E37C66
P 10850 1100
F 0 "#PWR?" H 10850 850 50  0001 C CNN
F 1 "GND" H 10850 950 50  0000 C CNN
F 2 "" H 10850 1100 50  0000 C CNN
F 3 "" H 10850 1100 50  0000 C CNN
	1    10850 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 1200 10300 1350
Wire Wire Line
	10300 1350 10550 1350
Wire Wire Line
	10550 1350 10550 1250
$Comp
L +8V #PWR?
U 1 1 56E37DC2
P 10550 1250
F 0 "#PWR?" H 10550 1100 50  0001 C CNN
F 1 "+8V" H 10550 1390 50  0000 C CNN
F 2 "" H 10550 1250 50  0000 C CNN
F 3 "" H 10550 1250 50  0000 C CNN
	1    10550 1250
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 56E37ECA
P 9000 1600
F 0 "C?" H 9025 1700 50  0000 L CNN
F 1 "10u" H 9025 1500 50  0000 L CNN
F 2 "" H 9038 1450 50  0000 C CNN
F 3 "" H 9000 1600 50  0000 C CNN
	1    9000 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 1000 9100 950 
Wire Wire Line
	10400 1350 10400 1450
Connection ~ 10400 1350
$Comp
L CP C?
U 1 1 56E38183
P 10400 1600
F 0 "C?" H 10425 1700 50  0000 L CNN
F 1 "10u" H 10425 1500 50  0000 L CNN
F 2 "" H 10438 1450 50  0000 C CNN
F 3 "" H 10400 1600 50  0000 C CNN
	1    10400 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 56E38216
P 10400 1800
F 0 "#PWR?" H 10400 1550 50  0001 C CNN
F 1 "GND" H 10400 1650 50  0000 C CNN
F 2 "" H 10400 1800 50  0000 C CNN
F 3 "" H 10400 1800 50  0000 C CNN
	1    10400 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	10400 1750 10400 1800
$Comp
L C C?
U 1 1 56E38418
P 9300 1600
F 0 "C?" H 9325 1700 50  0000 L CNN
F 1 "0.1u" H 9325 1500 50  0000 L CNN
F 2 "" H 9338 1450 50  0000 C CNN
F 3 "" H 9300 1600 50  0000 C CNN
	1    9300 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 1450 9000 1350
Wire Wire Line
	9000 1350 9300 1350
Wire Wire Line
	9300 1350 9300 1450
Connection ~ 9150 1350
Wire Wire Line
	9150 1200 9150 1350
Connection ~ 9150 1200
Wire Wire Line
	9000 1750 9000 1800
Wire Wire Line
	9000 1800 9300 1800
Wire Wire Line
	9300 1800 9300 1750
Wire Wire Line
	9150 1800 9150 1850
Connection ~ 9150 1800
$Comp
L GND #PWR?
U 1 1 56E38880
P 9150 1850
F 0 "#PWR?" H 9150 1600 50  0001 C CNN
F 1 "GND" H 9150 1700 50  0000 C CNN
F 2 "" H 9150 1850 50  0000 C CNN
F 3 "" H 9150 1850 50  0000 C CNN
	1    9150 1850
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 56E38A77
P 9100 2750
F 0 "C?" H 9125 2850 50  0000 L CNN
F 1 "10u" H 9125 2650 50  0000 L CNN
F 2 "" H 9138 2600 50  0000 C CNN
F 3 "" H 9100 2750 50  0000 C CNN
	1    9100 2750
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 56E38B08
P 10600 2750
F 0 "C?" H 10625 2850 50  0000 L CNN
F 1 "10u" H 10625 2650 50  0000 L CNN
F 2 "" H 10638 2600 50  0000 C CNN
F 3 "" H 10600 2750 50  0000 C CNN
	1    10600 2750
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56E38C7A
P 10850 2750
F 0 "C?" H 10875 2850 50  0000 L CNN
F 1 "0.1u" H 10875 2650 50  0000 L CNN
F 2 "" H 10888 2600 50  0000 C CNN
F 3 "" H 10850 2750 50  0000 C CNN
	1    10850 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	10850 2600 10850 2550
Wire Wire Line
	10850 2550 10600 2550
Wire Wire Line
	10600 2550 10600 2600
Wire Wire Line
	10750 2500 10750 2550
Connection ~ 10750 2550
Connection ~ 10550 2500
Wire Wire Line
	10600 2900 10600 2950
Wire Wire Line
	10600 2950 10850 2950
Wire Wire Line
	10850 2950 10850 2900
Wire Wire Line
	10750 2950 10750 3000
Connection ~ 10750 2950
$Comp
L GND #PWR?
U 1 1 56E390E6
P 10750 3000
F 0 "#PWR?" H 10750 2750 50  0001 C CNN
F 1 "GND" H 10750 2850 50  0000 C CNN
F 2 "" H 10750 3000 50  0000 C CNN
F 3 "" H 10750 3000 50  0000 C CNN
	1    10750 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 2500 9100 2600
Connection ~ 9100 2500
Wire Wire Line
	9100 2900 9100 3000
$Comp
L GND #PWR?
U 1 1 56E392E4
P 9100 3000
F 0 "#PWR?" H 9100 2750 50  0001 C CNN
F 1 "GND" H 9100 2850 50  0000 C CNN
F 2 "" H 9100 3000 50  0000 C CNN
F 3 "" H 9100 3000 50  0000 C CNN
	1    9100 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 6550 1800 6550
Wire Wire Line
	1800 6550 1800 7050
Wire Wire Line
	1800 6650 1850 6650
Wire Wire Line
	1800 6750 1850 6750
Connection ~ 1800 6650
Connection ~ 1800 6750
$Comp
L GND #PWR?
U 1 1 56E39E6F
P 1800 7050
F 0 "#PWR?" H 1800 6800 50  0001 C CNN
F 1 "GND" H 1800 6900 50  0000 C CNN
F 2 "" H 1800 7050 50  0000 C CNN
F 3 "" H 1800 7050 50  0000 C CNN
	1    1800 7050
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 56E39FA9
P 1700 5250
F 0 "C?" H 1725 5350 50  0000 L CNN
F 1 "C" H 1725 5150 50  0000 L CNN
F 2 "" H 1738 5100 50  0000 C CNN
F 3 "" H 1700 5250 50  0000 C CNN
	1    1700 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 5050 1700 5050
Wire Wire Line
	1700 5050 1700 5100
Wire Wire Line
	1700 5400 1700 5450
$Comp
L GND #PWR?
U 1 1 56E3A200
P 1700 5450
F 0 "#PWR?" H 1700 5200 50  0001 C CNN
F 1 "GND" H 1700 5300 50  0000 C CNN
F 2 "" H 1700 5450 50  0000 C CNN
F 3 "" H 1700 5450 50  0000 C CNN
	1    1700 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 4750 1850 4750
Wire Wire Line
	1800 4350 1800 4750
Wire Wire Line
	1800 4550 1850 4550
Wire Wire Line
	1800 4450 1850 4450
Connection ~ 1800 4550
Connection ~ 1800 4450
$Comp
L VCC #PWR?
U 1 1 56E3A676
P 1800 4350
F 0 "#PWR?" H 1800 4200 50  0001 C CNN
F 1 "VCC" H 1800 4500 50  0000 C CNN
F 2 "" H 1800 4350 50  0000 C CNN
F 3 "" H 1800 4350 50  0000 C CNN
	1    1800 4350
	1    0    0    -1  
$EndComp
$Comp
L DVR8833 U?
U 1 1 56E3AA68
P 2000 1600
F 0 "U?" H 2000 1500 50  0000 C CNN
F 1 "DVR8833" H 2000 1700 50  0000 C CNN
F 2 "MODULE" H 2000 1600 50  0001 C CNN
F 3 "DOCUMENTATION" H 2000 1600 50  0001 C CNN
	1    2000 1600
	1    0    0    -1  
$EndComp
$EndSCHEMATC
