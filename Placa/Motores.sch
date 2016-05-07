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
Sheet 3 4
Title ""
Date "5 sep 2015"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L DVR8833 U1
U 1 1 55EAE3E5
P 4400 3800
F 0 "U1" H 4400 3700 50  0000 C CNN
F 1 "DVR8833" H 4400 3900 50  0000 C CNN
F 2 "Housings_SSOP:TSSOP-16_4.4x5mm_Pitch0.65mm" H 4400 3800 50  0001 C CNN
F 3 "DOCUMENTATION" H 4400 3800 50  0001 C CNN
	1    4400 3800
	1    0    0    -1  
$EndComp
$Comp
L DVR8833 U2
U 1 1 55EAE3F2
P 7150 3800
F 0 "U2" H 7150 3700 50  0000 C CNN
F 1 "DVR8833" H 7150 3900 50  0000 C CNN
F 2 "Housings_SSOP:TSSOP-16_4.4x5mm_Pitch0.65mm" H 7150 3800 50  0001 C CNN
F 3 "DOCUMENTATION" H 7150 3800 50  0001 C CNN
	1    7150 3800
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 55EAFAA0
P 4350 2150
F 0 "C2" H 4350 2250 40  0000 L CNN
F 1 "0.01u" H 4356 2065 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4388 2000 30  0000 C CNN
F 3 "~" H 4350 2150 60  0000 C CNN
	1    4350 2150
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 55EAFAAF
P 5250 2200
F 0 "C4" H 5250 2300 40  0000 L CNN
F 1 "2.2u" H 5256 2115 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5288 2050 30  0000 C CNN
F 3 "~" H 5250 2200 60  0000 C CNN
	1    5250 2200
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 55EAFACD
P 5700 2200
F 0 "C5" H 5700 2300 40  0000 L CNN
F 1 "2.2u" H 5706 2115 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5738 2050 30  0000 C CNN
F 3 "~" H 5700 2200 60  0000 C CNN
	1    5700 2200
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 55EAFAEB
P 4750 2150
F 0 "C3" H 4750 2250 40  0000 L CNN
F 1 "0.01u" H 4756 2065 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4788 2000 30  0000 C CNN
F 3 "~" H 4750 2150 60  0000 C CNN
	1    4750 2150
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 55EAFB5A
P 7550 1950
F 0 "R1" V 7630 1950 40  0000 C CNN
F 1 "1k" V 7557 1951 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 7480 1950 30  0001 C CNN
F 3 "~" H 7550 1950 30  0000 C CNN
	1    7550 1950
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 55EAFB78
P 7550 2500
F 0 "D1" H 7550 2600 50  0000 C CNN
F 1 "LED" H 7550 2400 50  0000 C CNN
F 2 "LEDs:LED_0805" H 7550 2500 60  0001 C CNN
F 3 "~" H 7550 2500 60  0000 C CNN
	1    7550 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR037
U 1 1 55EB036E
P 3100 3700
F 0 "#PWR037" H 3100 3700 30  0001 C CNN
F 1 "GND" H 3100 3630 30  0001 C CNN
F 2 "" H 3100 3700 60  0000 C CNN
F 3 "" H 3100 3700 60  0000 C CNN
	1    3100 3700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR038
U 1 1 55EB037B
P 3100 4000
F 0 "#PWR038" H 3100 4000 30  0001 C CNN
F 1 "GND" H 3100 3930 30  0001 C CNN
F 2 "" H 3100 4000 60  0000 C CNN
F 3 "" H 3100 4000 60  0000 C CNN
	1    3100 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR039
U 1 1 55EB0384
P 5850 3700
F 0 "#PWR039" H 5850 3700 30  0001 C CNN
F 1 "GND" H 5850 3630 30  0001 C CNN
F 2 "" H 5850 3700 60  0000 C CNN
F 3 "" H 5850 3700 60  0000 C CNN
	1    5850 3700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR040
U 1 1 55EB0393
P 5850 4000
F 0 "#PWR040" H 5850 4000 30  0001 C CNN
F 1 "GND" H 5850 3930 30  0001 C CNN
F 2 "" H 5850 4000 60  0000 C CNN
F 3 "" H 5850 4000 60  0000 C CNN
	1    5850 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR041
U 1 1 55EB047B
P 5600 3800
F 0 "#PWR041" H 5600 3800 30  0001 C CNN
F 1 "GND" H 5600 3730 30  0001 C CNN
F 2 "" H 5600 3800 60  0000 C CNN
F 3 "" H 5600 3800 60  0000 C CNN
	1    5600 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR042
U 1 1 55EB0487
P 8550 3800
F 0 "#PWR042" H 8550 3800 30  0001 C CNN
F 1 "GND" H 8550 3730 30  0001 C CNN
F 2 "" H 8550 3800 60  0000 C CNN
F 3 "" H 8550 3800 60  0000 C CNN
	1    8550 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 3550 3650 3550
Wire Wire Line
	3650 3650 3100 3650
Wire Wire Line
	3100 3650 3100 3700
Wire Wire Line
	3600 3750 3650 3750
Wire Wire Line
	3600 3850 3650 3850
Wire Wire Line
	3650 3950 3100 3950
Wire Wire Line
	3100 3950 3100 4000
Wire Wire Line
	3600 4050 3650 4050
Wire Wire Line
	5150 3550 5200 3550
Wire Wire Line
	5150 4050 5200 4050
Wire Wire Line
	5150 3850 5200 3850
Wire Wire Line
	5150 3750 5600 3750
Wire Wire Line
	5600 3750 5600 3800
Wire Wire Line
	5150 3950 5450 3950
Wire Wire Line
	6350 3550 6400 3550
Wire Wire Line
	6400 3650 5850 3650
Wire Wire Line
	5850 3650 5850 3700
Wire Wire Line
	6400 3750 6350 3750
Wire Wire Line
	6400 3850 6350 3850
Wire Wire Line
	6400 3950 5850 3950
Wire Wire Line
	5850 3950 5850 4000
Wire Wire Line
	6350 4050 6400 4050
Wire Wire Line
	7900 3450 7950 3450
Wire Wire Line
	7900 3550 7950 3550
Wire Wire Line
	7900 3750 8550 3750
Wire Wire Line
	8550 3750 8550 3800
Wire Wire Line
	7900 3850 7950 3850
Wire Wire Line
	7900 3950 8250 3950
Wire Wire Line
	3900 1950 4750 1950
Wire Wire Line
	4550 1850 4550 1950
Connection ~ 4550 1950
Wire Wire Line
	4750 2300 4750 2450
Wire Wire Line
	4350 2300 4350 2450
$Comp
L GND #PWR043
U 1 1 55EB1201
P 3900 2450
F 0 "#PWR043" H 3900 2450 30  0001 C CNN
F 1 "GND" H 3900 2380 30  0001 C CNN
F 2 "" H 3900 2450 60  0000 C CNN
F 3 "" H 3900 2450 60  0000 C CNN
	1    3900 2450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR044
U 1 1 55EB12DF
P 5250 2450
F 0 "#PWR044" H 5250 2450 30  0001 C CNN
F 1 "GND" H 5250 2380 30  0001 C CNN
F 2 "" H 5250 2450 60  0000 C CNN
F 3 "" H 5250 2450 60  0000 C CNN
	1    5250 2450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR045
U 1 1 55EB12E5
P 5700 2450
F 0 "#PWR045" H 5700 2450 30  0001 C CNN
F 1 "GND" H 5700 2380 30  0001 C CNN
F 2 "" H 5700 2450 60  0000 C CNN
F 3 "" H 5700 2450 60  0000 C CNN
	1    5700 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 1950 5250 2050
Wire Wire Line
	5250 2350 5250 2450
Wire Wire Line
	5700 1950 5700 2050
Wire Wire Line
	5700 2350 5700 2450
Wire Wire Line
	6000 3450 6400 3450
Wire Wire Line
	3250 3450 3650 3450
Wire Wire Line
	3000 3450 3150 3450
Wire Wire Line
	3050 4150 3650 4150
Wire Wire Line
	5900 4150 6400 4150
Wire Wire Line
	7950 3650 7900 3650
Wire Wire Line
	5200 3650 5150 3650
Wire Wire Line
	5150 4150 5200 4150
Wire Wire Line
	5150 3450 5200 3450
Wire Wire Line
	7900 4050 7950 4050
Wire Wire Line
	7900 4150 7950 4150
Connection ~ 4350 1950
Wire Wire Line
	3900 2300 3900 2450
$Comp
L GND #PWR046
U 1 1 55EB18C4
P 7550 2800
F 0 "#PWR046" H 7550 2800 30  0001 C CNN
F 1 "GND" H 7550 2730 30  0001 C CNN
F 2 "" H 7550 2800 60  0000 C CNN
F 3 "" H 7550 2800 60  0000 C CNN
	1    7550 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 2100 7550 2300
Wire Wire Line
	7550 2700 7550 2800
Wire Wire Line
	7550 1650 7550 1800
$Comp
L CONN_01X01 P4
U 1 1 571ABD6A
P 2850 4150
F 0 "P4" H 2850 4250 50  0000 C CNN
F 1 "CONN_01X01" V 2950 4150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 2850 4150 50  0001 C CNN
F 3 "" H 2850 4150 50  0000 C CNN
	1    2850 4150
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P6
U 1 1 571ABE3D
P 5700 4150
F 0 "P6" H 5700 4250 50  0000 C CNN
F 1 "CONN_01X01" V 5800 4150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 5700 4150 50  0001 C CNN
F 3 "" H 5700 4150 50  0000 C CNN
	1    5700 4150
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 P7
U 1 1 571ABEF8
P 5950 3200
F 0 "P7" H 5950 3350 50  0000 C CNN
F 1 "CONN_01X02" V 6050 3200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 5950 3200 50  0001 C CNN
F 3 "" H 5950 3200 50  0000 C CNN
	1    5950 3200
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X02 P5
U 1 1 571ABF5C
P 3200 3200
F 0 "P5" H 3200 3350 50  0000 C CNN
F 1 "CONN_01X02" V 3300 3200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3200 3200 50  0001 C CNN
F 3 "" H 3200 3200 50  0000 C CNN
	1    3200 3200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3150 3450 3150 3400
Wire Wire Line
	3250 3400 3250 3450
Wire Wire Line
	5900 3450 5900 3400
Wire Wire Line
	6000 3400 6000 3450
Wire Wire Line
	4350 2000 4350 1950
Wire Wire Line
	4750 1950 4750 2000
$Comp
L CP C6
U 1 1 571AC7CC
P 3900 2150
F 0 "C6" H 3925 2250 50  0000 L CNN
F 1 "10uF" H 3925 2050 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Radial_D5_L11_P2.5" H 3938 2000 50  0001 C CNN
F 3 "" H 3900 2150 50  0000 C CNN
	1    3900 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 2000 3900 1950
Text HLabel 7950 3450 2    60   Input ~ 0
M2IN2
Text HLabel 7950 3550 2    60   Input ~ 0
M2IN1
Text HLabel 7950 3650 2    60   Input ~ 0
M2VINT
Text HLabel 7950 3850 2    60   Input ~ 0
VM
Text HLabel 7950 4050 2    60   Input ~ 0
M2IN2
Text HLabel 7950 4150 2    60   Input ~ 0
M2IN1
Text HLabel 5850 3450 0    60   Input ~ 0
VM
Wire Wire Line
	5850 3450 5900 3450
Text HLabel 6350 3550 0    60   Input ~ 0
M2OUT1
Text HLabel 6350 3750 0    60   Input ~ 0
M2OUT2
Text HLabel 6350 3850 0    60   Input ~ 0
M2OUT2
Text HLabel 6350 4050 0    60   Input ~ 0
M2OUT1
Text HLabel 5200 3450 2    60   Input ~ 0
M1IN2
Text HLabel 5200 3550 2    60   Input ~ 0
M1IN1
Text HLabel 5200 3650 2    60   Input ~ 0
M1VINT
Text HLabel 5200 3850 2    60   Input ~ 0
VM
Text HLabel 5200 4050 2    60   Input ~ 0
M1IN2
Text HLabel 5200 4150 2    60   Input ~ 0
M1IN1
Text HLabel 3600 3550 0    60   Input ~ 0
M1OUT1
Text HLabel 3600 3750 0    60   Input ~ 0
M1OUT2
Text HLabel 3600 3850 0    60   Input ~ 0
M1OUT2
Text HLabel 3600 4050 0    60   Input ~ 0
M1OUT1
Text HLabel 7550 1650 0    60   Input ~ 0
VM
Text HLabel 5250 1950 0    60   Input ~ 0
M1VINT
Text HLabel 5700 1950 0    60   Input ~ 0
M2VINT
Text HLabel 4550 1850 0    60   Input ~ 0
VM
Text HLabel 3000 3450 0    60   Input ~ 0
VM
Text Label 4350 2450 2    60   ~ 0
M1VCP
Text Label 5450 3950 2    60   ~ 0
M1VCP
Text Label 8250 3950 2    60   ~ 0
M2VCP
Text Label 4750 2450 2    60   ~ 0
M2VCP
$EndSCHEMATC
