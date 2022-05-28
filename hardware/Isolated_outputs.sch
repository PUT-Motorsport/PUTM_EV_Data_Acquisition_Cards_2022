EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
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
L Isolator:H11L1 U?
U 1 1 61924BAB
P 5500 3200
F 0 "U?" H 5844 3246 50  0000 L CNN
F 1 "H11L1" H 5844 3155 50  0000 L CNN
F 2 "" H 5410 3200 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/H11L3M-D.PDF" H 5410 3200 50  0001 C CNN
	1    5500 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 619257A7
P 5500 3550
F 0 "#PWR?" H 5500 3300 50  0001 C CNN
F 1 "GND" H 5505 3377 50  0000 C CNN
F 2 "" H 5500 3550 50  0001 C CNN
F 3 "" H 5500 3550 50  0001 C CNN
	1    5500 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 3500 5500 3550
Wire Wire Line
	5200 3300 5200 3500
Wire Wire Line
	5200 3500 5500 3500
Connection ~ 5500 3500
$Comp
L Device:R R?
U 1 1 619263FD
P 4750 3100
F 0 "R?" V 4543 3100 50  0000 C CNN
F 1 "1k" V 4634 3100 50  0000 C CNN
F 2 "" V 4680 3100 50  0001 C CNN
F 3 "~" H 4750 3100 50  0001 C CNN
	1    4750 3100
	0    1    1    0   
$EndComp
Text HLabel 6050 3200 2    50   Output ~ 0
OUT
Wire Wire Line
	6050 3200 5800 3200
Text HLabel 4600 3100 0    50   Input ~ 0
IN
$Comp
L power:+3.3V #PWR?
U 1 1 61927D24
P 5500 2900
F 0 "#PWR?" H 5500 2750 50  0001 C CNN
F 1 "+3.3V" H 5515 3073 50  0000 C CNN
F 2 "" H 5500 2900 50  0001 C CNN
F 3 "" H 5500 2900 50  0001 C CNN
	1    5500 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D?
U 1 1 6192A988
P 4900 3300
F 0 "D?" V 4939 3182 50  0000 R CNN
F 1 "LED" V 4848 3182 50  0000 R CNN
F 2 "" H 4900 3300 50  0001 C CNN
F 3 "~" H 4900 3300 50  0001 C CNN
	1    4900 3300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4900 3450 4900 3500
Wire Wire Line
	4900 3500 5200 3500
Connection ~ 5200 3500
Wire Wire Line
	5200 3100 4900 3100
Wire Wire Line
	4900 3100 4900 3150
Connection ~ 4900 3100
$EndSCHEMATC
