EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 13
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
L Amplifier_Operational:MCP6001x-LT U?
U 1 1 61B37ED7
P 4400 2600
F 0 "U?" H 4744 2646 50  0000 L CNN
F 1 "MCP6001x-LT" H 4744 2555 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-353_SC-70-5" H 4300 2400 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21733j.pdf" H 4400 2600 50  0001 C CNN
	1    4400 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 61B383AC
P 4450 3300
F 0 "R?" V 4243 3300 50  0000 C CNN
F 1 "1M" V 4334 3300 50  0000 C CNN
F 2 "" V 4380 3300 50  0001 C CNN
F 3 "~" H 4450 3300 50  0001 C CNN
	1    4450 3300
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 61B38C91
P 2750 2500
F 0 "R?" V 2543 2500 50  0000 C CNN
F 1 "1k" V 2634 2500 50  0000 C CNN
F 2 "" V 2680 2500 50  0001 C CNN
F 3 "~" H 2750 2500 50  0001 C CNN
	1    2750 2500
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 61B394A5
P 3250 2700
F 0 "R?" H 3180 2654 50  0000 R CNN
F 1 "2k" H 3180 2745 50  0000 R CNN
F 2 "" V 3180 2700 50  0001 C CNN
F 3 "~" H 3250 2700 50  0001 C CNN
	1    3250 2700
	-1   0    0    1   
$EndComp
$Comp
L Device:C C?
U 1 1 61B39F73
P 3000 2700
F 0 "C?" H 3115 2746 50  0000 L CNN
F 1 "100n" H 3115 2655 50  0000 L CNN
F 2 "" H 3038 2550 50  0001 C CNN
F 3 "~" H 3000 2700 50  0001 C CNN
	1    3000 2700
	1    0    0    -1  
$EndComp
$Comp
L Diode:BZX84Cxx D?
U 1 1 61B3AA62
P 3500 2700
F 0 "D?" V 3454 2780 50  0000 L CNN
F 1 "BZX84Cxx" V 3545 2780 50  0000 L CNN
F 2 "Diode_SMD:D_SOT-23_ANK" H 3500 2525 50  0001 C CNN
F 3 "https://diotec.com/tl_files/diotec/files/pdf/datasheets/bzx84c2v4.pdf" H 3500 2700 50  0001 C CNN
	1    3500 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	4100 2500 3500 2500
Wire Wire Line
	3000 2550 3000 2500
Connection ~ 3000 2500
Wire Wire Line
	3000 2500 2900 2500
Wire Wire Line
	3250 2550 3250 2500
Connection ~ 3250 2500
Wire Wire Line
	3250 2500 3000 2500
Wire Wire Line
	3500 2550 3500 2500
Connection ~ 3500 2500
Wire Wire Line
	3500 2500 3250 2500
Wire Wire Line
	3000 2850 3250 2850
Connection ~ 3250 2850
Wire Wire Line
	3250 2850 3500 2850
$Comp
L power:GND #PWR?
U 1 1 61B3C394
P 3250 2900
F 0 "#PWR?" H 3250 2650 50  0001 C CNN
F 1 "GND" H 3255 2727 50  0000 C CNN
F 2 "" H 3250 2900 50  0001 C CNN
F 3 "" H 3250 2900 50  0001 C CNN
	1    3250 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2900 3250 2850
Wire Wire Line
	4700 3300 4600 3300
Wire Wire Line
	4300 3300 4100 3300
$Comp
L power:GND #PWR?
U 1 1 61B3ED2C
P 4300 2950
F 0 "#PWR?" H 4300 2700 50  0001 C CNN
F 1 "GND" H 4305 2777 50  0000 C CNN
F 2 "" H 4300 2950 50  0001 C CNN
F 3 "" H 4300 2950 50  0001 C CNN
	1    4300 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2950 4300 2900
Wire Wire Line
	4100 2700 4100 3300
Wire Wire Line
	4700 2600 4700 3300
Text HLabel 2450 2500 0    50   Input ~ 0
IN
Text HLabel 5550 2600 2    50   Output ~ 0
OUT
Wire Wire Line
	5550 2600 4700 2600
Connection ~ 4700 2600
Wire Wire Line
	2600 2500 2450 2500
$Comp
L power:+3.3V #PWR?
U 1 1 61B44C39
P 4300 2200
F 0 "#PWR?" H 4300 2050 50  0001 C CNN
F 1 "+3.3V" H 4315 2373 50  0000 C CNN
F 2 "" H 4300 2200 50  0001 C CNN
F 3 "" H 4300 2200 50  0001 C CNN
	1    4300 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2200 4300 2250
$Comp
L Device:C C?
U 1 1 61B84F5F
P 4550 2250
F 0 "C?" V 4298 2250 50  0000 C CNN
F 1 "C" V 4389 2250 50  0000 C CNN
F 2 "" H 4588 2100 50  0001 C CNN
F 3 "~" H 4550 2250 50  0001 C CNN
	1    4550 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	4400 2250 4300 2250
Connection ~ 4300 2250
Wire Wire Line
	4300 2250 4300 2300
$Comp
L power:GND #PWR?
U 1 1 61B8626F
P 4700 2300
F 0 "#PWR?" H 4700 2050 50  0001 C CNN
F 1 "GND" H 4705 2127 50  0000 C CNN
F 2 "" H 4700 2300 50  0001 C CNN
F 3 "" H 4700 2300 50  0001 C CNN
	1    4700 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 2300 4700 2250
$EndSCHEMATC
