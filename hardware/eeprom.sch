EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 25
Title "EEPROM"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L AT24CSW020-STUM-T:M24C01-WMN U2
U 1 1 61B9A311
P 5350 2800
F 0 "U2" H 5100 3200 50  0000 C CNN
F 1 "M24C01-WMN" H 5050 3100 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 5350 3150 50  0001 C CNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/b0/d8/50/40/5a/85/49/6f/DM00071904.pdf/files/DM00071904.pdf/jcr:content/translations/en.DM00071904.pdf" H 5400 2300 50  0001 C CNN
	1    5350 2800
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR017
U 1 1 61B9A9EF
P 4150 1950
F 0 "#PWR017" H 4150 1800 50  0001 C CNN
F 1 "+3.3V" H 4165 2123 50  0000 C CNN
F 2 "" H 4150 1950 50  0001 C CNN
F 3 "" H 4150 1950 50  0001 C CNN
	1    4150 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 61B9CBFB
P 4150 2700
F 0 "#PWR018" H 4150 2450 50  0001 C CNN
F 1 "GND" H 4155 2527 50  0000 C CNN
F 2 "" H 4150 2700 50  0001 C CNN
F 3 "" H 4150 2700 50  0001 C CNN
	1    4150 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 2700 4150 2650
Wire Wire Line
	5750 2700 6250 2700
Wire Wire Line
	5750 2800 5900 2800
$Comp
L Device:R R9
U 1 1 61B9D782
P 5900 2150
F 0 "R9" H 5970 2196 50  0000 L CNN
F 1 "4.7k" H 5970 2105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5830 2150 50  0001 C CNN
F 3 "~" H 5900 2150 50  0001 C CNN
	1    5900 2150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 61B9E032
P 6250 2150
F 0 "R10" H 6320 2196 50  0000 L CNN
F 1 "4.7k" H 6320 2105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6180 2150 50  0001 C CNN
F 3 "~" H 6250 2150 50  0001 C CNN
	1    6250 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 2300 5900 2800
Connection ~ 5900 2800
Wire Wire Line
	5900 2800 6350 2800
Wire Wire Line
	6250 2300 6250 2700
Connection ~ 6250 2700
Wire Wire Line
	6250 2700 6350 2700
$Comp
L power:+3.3V #PWR020
U 1 1 61BA0103
P 6050 1850
F 0 "#PWR020" H 6050 1700 50  0001 C CNN
F 1 "+3.3V" H 6065 2023 50  0000 C CNN
F 2 "" H 6050 1850 50  0001 C CNN
F 3 "" H 6050 1850 50  0001 C CNN
	1    6050 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 1850 6050 1900
Wire Wire Line
	6050 1900 5900 1900
Wire Wire Line
	5900 1900 5900 2000
Wire Wire Line
	6050 1900 6250 1900
Wire Wire Line
	6250 1900 6250 2000
Connection ~ 6050 1900
Text HLabel 6350 2700 2    50   BiDi ~ 0
SDA
Text HLabel 6350 2800 2    50   BiDi ~ 0
SCL
$Comp
L power:GND #PWR019
U 1 1 61BA1A12
P 5350 3150
F 0 "#PWR019" H 5350 2900 50  0001 C CNN
F 1 "GND" H 5355 2977 50  0000 C CNN
F 2 "" H 5350 3150 50  0001 C CNN
F 3 "" H 5350 3150 50  0001 C CNN
	1    5350 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 3150 5350 3100
$Comp
L Device:C C13
U 1 1 61BBFFFD
P 4150 2500
F 0 "C13" H 4265 2546 50  0000 L CNN
F 1 "100nF" H 4265 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4188 2350 50  0001 C CNN
F 3 "~" H 4150 2500 50  0001 C CNN
	1    4150 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:Ferrite_Bead FB4
U 1 1 61BC287E
P 4150 2150
F 0 "FB4" H 4287 2196 50  0000 L CNN
F 1 "60R120MHz" H 4287 2105 50  0000 L CNN
F 2 "Inductor_SMD:L_0603_1608Metric" V 4080 2150 50  0001 C CNN
F 3 "~" H 4150 2150 50  0001 C CNN
	1    4150 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 1950 4150 2000
Wire Wire Line
	4150 2300 4150 2350
Connection ~ 4150 2350
Text Label 3250 2350 0    79   ~ 0
+3.3V_ee
Wire Wire Line
	3250 2350 3850 2350
Text Label 5350 1950 3    79   ~ 0
+3.3V_ee
NoConn ~ 5750 2900
NoConn ~ 4950 2900
NoConn ~ 4950 2800
NoConn ~ 4950 2700
Wire Wire Line
	5350 2500 5350 1950
$Comp
L power:PWR_FLAG #FLG04
U 1 1 61BA7258
P 3850 2350
F 0 "#FLG04" H 3850 2425 50  0001 C CNN
F 1 "PWR_FLAG" H 3850 2523 50  0000 C CNN
F 2 "" H 3850 2350 50  0001 C CNN
F 3 "~" H 3850 2350 50  0001 C CNN
	1    3850 2350
	1    0    0    -1  
$EndComp
Connection ~ 3850 2350
Wire Wire Line
	3850 2350 4150 2350
$EndSCHEMATC
