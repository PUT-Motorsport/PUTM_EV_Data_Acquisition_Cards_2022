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
Wire Wire Line
	3650 2250 3650 1850
$Comp
L Device:R R4
U 1 1 617AC265
P 3650 2400
F 0 "R4" H 3720 2446 50  0000 L CNN
F 1 "R" H 3720 2355 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3580 2400 50  0001 C CNN
F 3 "~" H 3650 2400 50  0001 C CNN
	1    3650 2400
	1    0    0    -1  
$EndComp
Text HLabel 3650 1850 2    50   Input ~ 0
+uC
Text HLabel 4650 2550 2    50   Output ~ 0
+s1
Wire Wire Line
	4650 2550 3650 2550
$EndSCHEMATC
