EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 23
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
L Sensor_Motion:LSM6DS3 U3
U 1 1 61B66B41
P 6250 3950
F 0 "U3" H 6894 3996 50  0000 L CNN
F 1 "IMS330DHCX" H 6894 3905 50  0000 L CNN
F 2 "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y" H 5850 3250 50  0001 L CNN
F 3 "www.st.com/resource/en/datasheet/lsm6ds3.pdf" H 6350 3300 50  0001 C CNN
	1    6250 3950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C17
U 1 1 61B69FE6
P 4050 2850
F 0 "C17" V 4302 2850 50  0000 C CNN
F 1 "C" V 4211 2850 50  0000 C CNN
F 2 "" H 4088 2700 50  0001 C CNN
F 3 "~" H 4050 2850 50  0001 C CNN
	1    4050 2850
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C18
U 1 1 61B6A93F
P 4050 3250
F 0 "C18" V 4302 3250 50  0000 C CNN
F 1 "C" V 4211 3250 50  0000 C CNN
F 2 "" H 4088 3100 50  0001 C CNN
F 3 "~" H 4050 3250 50  0001 C CNN
	1    4050 3250
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR019
U 1 1 61B6BFF6
P 3900 3450
F 0 "#PWR019" H 3900 3200 50  0001 C CNN
F 1 "GND" H 3905 3277 50  0000 C CNN
F 2 "" H 3900 3450 50  0001 C CNN
F 3 "" H 3900 3450 50  0001 C CNN
	1    3900 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 61B6D070
P 6300 4650
F 0 "#PWR022" H 6300 4400 50  0001 C CNN
F 1 "GND" H 6305 4477 50  0000 C CNN
F 2 "" H 6300 4650 50  0001 C CNN
F 3 "" H 6300 4650 50  0001 C CNN
	1    6300 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 4550 6300 4550
Wire Wire Line
	6300 4550 6300 4650
Wire Wire Line
	6300 4550 6250 4550
Connection ~ 6300 4550
$Comp
L power:GND #PWR021
U 1 1 61B6EA1E
P 5400 3850
F 0 "#PWR021" H 5400 3600 50  0001 C CNN
F 1 "GND" H 5405 3677 50  0000 C CNN
F 2 "" H 5400 3850 50  0001 C CNN
F 3 "" H 5400 3850 50  0001 C CNN
	1    5400 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 3850 5400 3850
Wire Wire Line
	5650 3750 5400 3750
Wire Wire Line
	5400 3750 5400 3850
Connection ~ 5400 3850
Wire Wire Line
	5650 4050 5200 4050
Wire Wire Line
	5650 4150 5200 4150
Wire Wire Line
	5650 4250 5200 4250
Text HLabel 5200 4050 0    50   Input ~ 0
SPI_MOSI
Text HLabel 5200 4150 0    50   Input ~ 0
SPI_CLK
Text HLabel 5200 4250 0    50   Input ~ 0
SPI_CS
Wire Wire Line
	6850 3650 7050 3650
Wire Wire Line
	6850 3750 7050 3750
Text HLabel 7050 3650 2    50   Output ~ 0
INT1
Text HLabel 7050 3750 2    50   Output ~ 0
INT2
$Comp
L power:+3.3V #PWR020
U 1 1 61BA6295
P 4200 2200
F 0 "#PWR020" H 4200 2050 50  0001 C CNN
F 1 "+3.3V" H 4215 2373 50  0000 C CNN
F 2 "" H 4200 2200 50  0001 C CNN
F 3 "" H 4200 2200 50  0001 C CNN
	1    4200 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 2850 3900 3250
Connection ~ 3900 3250
Wire Wire Line
	3900 3250 3900 3450
$Comp
L Device:Ferrite_Bead FB4
U 1 1 61BAD506
P 4200 2450
F 0 "FB4" H 4337 2496 50  0000 L CNN
F 1 "Ferrite_Bead" H 4337 2405 50  0000 L CNN
F 2 "" V 4130 2450 50  0001 C CNN
F 3 "~" H 4200 2450 50  0001 C CNN
	1    4200 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 2200 4200 2300
Wire Wire Line
	4200 2600 4200 2850
Connection ~ 4200 2850
Wire Wire Line
	4200 2850 4200 3250
Wire Wire Line
	4200 2850 4600 2850
Text Label 4600 2850 0    79   ~ 0
3.3V_ISM
Wire Wire Line
	6350 3100 6250 3100
Wire Wire Line
	6250 3100 6250 3350
Text Label 6350 3100 0    79   ~ 0
3.3V_ISM
Wire Wire Line
	6350 3350 6350 3100
Text HLabel 5200 3650 0    50   Input ~ 0
SPI_MISO
Wire Wire Line
	5200 3650 5650 3650
$EndSCHEMATC
