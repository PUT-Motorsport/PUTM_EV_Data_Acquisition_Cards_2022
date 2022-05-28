EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 22 23
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
L Interface_CAN_LIN:MCP2558FD-xSN U19
U 1 1 61BAC77F
P 5350 3450
F 0 "U19" H 5500 3900 50  0000 C CNN
F 1 "ATA6561-GAQW-N" H 5750 3800 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 5350 2850 50  0001 C CNN
F 3 "https://pl.mouser.com/datasheet/2/268/ATA6560_ATA6561_High_Speed_CAN_Transceiver_DS20005-1384933.pdf" H 5350 3450 50  0001 C CNN
	1    5350 3450
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR074
U 1 1 61BAD4BF
P 3300 2600
F 0 "#PWR074" H 3300 2450 50  0001 C CNN
F 1 "+3.3V" H 3315 2773 50  0000 C CNN
F 2 "" H 3300 2600 50  0001 C CNN
F 3 "" H 3300 2600 50  0001 C CNN
	1    3300 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:Ferrite_Bead FB6
U 1 1 61BADA22
P 3300 2800
F 0 "FB6" H 3437 2846 50  0000 L CNN
F 1 "Ferrite_Bead" H 3437 2755 50  0000 L CNN
F 2 "" V 3230 2800 50  0001 C CNN
F 3 "~" H 3300 2800 50  0001 C CNN
	1    3300 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 2650 3300 2600
$Comp
L Device:C C32
U 1 1 61BADF25
P 3300 3150
F 0 "C32" H 3415 3196 50  0000 L CNN
F 1 "C" H 3415 3105 50  0000 L CNN
F 2 "" H 3338 3000 50  0001 C CNN
F 3 "~" H 3300 3150 50  0001 C CNN
	1    3300 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 3000 3300 2950
$Comp
L power:GND #PWR075
U 1 1 61BAE5E9
P 3300 3350
F 0 "#PWR075" H 3300 3100 50  0001 C CNN
F 1 "GND" H 3305 3177 50  0000 C CNN
F 2 "" H 3300 3350 50  0001 C CNN
F 3 "" H 3300 3350 50  0001 C CNN
	1    3300 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 3350 3300 3300
Wire Wire Line
	3300 3000 3750 3000
Connection ~ 3300 3000
Text Label 3750 3000 2    39   ~ 0
3.3V_CAN
Text Label 4550 3550 0    39   ~ 0
3.3V_CAN
Wire Wire Line
	5350 3050 5350 2900
$Comp
L power:GND #PWR077
U 1 1 61BAF945
P 5350 3900
F 0 "#PWR077" H 5350 3650 50  0001 C CNN
F 1 "GND" H 5355 3727 50  0000 C CNN
F 2 "" H 5350 3900 50  0001 C CNN
F 3 "" H 5350 3900 50  0001 C CNN
	1    5350 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 3900 5350 3850
$Comp
L Device:D_TVS_x2_AAC D15
U 1 1 61BB8AD7
P 7500 3400
F 0 "D15" H 7500 3617 50  0000 C CNN
F 1 "PESD2CAN.215" H 7500 3526 50  0000 C CNN
F 2 "" H 7350 3400 50  0001 C CNN
F 3 "~" H 7350 3400 50  0001 C CNN
	1    7500 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 3350 6250 3350
Wire Wire Line
	5850 3550 6250 3550
Text HLabel 6250 3550 2    39   BiDi ~ 0
CANL
Text HLabel 6250 3350 2    39   BiDi ~ 0
CANH
Text HLabel 7150 3400 0    39   BiDi ~ 0
CANH
Text HLabel 7850 3400 2    39   BiDi ~ 0
CANL
Wire Wire Line
	7500 3550 7500 3600
$Comp
L power:GND #PWR078
U 1 1 61BBE23E
P 7500 3600
F 0 "#PWR078" H 7500 3350 50  0001 C CNN
F 1 "GND" H 7505 3427 50  0000 C CNN
F 2 "" H 7500 3600 50  0001 C CNN
F 3 "" H 7500 3600 50  0001 C CNN
	1    7500 3600
	1    0    0    -1  
$EndComp
Text HLabel 8850 2950 0    39   BiDi ~ 0
CANH
Text HLabel 8850 3800 2    39   BiDi ~ 0
CANL
$Comp
L Device:Jumper_NO_Small JP1
U 1 1 61BBF77B
P 8850 3200
F 0 "JP1" V 8804 3248 50  0000 L CNN
F 1 "Jumper_NO_Small" V 8895 3248 50  0000 L CNN
F 2 "" H 8850 3200 50  0001 C CNN
F 3 "~" H 8850 3200 50  0001 C CNN
	1    8850 3200
	0    1    1    0   
$EndComp
$Comp
L Device:R R49
U 1 1 61BBFF4B
P 8850 3500
F 0 "R49" H 8920 3546 50  0000 L CNN
F 1 "R" H 8920 3455 50  0000 L CNN
F 2 "" V 8780 3500 50  0001 C CNN
F 3 "~" H 8850 3500 50  0001 C CNN
	1    8850 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 3350 8850 3300
Wire Wire Line
	8850 3100 8850 2950
Wire Wire Line
	8850 3650 8850 3800
Wire Wire Line
	4850 3250 4650 3250
Wire Wire Line
	4850 3350 4650 3350
Wire Wire Line
	4550 3550 4850 3550
Wire Wire Line
	4850 3650 4750 3650
Wire Wire Line
	4750 3650 4750 3750
$Comp
L power:GND #PWR076
U 1 1 61BC602A
P 4750 3750
F 0 "#PWR076" H 4750 3500 50  0001 C CNN
F 1 "GND" H 4755 3577 50  0000 C CNN
F 2 "" H 4750 3750 50  0001 C CNN
F 3 "" H 4750 3750 50  0001 C CNN
	1    4750 3750
	1    0    0    -1  
$EndComp
Text HLabel 4650 3250 0    39   BiDi ~ 0
TX
Text HLabel 4650 3350 0    39   BiDi ~ 0
RX
$EndSCHEMATC
