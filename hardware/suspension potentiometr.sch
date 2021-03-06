EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 22 25
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
L power:GND #PWR?
U 1 1 6226A43C
P 6850 3650
AR Path="/61B3782A/6226A43C" Ref="#PWR?"  Part="1" 
AR Path="/61B81C92/6226A43C" Ref="#PWR?"  Part="1" 
AR Path="/61B830EB/6226A43C" Ref="#PWR?"  Part="1" 
AR Path="/61B89762/6226A43C" Ref="#PWR?"  Part="1" 
AR Path="/61B8AB81/6226A43C" Ref="#PWR?"  Part="1" 
AR Path="/61B8BFD3/6226A43C" Ref="#PWR?"  Part="1" 
AR Path="/62268C11/6226A43C" Ref="#PWR038"  Part="1" 
AR Path="/62278FB5/6226A43C" Ref="#PWR?"  Part="1" 
AR Path="/6228228B/6226A43C" Ref="#PWR044"  Part="1" 
F 0 "#PWR038" H 6850 3400 50  0001 C CNN
F 1 "GND" H 6855 3477 50  0000 C CNN
F 2 "" H 6850 3650 50  0001 C CNN
F 3 "" H 6850 3650 50  0001 C CNN
	1    6850 3650
	1    0    0    -1  
$EndComp
Text HLabel 4000 3250 0    50   Input ~ 0
POT
Text HLabel 8100 3250 2    50   Output ~ 0
OUT
Wire Wire Line
	4150 3250 4000 3250
$Comp
L power:GND #PWR?
U 1 1 6226A445
P 7250 2950
AR Path="/61B3782A/6226A445" Ref="#PWR?"  Part="1" 
AR Path="/61B81C92/6226A445" Ref="#PWR?"  Part="1" 
AR Path="/61B830EB/6226A445" Ref="#PWR?"  Part="1" 
AR Path="/61B89762/6226A445" Ref="#PWR?"  Part="1" 
AR Path="/61B8AB81/6226A445" Ref="#PWR?"  Part="1" 
AR Path="/61B8BFD3/6226A445" Ref="#PWR?"  Part="1" 
AR Path="/62268C11/6226A445" Ref="#PWR039"  Part="1" 
AR Path="/62278FB5/6226A445" Ref="#PWR?"  Part="1" 
AR Path="/6228228B/6226A445" Ref="#PWR045"  Part="1" 
F 0 "#PWR039" H 7250 2700 50  0001 C CNN
F 1 "GND" H 7255 2777 50  0000 C CNN
F 2 "" H 7250 2950 50  0001 C CNN
F 3 "" H 7250 2950 50  0001 C CNN
	1    7250 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 2950 7250 2900
Wire Wire Line
	6850 2900 6850 2950
Connection ~ 6850 2900
$Comp
L Device:C C?
U 1 1 6226A44F
P 7100 2900
AR Path="/61B3782A/6226A44F" Ref="C?"  Part="1" 
AR Path="/61B81C92/6226A44F" Ref="C?"  Part="1" 
AR Path="/61B830EB/6226A44F" Ref="C?"  Part="1" 
AR Path="/61B89762/6226A44F" Ref="C?"  Part="1" 
AR Path="/61B8AB81/6226A44F" Ref="C?"  Part="1" 
AR Path="/61B8BFD3/6226A44F" Ref="C?"  Part="1" 
AR Path="/62268C11/6226A44F" Ref="C16"  Part="1" 
AR Path="/62278FB5/6226A44F" Ref="C?"  Part="1" 
AR Path="/6228228B/6226A44F" Ref="C19"  Part="1" 
F 0 "C16" V 6848 2900 50  0000 C CNN
F 1 "100n" V 6939 2900 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7138 2750 50  0001 C CNN
F 3 "~" H 7100 2900 50  0001 C CNN
	1    7100 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	6850 2850 6850 2900
Wire Wire Line
	6950 2900 6850 2900
$Comp
L Amplifier_Operational:OPA340NA U?
U 1 1 6226A457
P 6950 3250
AR Path="/61B830EB/6226A457" Ref="U?"  Part="1" 
AR Path="/61B8BFD3/6226A457" Ref="U?"  Part="1" 
AR Path="/61B81C92/6226A457" Ref="U?"  Part="1" 
AR Path="/62268C11/6226A457" Ref="U10"  Part="1" 
AR Path="/62278FB5/6226A457" Ref="U?"  Part="1" 
AR Path="/6228228B/6226A457" Ref="U11"  Part="1" 
F 0 "U10" H 7294 3296 50  0000 L CNN
F 1 "TSX7191" H 7294 3205 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 6850 3050 50  0001 L CNN
F 3 "https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjP8r6k07_0AhXMsaQKHaufCNsQFnoECBAQAQ&url=https%3A%2F%2Fwww.st.com%2Fresource%2Fen%2Fdatasheet%2Ftsx7191.pdf&usg=AOvVaw36G1EYx2FJDEKu96iJ_PiG" H 6950 3450 50  0001 C CNN
	1    6950 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 6226A45D
P 7000 4100
AR Path="/61B830EB/6226A45D" Ref="R?"  Part="1" 
AR Path="/61B8BFD3/6226A45D" Ref="R?"  Part="1" 
AR Path="/61B81C92/6226A45D" Ref="R?"  Part="1" 
AR Path="/62268C11/6226A45D" Ref="R23"  Part="1" 
AR Path="/62278FB5/6226A45D" Ref="R?"  Part="1" 
AR Path="/6228228B/6226A45D" Ref="R31"  Part="1" 
F 0 "R23" V 6793 4100 50  0000 C CNN
F 1 "47k" V 6884 4100 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6930 4100 50  0001 C CNN
F 3 "~" H 7000 4100 50  0001 C CNN
	1    7000 4100
	0    1    1    0   
$EndComp
Wire Wire Line
	7250 4100 7150 4100
Wire Wire Line
	7250 3250 7250 4100
Wire Wire Line
	6850 4100 6650 4100
Wire Wire Line
	6650 3350 6650 4100
$Comp
L Device:C C?
U 1 1 6226A467
P 6950 4450
AR Path="/61B830EB/6226A467" Ref="C?"  Part="1" 
AR Path="/61B8BFD3/6226A467" Ref="C?"  Part="1" 
AR Path="/61B81C92/6226A467" Ref="C?"  Part="1" 
AR Path="/62268C11/6226A467" Ref="C15"  Part="1" 
AR Path="/62278FB5/6226A467" Ref="C?"  Part="1" 
AR Path="/6228228B/6226A467" Ref="C18"  Part="1" 
F 0 "C15" V 6698 4450 50  0000 C CNN
F 1 "1nF" V 6789 4450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6988 4300 50  0001 C CNN
F 3 "~" H 6950 4450 50  0001 C CNN
	1    6950 4450
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 6226A473
P 5450 3150
AR Path="/61B830EB/6226A473" Ref="R?"  Part="1" 
AR Path="/61B8BFD3/6226A473" Ref="R?"  Part="1" 
AR Path="/61B81C92/6226A473" Ref="R?"  Part="1" 
AR Path="/62268C11/6226A473" Ref="R20"  Part="1" 
AR Path="/62278FB5/6226A473" Ref="R?"  Part="1" 
AR Path="/6228228B/6226A473" Ref="R28"  Part="1" 
F 0 "R20" V 5243 3150 50  0000 C CNN
F 1 "10k" V 5334 3150 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5380 3150 50  0001 C CNN
F 3 "~" H 5450 3150 50  0001 C CNN
	1    5450 3150
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 6226A479
P 5450 3350
AR Path="/61B830EB/6226A479" Ref="R?"  Part="1" 
AR Path="/61B8BFD3/6226A479" Ref="R?"  Part="1" 
AR Path="/61B81C92/6226A479" Ref="R?"  Part="1" 
AR Path="/62268C11/6226A479" Ref="R21"  Part="1" 
AR Path="/62278FB5/6226A479" Ref="R?"  Part="1" 
AR Path="/6228228B/6226A479" Ref="R29"  Part="1" 
F 0 "R21" V 5550 3350 50  0000 C CNN
F 1 "10k" V 5650 3350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5380 3350 50  0001 C CNN
F 3 "~" H 5450 3350 50  0001 C CNN
	1    5450 3350
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 6226A47F
P 6200 3600
AR Path="/61B830EB/6226A47F" Ref="R?"  Part="1" 
AR Path="/61B8BFD3/6226A47F" Ref="R?"  Part="1" 
AR Path="/61B81C92/6226A47F" Ref="R?"  Part="1" 
AR Path="/62268C11/6226A47F" Ref="R22"  Part="1" 
AR Path="/62278FB5/6226A47F" Ref="R?"  Part="1" 
AR Path="/6228228B/6226A47F" Ref="R30"  Part="1" 
F 0 "R22" H 6270 3646 50  0000 L CNN
F 1 "47k" H 6270 3555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6130 3600 50  0001 C CNN
F 3 "~" H 6200 3600 50  0001 C CNN
	1    6200 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 6226A485
P 5900 3600
AR Path="/61B830EB/6226A485" Ref="C?"  Part="1" 
AR Path="/61B8BFD3/6226A485" Ref="C?"  Part="1" 
AR Path="/61B81C92/6226A485" Ref="C?"  Part="1" 
AR Path="/62268C11/6226A485" Ref="C14"  Part="1" 
AR Path="/62278FB5/6226A485" Ref="C?"  Part="1" 
AR Path="/6228228B/6226A485" Ref="C17"  Part="1" 
F 0 "C14" H 6015 3646 50  0000 L CNN
F 1 "1nF" H 6015 3555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5938 3450 50  0001 C CNN
F 3 "~" H 5900 3600 50  0001 C CNN
	1    5900 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 3150 6200 3150
Wire Wire Line
	6650 3350 5600 3350
Connection ~ 6650 3350
Wire Wire Line
	5900 3450 6200 3450
Wire Wire Line
	6200 3450 6200 3150
Connection ~ 6200 3450
Connection ~ 6200 3150
Wire Wire Line
	6200 3150 5600 3150
$Comp
L power:GND #PWR?
U 1 1 6226A493
P 5900 3850
AR Path="/61B830EB/6226A493" Ref="#PWR?"  Part="1" 
AR Path="/61B8BFD3/6226A493" Ref="#PWR?"  Part="1" 
AR Path="/61B81C92/6226A493" Ref="#PWR?"  Part="1" 
AR Path="/62268C11/6226A493" Ref="#PWR036"  Part="1" 
AR Path="/62278FB5/6226A493" Ref="#PWR?"  Part="1" 
AR Path="/6228228B/6226A493" Ref="#PWR042"  Part="1" 
F 0 "#PWR036" H 5900 3600 50  0001 C CNN
F 1 "GND" H 5905 3677 50  0000 C CNN
F 2 "" H 5900 3850 50  0001 C CNN
F 3 "" H 5900 3850 50  0001 C CNN
	1    5900 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6226A499
P 6200 3850
AR Path="/61B830EB/6226A499" Ref="#PWR?"  Part="1" 
AR Path="/61B8BFD3/6226A499" Ref="#PWR?"  Part="1" 
AR Path="/61B81C92/6226A499" Ref="#PWR?"  Part="1" 
AR Path="/62268C11/6226A499" Ref="#PWR037"  Part="1" 
AR Path="/62278FB5/6226A499" Ref="#PWR?"  Part="1" 
AR Path="/6228228B/6226A499" Ref="#PWR043"  Part="1" 
F 0 "#PWR037" H 6200 3600 50  0001 C CNN
F 1 "GND" H 6205 3677 50  0000 C CNN
F 2 "" H 6200 3850 50  0001 C CNN
F 3 "" H 6200 3850 50  0001 C CNN
	1    6200 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 3850 6200 3750
Wire Wire Line
	5900 3750 5900 3850
$Comp
L Device:R R?
U 1 1 6226A4A1
P 7800 3250
AR Path="/61B830EB/6226A4A1" Ref="R?"  Part="1" 
AR Path="/61B8BFD3/6226A4A1" Ref="R?"  Part="1" 
AR Path="/61B81C92/6226A4A1" Ref="R?"  Part="1" 
AR Path="/62268C11/6226A4A1" Ref="R24"  Part="1" 
AR Path="/62278FB5/6226A4A1" Ref="R?"  Part="1" 
AR Path="/6228228B/6226A4A1" Ref="R36"  Part="1" 
F 0 "R24" V 7593 3250 50  0000 C CNN
F 1 "100R" V 7684 3250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7730 3250 50  0001 C CNN
F 3 "~" H 7800 3250 50  0001 C CNN
	1    7800 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 3250 7950 3250
Wire Wire Line
	7650 3250 7250 3250
Connection ~ 7250 3250
$Comp
L Device:R R?
U 1 1 6226A4AA
P 4150 3000
AR Path="/61B830EB/6226A4AA" Ref="R?"  Part="1" 
AR Path="/61B8BFD3/6226A4AA" Ref="R?"  Part="1" 
AR Path="/61B81C92/6226A4AA" Ref="R?"  Part="1" 
AR Path="/62268C11/6226A4AA" Ref="R8"  Part="1" 
AR Path="/62278FB5/6226A4AA" Ref="R?"  Part="1" 
AR Path="/6228228B/6226A4AA" Ref="R25"  Part="1" 
F 0 "R8" H 4220 3046 50  0000 L CNN
F 1 "5k" H 4220 2955 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4080 3000 50  0001 C CNN
F 3 "~" H 4150 3000 50  0001 C CNN
	1    4150 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 6226A4B0
P 4600 3000
AR Path="/61B830EB/6226A4B0" Ref="R?"  Part="1" 
AR Path="/61B8BFD3/6226A4B0" Ref="R?"  Part="1" 
AR Path="/61B81C92/6226A4B0" Ref="R?"  Part="1" 
AR Path="/62268C11/6226A4B0" Ref="R18"  Part="1" 
AR Path="/62278FB5/6226A4B0" Ref="R?"  Part="1" 
AR Path="/6228228B/6226A4B0" Ref="R26"  Part="1" 
F 0 "R18" H 4670 3046 50  0000 L CNN
F 1 "5k" H 4670 2955 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4530 3000 50  0001 C CNN
F 3 "~" H 4600 3000 50  0001 C CNN
	1    4600 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 6226A4B6
P 4600 3550
AR Path="/61B830EB/6226A4B6" Ref="R?"  Part="1" 
AR Path="/61B8BFD3/6226A4B6" Ref="R?"  Part="1" 
AR Path="/61B81C92/6226A4B6" Ref="R?"  Part="1" 
AR Path="/62268C11/6226A4B6" Ref="R19"  Part="1" 
AR Path="/62278FB5/6226A4B6" Ref="R?"  Part="1" 
AR Path="/6228228B/6226A4B6" Ref="R27"  Part="1" 
F 0 "R19" H 4670 3596 50  0000 L CNN
F 1 "5k" H 4670 3505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4530 3550 50  0001 C CNN
F 3 "~" H 4600 3550 50  0001 C CNN
	1    4600 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 2850 4150 2750
Wire Wire Line
	4150 2750 4400 2750
Wire Wire Line
	4600 2750 4600 2850
Wire Wire Line
	4600 3150 4600 3250
Wire Wire Line
	4600 3700 4600 3800
$Comp
L power:GND #PWR?
U 1 1 6226A4C1
P 4600 3800
AR Path="/61B830EB/6226A4C1" Ref="#PWR?"  Part="1" 
AR Path="/61B8BFD3/6226A4C1" Ref="#PWR?"  Part="1" 
AR Path="/61B81C92/6226A4C1" Ref="#PWR?"  Part="1" 
AR Path="/62268C11/6226A4C1" Ref="#PWR035"  Part="1" 
AR Path="/62278FB5/6226A4C1" Ref="#PWR?"  Part="1" 
AR Path="/6228228B/6226A4C1" Ref="#PWR041"  Part="1" 
F 0 "#PWR035" H 4600 3550 50  0001 C CNN
F 1 "GND" H 4605 3627 50  0000 C CNN
F 2 "" H 4600 3800 50  0001 C CNN
F 3 "" H 4600 3800 50  0001 C CNN
	1    4600 3800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 6226A4C7
P 4400 2600
AR Path="/61B830EB/6226A4C7" Ref="#PWR?"  Part="1" 
AR Path="/61B8BFD3/6226A4C7" Ref="#PWR?"  Part="1" 
AR Path="/61B81C92/6226A4C7" Ref="#PWR?"  Part="1" 
AR Path="/62268C11/6226A4C7" Ref="#PWR016"  Part="1" 
AR Path="/62278FB5/6226A4C7" Ref="#PWR?"  Part="1" 
AR Path="/6228228B/6226A4C7" Ref="#PWR040"  Part="1" 
F 0 "#PWR016" H 4400 2450 50  0001 C CNN
F 1 "+5V" H 4415 2773 50  0000 C CNN
F 2 "" H 4400 2600 50  0001 C CNN
F 3 "" H 4400 2600 50  0001 C CNN
	1    4400 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 2600 4400 2750
Connection ~ 4400 2750
Wire Wire Line
	4400 2750 4600 2750
Wire Wire Line
	4150 3150 4150 3250
Text Label 4150 3250 0    39   ~ 0
IN-
Text Label 4600 3250 2    39   ~ 0
IN+
Wire Wire Line
	4600 3250 4500 3250
Connection ~ 4600 3250
Wire Wire Line
	4600 3250 4600 3400
Wire Wire Line
	4150 3250 4250 3250
Connection ~ 4150 3250
Text Label 5050 3150 0    39   ~ 0
IN+
Wire Wire Line
	5300 3150 5050 3150
Text Label 5050 3350 0    39   ~ 0
IN-
Wire Wire Line
	5300 3350 5050 3350
$Comp
L power:+3.3V #PWR0123
U 1 1 622EB1F7
P 6850 2850
AR Path="/62268C11/622EB1F7" Ref="#PWR0123"  Part="1" 
AR Path="/6228228B/622EB1F7" Ref="#PWR0124"  Part="1" 
F 0 "#PWR0123" H 6850 2700 50  0001 C CNN
F 1 "+3.3V" H 6865 3023 50  0000 C CNN
F 2 "" H 6850 2850 50  0001 C CNN
F 3 "" H 6850 2850 50  0001 C CNN
	1    6850 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 3550 6850 3650
Connection ~ 6650 4100
Connection ~ 7250 4100
Wire Wire Line
	7250 4100 7250 4450
Wire Wire Line
	6650 4100 6650 4450
Wire Wire Line
	7100 4450 7250 4450
Wire Wire Line
	6800 4450 6650 4450
$EndSCHEMATC
