EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 21 25
Title "Digital input"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R R11
U 1 1 619263FD
P 5350 3800
AR Path="/618D845A/619263FD" Ref="R11"  Part="1" 
AR Path="/619331FA/619263FD" Ref="R?"  Part="1" 
AR Path="/61933DA6/619263FD" Ref="R?"  Part="1" 
AR Path="/61936967/619263FD" Ref="R?"  Part="1" 
AR Path="/61937516/619263FD" Ref="R?"  Part="1" 
AR Path="/6193809A/619263FD" Ref="R?"  Part="1" 
AR Path="/619397D5/619263FD" Ref="R?"  Part="1" 
AR Path="/619A3A9D/619263FD" Ref="R?"  Part="1" 
AR Path="/61B6360A/619263FD" Ref="R?"  Part="1" 
AR Path="/61B64A46/619263FD" Ref="R?"  Part="1" 
AR Path="/61B70D73/619263FD" Ref="R?"  Part="1" 
AR Path="/61B723F8/619263FD" Ref="R?"  Part="1" 
AR Path="/61B73A44/619263FD" Ref="R?"  Part="1" 
AR Path="/61B813B4/619263FD" Ref="R?"  Part="1" 
AR Path="/61B82A11/619263FD" Ref="R?"  Part="1" 
AR Path="/61B865A3/619263FD" Ref="R?"  Part="1" 
AR Path="/61B1E4D3/619263FD" Ref="R12"  Part="1" 
AR Path="/61B1FB44/619263FD" Ref="R46"  Part="1" 
AR Path="/61B211D6/619263FD" Ref="R13"  Part="1" 
AR Path="/61B227D2/619263FD" Ref="R14"  Part="1" 
AR Path="/61B23D59/619263FD" Ref="R15"  Part="1" 
AR Path="/61B2B5D4/619263FD" Ref="R16"  Part="1" 
AR Path="/61B2CBB8/619263FD" Ref="R17"  Part="1" 
AR Path="/61B2E4E5/619263FD" Ref="R?"  Part="1" 
AR Path="/61C30A7A/619263FD" Ref="R45"  Part="1" 
F 0 "R46" V 5143 3800 50  0000 C CNN
F 1 "12k" V 5234 3800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5280 3800 50  0001 C CNN
F 3 "~" H 5350 3800 50  0001 C CNN
	1    5350 3800
	0    1    1    0   
$EndComp
Text HLabel 5150 3800 0    50   Input ~ 0
IN
$Comp
L power:+3.3V #PWR021
U 1 1 61927D24
P 6050 3400
AR Path="/618D845A/61927D24" Ref="#PWR021"  Part="1" 
AR Path="/619331FA/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61933DA6/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61936967/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61937516/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/6193809A/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/619397D5/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/619A3A9D/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61B6360A/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61B64A46/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61B70D73/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61B723F8/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61B73A44/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61B813B4/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61B82A11/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61B865A3/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61B1E4D3/61927D24" Ref="#PWR023"  Part="1" 
AR Path="/61B1FB44/61927D24" Ref="#PWR093"  Part="1" 
AR Path="/61B211D6/61927D24" Ref="#PWR025"  Part="1" 
AR Path="/61B227D2/61927D24" Ref="#PWR027"  Part="1" 
AR Path="/61B23D59/61927D24" Ref="#PWR029"  Part="1" 
AR Path="/61B2B5D4/61927D24" Ref="#PWR031"  Part="1" 
AR Path="/61B2CBB8/61927D24" Ref="#PWR033"  Part="1" 
AR Path="/61B2E4E5/61927D24" Ref="#PWR?"  Part="1" 
AR Path="/61C30A7A/61927D24" Ref="#PWR091"  Part="1" 
F 0 "#PWR093" H 6050 3250 50  0001 C CNN
F 1 "+3.3V" H 6065 3573 50  0000 C CNN
F 2 "" H 6050 3400 50  0001 C CNN
F 3 "" H 6050 3400 50  0001 C CNN
	1    6050 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 3800 5200 3800
Text HLabel 7050 3900 2    39   Output ~ 0
OUT
$Comp
L power:GND #PWR022
U 1 1 61D69EFF
P 6050 4350
AR Path="/618D845A/61D69EFF" Ref="#PWR022"  Part="1" 
AR Path="/61B1E4D3/61D69EFF" Ref="#PWR024"  Part="1" 
AR Path="/61B211D6/61D69EFF" Ref="#PWR026"  Part="1" 
AR Path="/61B227D2/61D69EFF" Ref="#PWR028"  Part="1" 
AR Path="/61B23D59/61D69EFF" Ref="#PWR030"  Part="1" 
AR Path="/61B2B5D4/61D69EFF" Ref="#PWR032"  Part="1" 
AR Path="/61B2CBB8/61D69EFF" Ref="#PWR034"  Part="1" 
AR Path="/61C30A7A/61D69EFF" Ref="#PWR092"  Part="1" 
AR Path="/61B1FB44/61D69EFF" Ref="#PWR094"  Part="1" 
F 0 "#PWR094" H 6050 4100 50  0001 C CNN
F 1 "GND" H 6055 4177 50  0000 C CNN
F 2 "" H 6050 4350 50  0001 C CNN
F 3 "" H 6050 4350 50  0001 C CNN
	1    6050 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 4250 5750 4250
Wire Wire Line
	5750 4250 5750 4000
Connection ~ 6050 4250
Wire Wire Line
	6050 4250 6050 4200
Wire Wire Line
	6050 4250 6050 4350
$Comp
L Isolatorr:H11L1 U3
U 1 1 61D9AB7B
P 6050 3900
AR Path="/618D845A/61D9AB7B" Ref="U3"  Part="1" 
AR Path="/61B1E4D3/61D9AB7B" Ref="U4"  Part="1" 
AR Path="/61B211D6/61D9AB7B" Ref="U5"  Part="1" 
AR Path="/61B227D2/61D9AB7B" Ref="U6"  Part="1" 
AR Path="/61B23D59/61D9AB7B" Ref="U7"  Part="1" 
AR Path="/61B2B5D4/61D9AB7B" Ref="U8"  Part="1" 
AR Path="/61B2CBB8/61D9AB7B" Ref="U9"  Part="1" 
AR Path="/61C30A7A/61D9AB7B" Ref="U19"  Part="1" 
AR Path="/61B1FB44/61D9AB7B" Ref="U20"  Part="1" 
F 0 "U20" H 6394 3946 50  0000 L CNN
F 1 "H11L1" H 6394 3855 50  0000 L CNN
F 2 "Package_DIP:SMDIP-6_W7.62mm" H 5960 3900 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/H11L3M-D.PDF" H 5960 3900 50  0001 C CNN
F 4 "H11L3SM" H 6050 3900 50  0001 C CNN "PartNumber"
F 5 "https://pl.farnell.com/on-semiconductor/h11l3sm/opto-cplr-schmitt-trigger-7-5kv/dp/2322532?st=H11L3SM" H 6050 3900 50  0001 C CNN "Link"
	1    6050 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 3800 5500 3800
Wire Wire Line
	7050 3900 6750 3900
$Comp
L Device:R R62
U 1 1 6250A670
P 6750 3700
AR Path="/618D845A/6250A670" Ref="R62"  Part="1" 
AR Path="/61B1E4D3/6250A670" Ref="R63"  Part="1" 
AR Path="/61B211D6/6250A670" Ref="R64"  Part="1" 
AR Path="/61B227D2/6250A670" Ref="R65"  Part="1" 
AR Path="/61B23D59/6250A670" Ref="R66"  Part="1" 
AR Path="/61B2B5D4/6250A670" Ref="R67"  Part="1" 
AR Path="/61B2CBB8/6250A670" Ref="R68"  Part="1" 
AR Path="/61C30A7A/6250A670" Ref="R69"  Part="1" 
AR Path="/61B1FB44/6250A670" Ref="R70"  Part="1" 
F 0 "R70" H 6820 3746 50  0000 L CNN
F 1 "1k" H 6820 3655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6680 3700 50  0001 C CNN
F 3 "~" H 6750 3700 50  0001 C CNN
	1    6750 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 3400 6050 3550
Wire Wire Line
	6750 3550 6050 3550
Connection ~ 6050 3550
Wire Wire Line
	6050 3550 6050 3600
Wire Wire Line
	6750 3850 6750 3900
Connection ~ 6750 3900
Wire Wire Line
	6750 3900 6350 3900
$EndSCHEMATC
