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
L Regulator_Switching:LM2575BT-ADJ U?
U 1 1 61884134
P 6000 3350
AR Path="/61884134" Ref="U?"  Part="1" 
AR Path="/6187F069/61884134" Ref="U?"  Part="1" 
F 0 "U?" H 6000 3717 50  0000 C CNN
F 1 "LM2575BT-ADJ" H 6000 3626 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-5_Vertical" H 6000 3100 50  0001 L CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/lm2575.pdf" H 6000 3350 50  0001 C CNN
	1    6000 3350
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BC547 Q?
U 1 1 6188413A
P 4800 3750
AR Path="/6188413A" Ref="Q?"  Part="1" 
AR Path="/6187F069/6188413A" Ref="Q?"  Part="1" 
F 0 "Q?" H 4991 3796 50  0000 L CNN
F 1 "BC547" H 4991 3705 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 5000 3675 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC547.pdf" H 4800 3750 50  0001 L CNN
	1    4800 3750
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N5819 D?
U 1 1 61884140
P 6700 3600
AR Path="/61884140" Ref="D?"  Part="1" 
AR Path="/6187F069/61884140" Ref="D?"  Part="1" 
F 0 "D?" H 6700 3816 50  0000 C CNN
F 1 "1N5819" H 6700 3725 50  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 6700 3425 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88525/1n5817.pdf" H 6700 3600 50  0001 C CNN
	1    6700 3600
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 3400 4200 3350
$Comp
L power:GND #PWR?
U 1 1 6188414D
P 3600 3500
AR Path="/6188414D" Ref="#PWR?"  Part="1" 
AR Path="/6187F069/6188414D" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3600 3250 50  0001 C CNN
F 1 "GND" H 3605 3327 50  0000 C CNN
F 2 "" H 3600 3500 50  0001 C CNN
F 3 "" H 3600 3500 50  0001 C CNN
	1    3600 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61884153
P 4900 4000
AR Path="/61884153" Ref="#PWR?"  Part="1" 
AR Path="/6187F069/61884153" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4900 3750 50  0001 C CNN
F 1 "GND" H 4905 3827 50  0000 C CNN
F 2 "" H 4900 4000 50  0001 C CNN
F 3 "" H 4900 4000 50  0001 C CNN
	1    4900 4000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61884159
P 6000 3700
AR Path="/61884159" Ref="#PWR?"  Part="1" 
AR Path="/6187F069/61884159" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6000 3450 50  0001 C CNN
F 1 "GND" H 6005 3527 50  0000 C CNN
F 2 "" H 6000 3700 50  0001 C CNN
F 3 "" H 6000 3700 50  0001 C CNN
	1    6000 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 3650 6000 3700
$Comp
L Device:R R?
U 1 1 61884160
P 4200 3550
AR Path="/61884160" Ref="R?"  Part="1" 
AR Path="/6187F069/61884160" Ref="R?"  Part="1" 
F 0 "R?" H 4270 3596 50  0000 L CNN
F 1 "10k" H 4270 3505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4130 3550 50  0001 C CNN
F 3 "~" H 4200 3550 50  0001 C CNN
	1    4200 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 3050 3600 3050
$Comp
L Device:CP C?
U 1 1 61884167
P 3600 3200
AR Path="/61884167" Ref="C?"  Part="1" 
AR Path="/6187F069/61884167" Ref="C?"  Part="1" 
F 0 "C?" H 3718 3246 50  0000 L CNN
F 1 "47u" H 3718 3155 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H11.0mm_P2.50mm" H 3638 3050 50  0001 C CNN
F 3 "~" H 3600 3200 50  0001 C CNN
	1    3600 3200
	1    0    0    -1  
$EndComp
Connection ~ 3600 3050
Wire Wire Line
	3600 3500 3600 3350
$Comp
L Device:R R?
U 1 1 61884173
P 6500 2700
AR Path="/61884173" Ref="R?"  Part="1" 
AR Path="/6187F069/61884173" Ref="R?"  Part="1" 
F 0 "R?" V 6293 2700 50  0000 C CNN
F 1 "1k1" V 6384 2700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6430 2700 50  0001 C CNN
F 3 "~" H 6500 2700 50  0001 C CNN
	1    6500 2700
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 61884179
P 6950 2700
AR Path="/61884179" Ref="R?"  Part="1" 
AR Path="/6187F069/61884179" Ref="R?"  Part="1" 
F 0 "R?" V 6743 2700 50  0000 C CNN
F 1 "3k3" V 6834 2700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6880 2700 50  0001 C CNN
F 3 "~" H 6950 2700 50  0001 C CNN
	1    6950 2700
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 6188417F
P 4900 3350
AR Path="/6188417F" Ref="R?"  Part="1" 
AR Path="/6187F069/6188417F" Ref="R?"  Part="1" 
F 0 "R?" H 4970 3396 50  0000 L CNN
F 1 "47k" H 4970 3305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4830 3350 50  0001 C CNN
F 3 "~" H 4900 3350 50  0001 C CNN
	1    4900 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 3700 4200 3750
Wire Wire Line
	4600 3750 4200 3750
Wire Wire Line
	4900 4000 4900 3950
Wire Wire Line
	4900 3050 4900 3200
Wire Wire Line
	4900 3500 4900 3550
Wire Wire Line
	5350 3450 5350 3550
Wire Wire Line
	5350 3550 4900 3550
Connection ~ 4900 3550
Wire Wire Line
	5350 3250 5350 3050
Wire Wire Line
	5350 3050 4900 3050
Connection ~ 4900 3050
$Comp
L pspice:INDUCTOR L?
U 1 1 61884190
P 7200 3450
AR Path="/61884190" Ref="L?"  Part="1" 
AR Path="/6187F069/61884190" Ref="L?"  Part="1" 
F 0 "L?" H 7200 3665 50  0000 C CNN
F 1 "470u" H 7200 3574 50  0000 C CNN
F 2 "Inductor_THT:L_Radial_D10.5mm_P5.00mm_Abacron_AISR-01" H 7200 3450 50  0001 C CNN
F 3 "~" H 7200 3450 50  0001 C CNN
	1    7200 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 3450 6700 3450
Connection ~ 6700 3450
Wire Wire Line
	6700 3450 6950 3450
$Comp
L Device:CP C?
U 1 1 61884199
P 7750 3600
AR Path="/61884199" Ref="C?"  Part="1" 
AR Path="/6187F069/61884199" Ref="C?"  Part="1" 
F 0 "C?" H 7868 3646 50  0000 L CNN
F 1 "470u" H 7868 3555 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D8.0mm_H11.5mm_P3.50mm" H 7788 3450 50  0001 C CNN
F 3 "~" H 7750 3600 50  0001 C CNN
	1    7750 3600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6188419F
P 7250 3950
AR Path="/6188419F" Ref="#PWR?"  Part="1" 
AR Path="/6187F069/6188419F" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 7250 3700 50  0001 C CNN
F 1 "GND" H 7255 3777 50  0000 C CNN
F 2 "" H 7250 3950 50  0001 C CNN
F 3 "" H 7250 3950 50  0001 C CNN
	1    7250 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 3750 6700 3800
Wire Wire Line
	6700 3800 7250 3800
Wire Wire Line
	7750 3800 7750 3750
Wire Wire Line
	7250 3950 7250 3800
Connection ~ 7250 3800
Wire Wire Line
	7250 3800 7750 3800
Wire Wire Line
	7450 3450 7750 3450
$Comp
L power:GND #PWR?
U 1 1 618841AC
P 6250 2750
AR Path="/618841AC" Ref="#PWR?"  Part="1" 
AR Path="/6187F069/618841AC" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6250 2500 50  0001 C CNN
F 1 "GND" H 6255 2577 50  0000 C CNN
F 2 "" H 6250 2750 50  0001 C CNN
F 3 "" H 6250 2750 50  0001 C CNN
	1    6250 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 2700 6350 2700
Wire Wire Line
	6650 2700 6700 2700
Wire Wire Line
	7750 3450 7750 2700
Wire Wire Line
	7750 2700 7550 2700
Connection ~ 7750 3450
Wire Wire Line
	6500 3250 6700 3250
Wire Wire Line
	6700 3250 6700 2700
Connection ~ 6700 2700
Wire Wire Line
	6700 2700 6800 2700
$Comp
L Device:R R?
U 1 1 618841BB
P 7400 2700
AR Path="/618841BB" Ref="R?"  Part="1" 
AR Path="/6187F069/618841BB" Ref="R?"  Part="1" 
F 0 "R?" V 7193 2700 50  0000 C CNN
F 1 "1k" V 7284 2700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7330 2700 50  0001 C CNN
F 3 "~" H 7400 2700 50  0001 C CNN
	1    7400 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	7100 2700 7150 2700
Connection ~ 7150 2700
Wire Wire Line
	7150 2700 7250 2700
Wire Wire Line
	7750 2150 7750 2700
Connection ~ 7750 2700
$Comp
L Device:R_POT_TRIM RV?
U 1 1 618841C6
P 7450 2150
AR Path="/618841C6" Ref="RV?"  Part="1" 
AR Path="/6187F069/618841C6" Ref="RV?"  Part="1" 
F 0 "RV?" V 7335 2150 50  0000 C CNN
F 1 "R_POT_TRIM" V 7244 2150 50  0000 C CNN
F 2 "fanControl:Potentiometer_ACP_CA9-V10_Vertical" H 7450 2150 50  0001 C CNN
F 3 "~" H 7450 2150 50  0001 C CNN
	1    7450 2150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7750 2150 7600 2150
Wire Wire Line
	7300 2150 7150 2150
Wire Wire Line
	6250 2750 6250 2700
$Comp
L Diode:1N5819 D?
U 1 1 618841CF
P 8350 3450
AR Path="/618841CF" Ref="D?"  Part="1" 
AR Path="/6187F069/618841CF" Ref="D?"  Part="1" 
F 0 "D?" H 8350 3234 50  0000 C CNN
F 1 "1N5819" H 8350 3325 50  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 8350 3275 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88525/1n5817.pdf" H 8350 3450 50  0001 C CNN
	1    8350 3450
	-1   0    0    1   
$EndComp
Wire Wire Line
	7750 3450 8200 3450
$Comp
L Diode:ZYxxx D?
U 1 1 618841D6
P 4200 3200
AR Path="/618841D6" Ref="D?"  Part="1" 
AR Path="/6187F069/618841D6" Ref="D?"  Part="1" 
F 0 "D?" V 4154 3279 50  0000 L CNN
F 1 "ZY6.8" V 4245 3279 50  0000 L CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 4200 3025 50  0001 C CNN
F 3 "https://diotec.com/tl_files/diotec/files/pdf/datasheets/zy1" H 4200 3200 50  0001 C CNN
	1    4200 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	3600 3050 4200 3050
Connection ~ 4200 3050
Wire Wire Line
	4200 3050 4900 3050
$Comp
L Device:R R?
U 1 1 618841E0
P 4200 4000
AR Path="/618841E0" Ref="R?"  Part="1" 
AR Path="/6187F069/618841E0" Ref="R?"  Part="1" 
F 0 "R?" H 4300 4050 50  0000 L CNN
F 1 "10k" H 4270 3955 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4130 4000 50  0001 C CNN
F 3 "~" H 4200 4000 50  0001 C CNN
	1    4200 4000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 618841E6
P 4200 4250
AR Path="/618841E6" Ref="#PWR?"  Part="1" 
AR Path="/6187F069/618841E6" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4200 4000 50  0001 C CNN
F 1 "GND" H 4205 4077 50  0000 C CNN
F 2 "" H 4200 4250 50  0001 C CNN
F 3 "" H 4200 4250 50  0001 C CNN
	1    4200 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4250 4200 4150
Wire Wire Line
	4200 3850 4200 3750
Connection ~ 4200 3750
Wire Wire Line
	7450 2000 7450 1900
Wire Wire Line
	7450 1900 7150 1900
Wire Wire Line
	7150 1900 7150 2150
Connection ~ 7150 2150
Wire Wire Line
	7150 2150 7150 2700
Wire Wire Line
	5350 3250 5500 3250
Wire Wire Line
	5350 3450 5500 3450
Text HLabel 3100 3050 0    50   Input ~ 0
Vin
Text HLabel 8500 3450 2    50   Input ~ 0
5V
$EndSCHEMATC
