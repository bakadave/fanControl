EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
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
L Connector:Jack-DC J?
U 1 1 618857A2
P 2550 2250
AR Path="/618857A2" Ref="J?"  Part="1" 
AR Path="/6187F069/618857A2" Ref="J?"  Part="1" 
AR Path="/6187F6D8/618857A2" Ref="J?"  Part="1" 
F 0 "J?" H 2321 2208 50  0000 R CNN
F 1 "Jack-DC" H 2321 2299 50  0000 R CNN
F 2 "fanControl:BarrelJack_Wuerth_6941xx301002" H 2600 2210 50  0001 C CNN
F 3 "~" H 2600 2210 50  0001 C CNN
	1    2550 2250
	1    0    0    1   
$EndComp
Text HLabel 3050 2150 2    50   Input ~ 0
Vin
Text HLabel 3050 2350 2    50   Input ~ 0
GND
Wire Wire Line
	2850 2150 3050 2150
Wire Wire Line
	2850 2350 3050 2350
$EndSCHEMATC
