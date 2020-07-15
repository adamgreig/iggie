EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 7 8
Title "USB JTAG/UART"
Date "2020-07-15"
Rev "1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L agg-kicad:FT2232H IC?
U 1 1 5F3992D3
P 6750 3650
F 0 "IC?" H 5850 5850 50  0000 L CNN
F 1 "FT2232H" H 5850 1450 50  0000 L CNN
F 2 "agg:LQFP-64" H 5850 1350 50  0001 L CNN
F 3 "https://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT2232H.pdf" H 5850 1250 50  0001 L CNN
F 4 "1697461" H 5850 1150 50  0001 L CNN "Farnell"
	1    6750 3650
	1    0    0    -1  
$EndComp
$Comp
L agg-kicad:USB-C P?
U 1 1 5F39AC84
P 1900 4500
F 0 "P?" H 1450 5200 50  0000 L CNN
F 1 "USB-C" H 1450 3800 50  0000 L CNN
F 2 "agg:MOLEX-USB-C-105450-0101" H 1450 3700 50  0001 L CNN
F 3 "http://www.te.com/usa-en/product-2129691-1.html" H 1450 3600 50  0001 L CNN
F 4 "2524076" H 1450 3500 50  0001 L CNN "Farnell"
	1    1900 4500
	1    0    0    -1  
$EndComp
$Comp
L agg-kicad:93LC46B-OT IC?
U 1 1 5F39C51D
P 3850 6300
F 0 "IC?" H 3650 6600 50  0000 L CNN
F 1 "93LC46B-OT" H 3650 6000 50  0000 L CNN
F 2 "agg:SOT-23-6" H 3650 5900 50  0001 L CNN
F 3 "https://ww1.microchip.com/downloads/en/DeviceDoc/20001749K.pdf" H 3650 5800 50  0001 L CNN
F 4 "1556164" H 3650 5700 50  0001 L CNN "Farnell"
	1    3850 6300
	1    0    0    -1  
$EndComp
$Comp
L agg-kicad:R R?
U 1 1 5F39DADC
P 4250 5850
F 0 "R?" H 4300 5900 50  0000 C CNN
F 1 "10k" H 4300 5800 50  0000 C CNN
F 2 "agg:0402" H 4250 5850 50  0001 C CNN
F 3 "" H 4250 5850 50  0001 C CNN
	1    4250 5850
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:R R?
U 1 1 5F39F5C4
P 4450 5850
F 0 "R?" H 4500 5900 50  0000 C CNN
F 1 "10k" H 4500 5800 50  0000 C CNN
F 2 "agg:0402" H 4450 5850 50  0001 C CNN
F 3 "" H 4450 5850 50  0001 C CNN
	1    4450 5850
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:R R?
U 1 1 5F39FD40
P 4900 6300
F 0 "R?" H 4950 6350 50  0000 C CNN
F 1 "2k2" H 4950 6250 50  0000 C CNN
F 2 "agg:0402" H 4900 6300 50  0001 C CNN
F 3 "" H 4900 6300 50  0001 C CNN
	1    4900 6300
	-1   0    0    1   
$EndComp
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F3A0399
P 3500 6050
F 0 "#PWR?" H 3500 6160 50  0001 L CNN
F 1 "3v3" H 3500 6140 50  0000 C CNN
F 2 "" H 3500 6050 50  0001 C CNN
F 3 "" H 3500 6050 50  0001 C CNN
	1    3500 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6050 3500 6100
Wire Wire Line
	3500 6100 3550 6100
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F3A0B39
P 3500 6350
F 0 "#PWR?" H 3370 6390 50  0001 L CNN
F 1 "GND" H 3500 6250 50  0000 C CNN
F 2 "" H 3500 6350 50  0001 C CNN
F 3 "" H 3500 6350 50  0001 C CNN
	1    3500 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6350 3500 6300
Wire Wire Line
	3500 6300 3550 6300
Wire Wire Line
	4150 6100 4250 6100
Wire Wire Line
	4250 6100 4250 5950
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F3A11D2
P 4250 5800
F 0 "#PWR?" H 4250 5910 50  0001 L CNN
F 1 "3v3" H 4250 5890 50  0000 C CNN
F 2 "" H 4250 5800 50  0001 C CNN
F 3 "" H 4250 5800 50  0001 C CNN
	1    4250 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 5800 4250 5850
Wire Wire Line
	4250 6100 5400 6100
Wire Wire Line
	5400 6100 5400 5450
Wire Wire Line
	5400 5450 5750 5450
Connection ~ 4250 6100
Wire Wire Line
	4150 6200 4450 6200
Wire Wire Line
	4450 6200 4450 5950
Wire Wire Line
	4450 6200 5500 6200
Wire Wire Line
	5500 6200 5500 5550
Wire Wire Line
	5500 5550 5750 5550
Connection ~ 4450 6200
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F3A2027
P 4450 5800
F 0 "#PWR?" H 4450 5910 50  0001 L CNN
F 1 "3v3" H 4450 5890 50  0000 C CNN
F 2 "" H 4450 5800 50  0001 C CNN
F 3 "" H 4450 5800 50  0001 C CNN
	1    4450 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 5800 4450 5850
Wire Wire Line
	5600 5650 5750 5650
$Comp
L agg-kicad:R R?
U 1 1 5F3A46A6
P 4650 5850
F 0 "R?" H 4700 5900 50  0000 C CNN
F 1 "10k" H 4700 5800 50  0000 C CNN
F 2 "agg:0402" H 4650 5850 50  0001 C CNN
F 3 "" H 4650 5850 50  0001 C CNN
	1    4650 5850
	0    1    1    0   
$EndComp
Wire Wire Line
	4150 6300 4650 6300
Wire Wire Line
	4650 5950 4650 6300
Connection ~ 4650 6300
Wire Wire Line
	4650 6300 4800 6300
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F3A631F
P 4650 5800
F 0 "#PWR?" H 4650 5910 50  0001 L CNN
F 1 "3v3" H 4650 5890 50  0000 C CNN
F 2 "" H 4650 5800 50  0001 C CNN
F 3 "" H 4650 5800 50  0001 C CNN
	1    4650 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 5800 4650 5850
Wire Wire Line
	4900 6300 5000 6300
Wire Wire Line
	5600 6300 5600 5650
Wire Wire Line
	4150 6400 5000 6400
Wire Wire Line
	5000 6400 5000 6300
Connection ~ 5000 6300
Wire Wire Line
	5000 6300 5600 6300
$Comp
L agg-kicad:C C?
U 1 1 5F3A87CA
P 3250 6150
F 0 "C?" H 3300 6220 50  0000 C CNN
F 1 "100n" H 3300 6080 50  0000 C CNN
F 2 "agg:0402" H 3250 6150 50  0001 C CNN
F 3 "" H 3250 6150 50  0001 C CNN
	1    3250 6150
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F3B2403
P 1250 5150
F 0 "#PWR?" H 1120 5190 50  0001 L CNN
F 1 "GND" H 1250 5050 50  0000 C CNN
F 2 "" H 1250 5150 50  0001 C CNN
F 3 "" H 1250 5150 50  0001 C CNN
	1    1250 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 5150 1250 5100
Wire Wire Line
	1250 5000 1300 5000
Wire Wire Line
	1300 5100 1250 5100
Connection ~ 1250 5100
Wire Wire Line
	1250 5100 1250 5000
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F3B2FB2
P 2550 5150
F 0 "#PWR?" H 2420 5190 50  0001 L CNN
F 1 "GND" H 2550 5050 50  0000 C CNN
F 2 "" H 2550 5150 50  0001 C CNN
F 3 "" H 2550 5150 50  0001 C CNN
	1    2550 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 5000 2550 5000
Wire Wire Line
	2550 5000 2550 5150
NoConn ~ 2500 4900
NoConn ~ 2500 4800
NoConn ~ 1300 4800
NoConn ~ 1300 4900
NoConn ~ 1300 4000
NoConn ~ 1300 4100
NoConn ~ 2500 4000
NoConn ~ 2500 4100
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F3B64CD
P 1250 3900
F 0 "#PWR?" H 1120 3940 50  0001 L CNN
F 1 "GND" H 1250 3800 50  0000 C CNN
F 2 "" H 1250 3900 50  0001 C CNN
F 3 "" H 1250 3900 50  0001 C CNN
	1    1250 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	1250 3900 1300 3900
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F3B6F66
P 2550 3900
F 0 "#PWR?" H 2420 3940 50  0001 L CNN
F 1 "GND" H 2550 3800 50  0000 C CNN
F 2 "" H 2550 3900 50  0001 C CNN
F 3 "" H 2550 3900 50  0001 C CNN
	1    2550 3900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2500 3900 2550 3900
$Comp
L agg-kicad:R R?
U 1 1 5F3B8474
P 1050 4300
F 0 "R?" H 1100 4350 50  0000 C CNN
F 1 "5k1" H 1100 4250 50  0000 C CNN
F 2 "agg:0402" H 1050 4300 50  0001 C CNN
F 3 "" H 1050 4300 50  0001 C CNN
	1    1050 4300
	-1   0    0    1   
$EndComp
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F3B895E
P 900 4300
F 0 "#PWR?" H 770 4340 50  0001 L CNN
F 1 "GND" H 900 4200 50  0000 C CNN
F 2 "" H 900 4300 50  0001 C CNN
F 3 "" H 900 4300 50  0001 C CNN
	1    900  4300
	0    1    1    0   
$EndComp
Wire Wire Line
	900  4300 950  4300
Wire Wire Line
	1050 4300 1300 4300
$Comp
L agg-kicad:R R?
U 1 1 5F3B98BD
P 2750 4300
F 0 "R?" H 2800 4350 50  0000 C CNN
F 1 "5k1" H 2800 4250 50  0000 C CNN
F 2 "agg:0402" H 2750 4300 50  0001 C CNN
F 3 "" H 2750 4300 50  0001 C CNN
	1    2750 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 4300 2500 4300
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F3BA6F8
P 2900 4300
F 0 "#PWR?" H 2770 4340 50  0001 L CNN
F 1 "GND" H 2900 4200 50  0000 C CNN
F 2 "" H 2900 4300 50  0001 C CNN
F 3 "" H 2900 4300 50  0001 C CNN
	1    2900 4300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2900 4300 2850 4300
Wire Wire Line
	1300 4200 1200 4200
Text Label 1200 4200 2    50   ~ 0
VBUS
Wire Wire Line
	2500 4200 2600 4200
Text Label 2600 4200 0    50   ~ 0
VBUS
Text Label 1200 4700 2    50   ~ 0
VBUS
Wire Wire Line
	1200 4700 1300 4700
Text Label 2600 4700 0    50   ~ 0
VBUS
Wire Wire Line
	2600 4700 2500 4700
NoConn ~ 1300 4600
NoConn ~ 2500 4600
Wire Wire Line
	1300 4400 1200 4400
Text Label 1200 4400 2    50   ~ 0
D+
Text Label 1200 4500 2    50   ~ 0
D-
Wire Wire Line
	1200 4500 1300 4500
Wire Wire Line
	2500 4400 2600 4400
Text Label 2600 4400 0    50   ~ 0
D+
Text Label 2600 4500 0    50   ~ 0
D-
Wire Wire Line
	2600 4500 2500 4500
$Comp
L agg-kicad:USBLC6-2P6 D?
U 1 1 5F3CCF90
P 4200 4950
F 0 "D?" H 3900 5150 50  0000 L CNN
F 1 "USBLC6-2SC6" H 3900 4750 50  0000 L CNN
F 2 "ael:SOT-23-6" H 3900 4650 50  0001 L CNN
F 3 "http://www.st.com/resource/en/datasheet/usblc6-2.pdf" H 3900 4550 50  0001 L CNN
F 4 "1269406" H 3900 4450 50  0001 L CNN "Farnell"
	1    4200 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 4850 3700 4850
Wire Wire Line
	3700 5050 3800 5050
Text Label 3700 5050 2    50   ~ 0
D+
Text Label 3700 4850 2    50   ~ 0
D-
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F3EA948
P 3600 4950
F 0 "#PWR?" H 3470 4990 50  0001 L CNN
F 1 "GND" H 3600 4850 50  0000 C CNN
F 2 "" H 3600 4950 50  0001 C CNN
F 3 "" H 3600 4950 50  0001 C CNN
	1    3600 4950
	0    1    1    0   
$EndComp
Wire Wire Line
	3600 4950 3800 4950
Text Label 4700 4950 0    50   ~ 0
VBUS
Wire Wire Line
	4700 4950 4600 4950
Wire Wire Line
	4600 4850 5750 4850
Wire Wire Line
	4600 5050 4950 5050
Wire Wire Line
	4950 5050 4950 4950
Wire Wire Line
	4950 4950 5750 4950
$Comp
L agg-kicad:R R?
U 1 1 5F41038B
P 5550 4650
F 0 "R?" H 5600 4700 50  0000 C CNN
F 1 "12k" H 5600 4600 50  0000 C CNN
F 2 "agg:0402" H 5550 4650 50  0001 C CNN
F 3 "" H 5550 4650 50  0001 C CNN
	1    5550 4650
	1    0    0    -1  
$EndComp
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F41162F
P 5500 4650
F 0 "#PWR?" H 5370 4690 50  0001 L CNN
F 1 "GND" H 5500 4550 50  0000 C CNN
F 2 "" H 5500 4650 50  0001 C CNN
F 3 "" H 5500 4650 50  0001 C CNN
	1    5500 4650
	0    1    1    0   
$EndComp
Wire Wire Line
	5500 4650 5550 4650
Wire Wire Line
	5650 4650 5750 4650
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F415EA5
P 5650 5150
F 0 "#PWR?" H 5520 5190 50  0001 L CNN
F 1 "GND" H 5650 5050 50  0000 C CNN
F 2 "" H 5650 5150 50  0001 C CNN
F 3 "" H 5650 5150 50  0001 C CNN
	1    5650 5150
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 5150 5750 5150
$Comp
L agg-kicad:R R?
U 1 1 5F418BAC
P 5450 5250
F 0 "R?" H 5500 5300 50  0000 C CNN
F 1 "10k" H 5500 5200 50  0000 C CNN
F 2 "agg:0402" H 5450 5250 50  0001 C CNN
F 3 "" H 5450 5250 50  0001 C CNN
	1    5450 5250
	-1   0    0    1   
$EndComp
Wire Wire Line
	5450 5250 5750 5250
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F41B4D3
P 5300 5200
F 0 "#PWR?" H 5300 5310 50  0001 L CNN
F 1 "3v3" H 5300 5290 50  0000 C CNN
F 2 "" H 5300 5200 50  0001 C CNN
F 3 "" H 5300 5200 50  0001 C CNN
	1    5300 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 5200 5300 5250
Wire Wire Line
	5300 5250 5350 5250
NoConn ~ 5750 4350
NoConn ~ 5750 4450
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F4235DB
P 5650 3900
F 0 "#PWR?" H 5520 3940 50  0001 L CNN
F 1 "GND" H 5650 3800 50  0000 C CNN
F 2 "" H 5650 3900 50  0001 C CNN
F 3 "" H 5650 3900 50  0001 C CNN
	1    5650 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 3900 5650 3850
Connection ~ 5650 3150
Wire Wire Line
	5650 3150 5650 3050
Connection ~ 5650 3250
Wire Wire Line
	5650 3250 5650 3150
Connection ~ 5650 3350
Wire Wire Line
	5650 3350 5650 3250
Connection ~ 5650 3450
Wire Wire Line
	5650 3450 5650 3350
Connection ~ 5650 3550
Wire Wire Line
	5650 3550 5650 3450
Connection ~ 5650 3650
Wire Wire Line
	5650 3650 5650 3550
Connection ~ 5650 3750
Wire Wire Line
	5650 3750 5650 3650
Connection ~ 5650 3850
Wire Wire Line
	5650 3850 5650 3750
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F43E030
P 5650 1450
F 0 "#PWR?" H 5650 1560 50  0001 L CNN
F 1 "3v3" H 5650 1540 50  0000 C CNN
F 2 "" H 5650 1450 50  0001 C CNN
F 3 "" H 5650 1450 50  0001 C CNN
	1    5650 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 1450 5650 1550
Wire Wire Line
	5650 1850 5750 1850
Wire Wire Line
	5750 1550 5650 1550
Connection ~ 5650 1550
Wire Wire Line
	5650 1550 5650 1650
Wire Wire Line
	5650 1650 5750 1650
Connection ~ 5650 1650
Wire Wire Line
	5650 1650 5650 1750
Wire Wire Line
	5750 1750 5650 1750
Connection ~ 5650 1750
Wire Wire Line
	5650 1750 5650 1850
Wire Wire Line
	5650 3050 5750 3050
Wire Wire Line
	5650 3150 5750 3150
Wire Wire Line
	5650 3250 5750 3250
Wire Wire Line
	5650 3350 5750 3350
Wire Wire Line
	5650 3450 5750 3450
Wire Wire Line
	5650 3550 5750 3550
Wire Wire Line
	5650 3650 5750 3650
Wire Wire Line
	5650 3750 5750 3750
Wire Wire Line
	5650 3850 5750 3850
$Comp
L agg-kicad:L L?
U 1 1 5F463767
P 5400 1750
F 0 "L?" H 5450 1800 50  0000 C CNN
F 1 "FB" H 5450 1700 50  0000 C CNN
F 2 "" H 5400 1750 50  0001 C CNN
F 3 "" H 5400 1750 50  0001 C CNN
	1    5400 1750
	0    -1   -1   0   
$EndComp
$Comp
L agg-kicad:L L?
U 1 1 5F466D11
P 5200 1750
F 0 "L?" H 5250 1800 50  0000 C CNN
F 1 "FB" H 5250 1700 50  0000 C CNN
F 2 "" H 5200 1750 50  0001 C CNN
F 3 "" H 5200 1750 50  0001 C CNN
	1    5200 1750
	0    -1   -1   0   
$EndComp
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F4689A5
P 5400 1450
F 0 "#PWR?" H 5400 1560 50  0001 L CNN
F 1 "3v3" H 5400 1540 50  0000 C CNN
F 2 "" H 5400 1450 50  0001 C CNN
F 3 "" H 5400 1450 50  0001 C CNN
	1    5400 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 1450 5400 1650
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F46B350
P 5200 1450
F 0 "#PWR?" H 5200 1560 50  0001 L CNN
F 1 "3v3" H 5200 1540 50  0000 C CNN
F 2 "" H 5200 1450 50  0001 C CNN
F 3 "" H 5200 1450 50  0001 C CNN
	1    5200 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1650 5200 1450
Wire Wire Line
	5400 1750 5400 2050
Wire Wire Line
	5400 2050 5750 2050
Wire Wire Line
	5200 1750 5200 2150
Wire Wire Line
	5200 2150 5750 2150
Wire Wire Line
	5750 2850 5650 2850
Wire Wire Line
	5650 2850 5650 2750
Wire Wire Line
	5650 2450 5750 2450
Wire Wire Line
	5750 2650 5650 2650
Connection ~ 5650 2650
Wire Wire Line
	5650 2650 5650 2450
Wire Wire Line
	5650 2750 5750 2750
Connection ~ 5650 2750
Wire Wire Line
	5650 2750 5650 2650
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F47DC16
P 5200 2300
F 0 "#PWR?" H 5200 2410 50  0001 L CNN
F 1 "3v3" H 5200 2390 50  0000 C CNN
F 2 "" H 5200 2300 50  0001 C CNN
F 3 "" H 5200 2300 50  0001 C CNN
	1    5200 2300
	1    0    0    -1  
$EndComp
Text Label 8150 4750 0    50   ~ 0
VBUS
$Comp
L agg-kicad:R R?
U 1 1 5F48F3A0
P 8100 4800
F 0 "R?" H 8150 4850 50  0000 C CNN
F 1 "4k7" H 8150 4750 50  0000 C CNN
F 2 "" H 8100 4800 50  0001 C CNN
F 3 "" H 8100 4800 50  0001 C CNN
	1    8100 4800
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:R R?
U 1 1 5F48F6B6
P 8100 5000
F 0 "R?" H 8150 5050 50  0000 C CNN
F 1 "10k" H 8150 4950 50  0000 C CNN
F 2 "agg:0402" H 8100 5000 50  0001 C CNN
F 3 "" H 8100 5000 50  0001 C CNN
	1    8100 5000
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 4900 8100 4950
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F4934E4
P 8100 5150
F 0 "#PWR?" H 7970 5190 50  0001 L CNN
F 1 "GND" H 8100 5050 50  0000 C CNN
F 2 "" H 8100 5150 50  0001 C CNN
F 3 "" H 8100 5150 50  0001 C CNN
	1    8100 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 5150 8100 5100
Wire Wire Line
	8100 4800 8100 4750
Wire Wire Line
	8100 4750 8150 4750
Wire Wire Line
	7750 4950 8100 4950
Connection ~ 8100 4950
Wire Wire Line
	8100 4950 8100 5000
Wire Wire Line
	7750 1550 7850 1550
Wire Wire Line
	7750 1650 7850 1650
Wire Wire Line
	7750 1750 7850 1750
Wire Wire Line
	7750 1850 7850 1850
Text HLabel 7850 1550 2    50   Output ~ 0
TCK
Text HLabel 7850 1650 2    50   Output ~ 0
TDI
Text HLabel 7850 1750 2    50   Input ~ 0
TDO
Text HLabel 7850 1850 2    50   Output ~ 0
TMS
NoConn ~ 7750 1950
NoConn ~ 7750 2050
NoConn ~ 7750 2150
NoConn ~ 7750 2250
NoConn ~ 7750 2450
NoConn ~ 7750 2550
NoConn ~ 7750 2650
NoConn ~ 7750 2750
NoConn ~ 7750 2850
NoConn ~ 7750 2950
NoConn ~ 7750 3050
NoConn ~ 7750 3150
Wire Wire Line
	7750 3350 7850 3350
Text HLabel 7850 3350 2    50   Output ~ 0
TXD
Text HLabel 7850 3450 2    50   Input ~ 0
RXD
Wire Wire Line
	7850 3450 7750 3450
NoConn ~ 7750 3550
NoConn ~ 7750 3650
NoConn ~ 7750 3750
NoConn ~ 7750 3850
NoConn ~ 7750 3950
NoConn ~ 7750 4050
NoConn ~ 7750 4250
NoConn ~ 7750 4350
NoConn ~ 7750 4450
NoConn ~ 7750 4750
NoConn ~ 7750 4850
$Comp
L agg-kicad:C C?
U 1 1 5F51D97D
P 4300 1800
AR Path="/5F0FD3F5/5F51D97D" Ref="C?"  Part="1" 
AR Path="/5F117D14/5F51D97D" Ref="C?"  Part="1" 
F 0 "C?" H 4350 1870 50  0000 C CNN
F 1 "4µ7" H 4350 1730 50  0000 C CNN
F 2 "agg:0603" H 4300 1800 50  0001 C CNN
F 3 "" H 4300 1800 50  0001 C CNN
	1    4300 1800
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F524E02
P 3250 6050
F 0 "#PWR?" H 3250 6160 50  0001 L CNN
F 1 "3v3" H 3250 6140 50  0000 C CNN
F 2 "" H 3250 6050 50  0001 C CNN
F 3 "" H 3250 6050 50  0001 C CNN
	1    3250 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 6050 3250 6150
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F52902A
P 3250 6350
F 0 "#PWR?" H 3120 6390 50  0001 L CNN
F 1 "GND" H 3250 6250 50  0000 C CNN
F 2 "" H 3250 6350 50  0001 C CNN
F 3 "" H 3250 6350 50  0001 C CNN
	1    3250 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 6350 3250 6250
$Comp
L agg-kicad:C C?
U 1 1 5F53C079
P 4550 1800
F 0 "C?" H 4600 1870 50  0000 C CNN
F 1 "100n" H 4600 1730 50  0000 C CNN
F 2 "agg:0402" H 4550 1800 50  0001 C CNN
F 3 "" H 4550 1800 50  0001 C CNN
	1    4550 1800
	0    1    1    0   
$EndComp
Text Label 5450 2050 0    50   ~ 0
VPLL
Text Label 5450 2150 0    50   ~ 0
VPHY
$Comp
L agg-kicad:C C?
U 1 1 5F545E96
P 4300 2350
AR Path="/5F0FD3F5/5F545E96" Ref="C?"  Part="1" 
AR Path="/5F117D14/5F545E96" Ref="C?"  Part="1" 
F 0 "C?" H 4350 2420 50  0000 C CNN
F 1 "4µ7" H 4350 2280 50  0000 C CNN
F 2 "agg:0603" H 4300 2350 50  0001 C CNN
F 3 "" H 4300 2350 50  0001 C CNN
	1    4300 2350
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:C C?
U 1 1 5F545E9C
P 4550 2350
F 0 "C?" H 4600 2420 50  0000 C CNN
F 1 "100n" H 4600 2280 50  0000 C CNN
F 2 "agg:0402" H 4550 2350 50  0001 C CNN
F 3 "" H 4550 2350 50  0001 C CNN
	1    4550 2350
	0    1    1    0   
$EndComp
Wire Wire Line
	4550 1700 4550 1800
Wire Wire Line
	4300 1800 4300 1700
Wire Wire Line
	4300 1700 4550 1700
Text Label 4650 1700 0    50   ~ 0
VPLL
Wire Wire Line
	4650 1700 4550 1700
Connection ~ 4550 1700
Wire Wire Line
	4300 2350 4300 2250
Wire Wire Line
	4300 2250 4550 2250
Wire Wire Line
	4550 2350 4550 2250
Connection ~ 4550 2250
Wire Wire Line
	4550 2250 4650 2250
Text Label 4650 2250 0    50   ~ 0
VPHY
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F56090E
P 4300 1950
F 0 "#PWR?" H 4170 1990 50  0001 L CNN
F 1 "GND" H 4300 1850 50  0000 C CNN
F 2 "" H 4300 1950 50  0001 C CNN
F 3 "" H 4300 1950 50  0001 C CNN
	1    4300 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 1950 4300 1900
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F565937
P 4550 1950
F 0 "#PWR?" H 4420 1990 50  0001 L CNN
F 1 "GND" H 4550 1850 50  0000 C CNN
F 2 "" H 4550 1950 50  0001 C CNN
F 3 "" H 4550 1950 50  0001 C CNN
	1    4550 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1950 4550 1900
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F56B54E
P 4300 2500
F 0 "#PWR?" H 4170 2540 50  0001 L CNN
F 1 "GND" H 4300 2400 50  0000 C CNN
F 2 "" H 4300 2500 50  0001 C CNN
F 3 "" H 4300 2500 50  0001 C CNN
	1    4300 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2500 4300 2450
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F56B559
P 4550 2500
F 0 "#PWR?" H 4420 2540 50  0001 L CNN
F 1 "GND" H 4550 2400 50  0000 C CNN
F 2 "" H 4550 2500 50  0001 C CNN
F 3 "" H 4550 2500 50  0001 C CNN
	1    4550 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 2500 4550 2450
Wire Wire Line
	5550 2450 5650 2450
Connection ~ 5650 2450
$Comp
L agg-kicad:C C?
U 1 1 5F59ED1D
P 5200 2500
F 0 "C?" H 5250 2570 50  0000 C CNN
F 1 "100n" H 5250 2430 50  0000 C CNN
F 2 "agg:0402" H 5200 2500 50  0001 C CNN
F 3 "" H 5200 2500 50  0001 C CNN
	1    5200 2500
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F59F32E
P 5200 2650
F 0 "#PWR?" H 5070 2690 50  0001 L CNN
F 1 "GND" H 5200 2550 50  0000 C CNN
F 2 "" H 5200 2650 50  0001 C CNN
F 3 "" H 5200 2650 50  0001 C CNN
	1    5200 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 2650 5200 2600
Wire Wire Line
	5200 2300 5200 2350
Wire Wire Line
	5750 2350 5200 2350
Connection ~ 5200 2350
Wire Wire Line
	5200 2350 5200 2500
Text Label 5550 2450 2    50   ~ 0
VCORE
$Comp
L agg-kicad:C C?
U 1 1 5F6136B3
P 4300 1250
F 0 "C?" H 4350 1320 50  0000 C CNN
F 1 "100n" H 4350 1180 50  0000 C CNN
F 2 "agg:0402" H 4300 1250 50  0001 C CNN
F 3 "" H 4300 1250 50  0001 C CNN
	1    4300 1250
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:C C?
U 1 1 5F613D51
P 4550 1250
F 0 "C?" H 4600 1320 50  0000 C CNN
F 1 "100n" H 4600 1180 50  0000 C CNN
F 2 "agg:0402" H 4550 1250 50  0001 C CNN
F 3 "" H 4550 1250 50  0001 C CNN
	1    4550 1250
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:C C?
U 1 1 5F6141B6
P 4050 1250
F 0 "C?" H 4100 1320 50  0000 C CNN
F 1 "100n" H 4100 1180 50  0000 C CNN
F 2 "agg:0402" H 4050 1250 50  0001 C CNN
F 3 "" H 4050 1250 50  0001 C CNN
	1    4050 1250
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F6146BE
P 4650 1100
F 0 "#PWR?" H 4650 1210 50  0001 L CNN
F 1 "3v3" H 4650 1190 50  0000 C CNN
F 2 "" H 4650 1100 50  0001 C CNN
F 3 "" H 4650 1100 50  0001 C CNN
	1    4650 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 1150 4300 1150
Wire Wire Line
	4550 1150 4550 1250
Wire Wire Line
	4050 1150 4050 1250
Wire Wire Line
	4300 1250 4300 1150
Connection ~ 4300 1150
Wire Wire Line
	4300 1150 4550 1150
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F6291BE
P 4050 1400
F 0 "#PWR?" H 3920 1440 50  0001 L CNN
F 1 "GND" H 4050 1300 50  0000 C CNN
F 2 "" H 4050 1400 50  0001 C CNN
F 3 "" H 4050 1400 50  0001 C CNN
	1    4050 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 1400 4050 1350
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F6331BE
P 4300 1400
F 0 "#PWR?" H 4170 1440 50  0001 L CNN
F 1 "GND" H 4300 1300 50  0000 C CNN
F 2 "" H 4300 1400 50  0001 C CNN
F 3 "" H 4300 1400 50  0001 C CNN
	1    4300 1400
	1    0    0    -1  
$EndComp
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F6335B9
P 4550 1400
F 0 "#PWR?" H 4420 1440 50  0001 L CNN
F 1 "GND" H 4550 1300 50  0000 C CNN
F 2 "" H 4550 1400 50  0001 C CNN
F 3 "" H 4550 1400 50  0001 C CNN
	1    4550 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1400 4550 1350
Wire Wire Line
	4300 1350 4300 1400
Text Label 4650 2800 0    50   ~ 0
VCORE
Wire Wire Line
	3800 3000 3800 3050
Wire Wire Line
	4050 3050 4050 3000
Wire Wire Line
	4300 3000 4300 3050
Wire Wire Line
	4550 3050 4550 3000
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F5F7A95
P 4550 3050
F 0 "#PWR?" H 4420 3090 50  0001 L CNN
F 1 "GND" H 4550 2950 50  0000 C CNN
F 2 "" H 4550 3050 50  0001 C CNN
F 3 "" H 4550 3050 50  0001 C CNN
	1    4550 3050
	1    0    0    -1  
$EndComp
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F5F7740
P 4300 3050
F 0 "#PWR?" H 4170 3090 50  0001 L CNN
F 1 "GND" H 4300 2950 50  0000 C CNN
F 2 "" H 4300 3050 50  0001 C CNN
F 3 "" H 4300 3050 50  0001 C CNN
	1    4300 3050
	1    0    0    -1  
$EndComp
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F5F74E2
P 4050 3050
F 0 "#PWR?" H 3920 3090 50  0001 L CNN
F 1 "GND" H 4050 2950 50  0000 C CNN
F 2 "" H 4050 3050 50  0001 C CNN
F 3 "" H 4050 3050 50  0001 C CNN
	1    4050 3050
	1    0    0    -1  
$EndComp
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F5F6FCD
P 3800 3050
F 0 "#PWR?" H 3670 3090 50  0001 L CNN
F 1 "GND" H 3800 2950 50  0000 C CNN
F 2 "" H 3800 3050 50  0001 C CNN
F 3 "" H 3800 3050 50  0001 C CNN
	1    3800 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 2800 4300 2800
Connection ~ 4050 2800
Wire Wire Line
	4050 2900 4050 2800
Wire Wire Line
	4300 2800 4550 2800
Connection ~ 4300 2800
Wire Wire Line
	4300 2800 4300 2900
Wire Wire Line
	4550 2800 4650 2800
Connection ~ 4550 2800
Wire Wire Line
	4550 2900 4550 2800
Wire Wire Line
	3800 2800 4050 2800
Wire Wire Line
	3800 2900 3800 2800
$Comp
L agg-kicad:C C?
U 1 1 5F5D178D
P 4550 2900
F 0 "C?" H 4600 2970 50  0000 C CNN
F 1 "100n" H 4600 2830 50  0000 C CNN
F 2 "agg:0402" H 4550 2900 50  0001 C CNN
F 3 "" H 4550 2900 50  0001 C CNN
	1    4550 2900
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:C C?
U 1 1 5F5D13DA
P 4300 2900
F 0 "C?" H 4350 2970 50  0000 C CNN
F 1 "100n" H 4350 2830 50  0000 C CNN
F 2 "agg:0402" H 4300 2900 50  0001 C CNN
F 3 "" H 4300 2900 50  0001 C CNN
	1    4300 2900
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:C C?
U 1 1 5F5D0ED0
P 4050 2900
F 0 "C?" H 4100 2970 50  0000 C CNN
F 1 "100n" H 4100 2830 50  0000 C CNN
F 2 "agg:0402" H 4050 2900 50  0001 C CNN
F 3 "" H 4050 2900 50  0001 C CNN
	1    4050 2900
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:C C?
U 1 1 5F583255
P 3800 2900
AR Path="/5F0FD3F5/5F583255" Ref="C?"  Part="1" 
AR Path="/5F117D14/5F583255" Ref="C?"  Part="1" 
F 0 "C?" H 3850 2970 50  0000 C CNN
F 1 "4µ7" H 3850 2830 50  0000 C CNN
F 2 "agg:0603" H 3800 2900 50  0001 C CNN
F 3 "" H 3800 2900 50  0001 C CNN
	1    3800 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	4550 1150 4650 1150
Wire Wire Line
	4650 1150 4650 1100
Connection ~ 4550 1150
NoConn ~ 5750 4150
$Comp
L agg-kicad:TCXO_ST Y?
U 1 1 5F68E4F2
P 5200 4050
F 0 "Y?" H 5000 4150 50  0000 L CNN
F 1 "12M" H 5000 3850 50  0000 L CNN
F 2 "agg:XTAL-25x20" H 5000 4050 50  0001 C CNN
F 3 "" H 5000 4050 50  0001 C CNN
F 4 "2911104" H 5000 3750 50  0001 L CNN "Farnell"
	1    5200 4050
	1    0    0    -1  
$EndComp
NoConn ~ 5500 4150
Wire Wire Line
	5500 4050 5750 4050
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F6A3E86
P 4850 4000
F 0 "#PWR?" H 4850 4110 50  0001 L CNN
F 1 "3v3" H 4850 4090 50  0000 C CNN
F 2 "" H 4850 4000 50  0001 C CNN
F 3 "" H 4850 4000 50  0001 C CNN
	1    4850 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 4000 4850 4050
Wire Wire Line
	4850 4050 4900 4050
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F6AB4D5
P 4850 4200
F 0 "#PWR?" H 4720 4240 50  0001 L CNN
F 1 "GND" H 4850 4100 50  0000 C CNN
F 2 "" H 4850 4200 50  0001 C CNN
F 3 "" H 4850 4200 50  0001 C CNN
	1    4850 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 4200 4850 4150
Wire Wire Line
	4850 4150 4900 4150
$Comp
L agg-kicad:C C?
U 1 1 5F6B4816
P 4650 4050
F 0 "C?" H 4700 4120 50  0000 C CNN
F 1 "100n" H 4700 3980 50  0000 C CNN
F 2 "agg:0402" H 4650 4050 50  0001 C CNN
F 3 "" H 4650 4050 50  0001 C CNN
	1    4650 4050
	0    1    1    0   
$EndComp
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F6B4820
P 4650 4000
F 0 "#PWR?" H 4650 4110 50  0001 L CNN
F 1 "3v3" H 4650 4090 50  0000 C CNN
F 2 "" H 4650 4000 50  0001 C CNN
F 3 "" H 4650 4000 50  0001 C CNN
	1    4650 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 4000 4650 4050
$Comp
L agg-kicad:GND #PWR?
U 1 1 5F6B482B
P 4650 4200
F 0 "#PWR?" H 4520 4240 50  0001 L CNN
F 1 "GND" H 4650 4100 50  0000 C CNN
F 2 "" H 4650 4200 50  0001 C CNN
F 3 "" H 4650 4200 50  0001 C CNN
	1    4650 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 4200 4650 4150
$Comp
L agg-kicad:LED D?
U 1 1 5F6D8E5D
P 8500 4650
F 0 "D?" H 8500 4750 50  0000 L CNN
F 1 "GRN" H 8500 4575 50  0000 L CNN
F 2 "agg:0603-LED" H 8500 4650 50  0001 C CNN
F 3 "" H 8500 4650 50  0001 C CNN
F 4 "2290328" H 8500 4650 50  0001 C CNN "Farnell"
	1    8500 4650
	1    0    0    -1  
$EndComp
$Comp
L agg-kicad:R R?
U 1 1 5F6DA7D2
P 8800 4650
F 0 "R?" H 8850 4700 50  0000 C CNN
F 1 "1k" H 8850 4600 50  0000 C CNN
F 2 "" H 8800 4650 50  0001 C CNN
F 3 "" H 8800 4650 50  0001 C CNN
	1    8800 4650
	-1   0    0    1   
$EndComp
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F6DCC61
P 8900 4600
F 0 "#PWR?" H 8900 4710 50  0001 L CNN
F 1 "3v3" H 8900 4690 50  0000 C CNN
F 2 "" H 8900 4600 50  0001 C CNN
F 3 "" H 8900 4600 50  0001 C CNN
	1    8900 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 4600 8900 4650
Wire Wire Line
	8900 4650 8800 4650
Wire Wire Line
	8700 4650 8600 4650
Wire Wire Line
	7750 4650 8500 4650
$Comp
L agg-kicad:LED D?
U 1 1 5F7086B2
P 8050 4550
F 0 "D?" H 8050 4650 50  0000 L CNN
F 1 "GRN" H 8050 4475 50  0000 L CNN
F 2 "agg:0603-LED" H 8050 4550 50  0001 C CNN
F 3 "" H 8050 4550 50  0001 C CNN
F 4 "2290328" H 8050 4550 50  0001 C CNN "Farnell"
	1    8050 4550
	1    0    0    -1  
$EndComp
$Comp
L agg-kicad:R R?
U 1 1 5F7086BC
P 8350 4550
F 0 "R?" H 8400 4600 50  0000 C CNN
F 1 "1k" H 8400 4500 50  0000 C CNN
F 2 "" H 8350 4550 50  0001 C CNN
F 3 "" H 8350 4550 50  0001 C CNN
	1    8350 4550
	-1   0    0    1   
$EndComp
$Comp
L agg-kicad:3v3 #PWR?
U 1 1 5F7086C6
P 8450 4500
F 0 "#PWR?" H 8450 4610 50  0001 L CNN
F 1 "3v3" H 8450 4590 50  0000 C CNN
F 2 "" H 8450 4500 50  0001 C CNN
F 3 "" H 8450 4500 50  0001 C CNN
	1    8450 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 4500 8450 4550
Wire Wire Line
	8450 4550 8350 4550
Wire Wire Line
	8250 4550 8150 4550
Wire Wire Line
	7750 4550 8050 4550
$EndSCHEMATC