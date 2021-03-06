EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
	5200 2300 5600 2300
Wire Wire Line
	5600 2500 5200 2500
$Comp
L Device:R R4
U 1 1 61667AB2
P 3900 2550
F 0 "R4" V 4100 2550 50  0000 C CNN
F 1 "10" V 4000 2550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3830 2550 50  0001 C CNN
F 3 "~" H 3900 2550 50  0001 C CNN
F 4 "C22859" V 3900 2550 50  0001 C CNN "LCSC"
	1    3900 2550
	0    1    1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 616679C6
P 5050 2500
F 0 "C2" V 5300 2500 50  0000 C CNN
F 1 "1n 1000v" V 5200 2500 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 5088 2350 50  0001 C CNN
F 3 "~" H 5050 2500 50  0001 C CNN
F 4 "C1941" V 5050 2500 50  0001 C CNN "LCSC"
	1    5050 2500
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 616640A7
P 3900 2250
F 0 "R3" V 3693 2250 50  0000 C CNN
F 1 "10" V 3784 2250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3830 2250 50  0001 C CNN
F 3 "~" H 3900 2250 50  0001 C CNN
F 4 "C22859" V 3900 2250 50  0001 C CNN "LCSC"
	1    3900 2250
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 61663902
P 5050 2300
F 0 "C1" V 4798 2300 50  0000 C CNN
F 1 "1n 1000v" V 4889 2300 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 5088 2150 50  0001 C CNN
F 3 "~" H 5050 2300 50  0001 C CNN
F 4 "C1941" V 5050 2300 50  0001 C CNN "LCSC"
	1    5050 2300
	0    1    1    0   
$EndComp
$Comp
L Device:L_Core_Ferrite_Coupled_1324 L1
U 1 1 61661A9D
P 4450 2400
F 0 "L1" H 4450 2681 50  0000 C CNN
F 1 "ACT45B-101-2P-TL003" H 4450 2590 50  0000 C CNN
F 2 "footprints:ACT45B-101-2P-TL003" H 4450 2400 50  0001 C CNN
F 3 "~" H 4450 2400 50  0001 C CNN
F 4 "C88056" H 4450 2400 50  0001 C CNN "LCSC"
	1    4450 2400
	1    0    0    -1  
$EndComp
Text GLabel 5600 2500 2    50   BiDi ~ 0
D-
Text GLabel 5600 2300 2    50   BiDi ~ 0
D+
$Comp
L power:GND #PWR02
U 1 1 6166A15C
P 3650 3200
F 0 "#PWR02" H 3650 2950 50  0001 C CNN
F 1 "GND" H 3655 3027 50  0000 C CNN
F 2 "" H 3650 3200 50  0001 C CNN
F 3 "" H 3650 3200 50  0001 C CNN
	1    3650 3200
	1    0    0    -1  
$EndComp
$Comp
L Interface_UART:MAX3485 U1
U 1 1 6165CE73
P 2350 2350
F 0 "U1" H 1950 1900 50  0000 C CNN
F 1 "MAX3485" H 2050 1800 50  0000 C CNN
F 2 "Package_SO:SOP-8_3.9x4.9mm_P1.27mm" H 2350 1650 50  0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX3483-MAX3491.pdf" H 2350 2400 50  0001 C CNN
F 4 "C18148" H 2350 2350 50  0001 C CNN "LCSC"
	1    2350 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 3200 3650 3100
$Comp
L Device:R R2
U 1 1 61683B8E
P 3650 2950
F 0 "R2" H 3700 3000 50  0000 L CNN
F 1 "1k" H 3700 2900 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3580 2950 50  0001 C CNN
F 3 "~" H 3650 2950 50  0001 C CNN
F 4 "C21190" H 3650 2950 50  0001 C CNN "LCSC"
	1    3650 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 2550 3750 2550
Wire Wire Line
	3650 2800 3650 2550
Wire Wire Line
	3650 2250 3750 2250
Wire Wire Line
	3650 2000 3650 2250
Connection ~ 3650 2250
Connection ~ 3650 2550
Text GLabel 1950 2250 0    50   Output ~ 0
RXD
$Comp
L power:GND #PWR0102
U 1 1 6172F924
P 2350 2950
F 0 "#PWR0102" H 2350 2700 50  0001 C CNN
F 1 "GND" H 2355 2777 50  0000 C CNN
F 2 "" H 2350 2950 50  0001 C CNN
F 3 "" H 2350 2950 50  0001 C CNN
	1    2350 2950
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0103
U 1 1 617300FA
P 2350 1850
F 0 "#PWR0103" H 2350 1700 50  0001 C CNN
F 1 "+3V3" H 2365 2023 50  0000 C CNN
F 2 "" H 2350 1850 50  0001 C CNN
F 3 "" H 2350 1850 50  0001 C CNN
	1    2350 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 617307F6
P 1950 2350
F 0 "#PWR0104" H 1950 2100 50  0001 C CNN
F 1 "GND" V 1955 2222 50  0000 R CNN
F 2 "" H 1950 2350 50  0001 C CNN
F 3 "" H 1950 2350 50  0001 C CNN
	1    1950 2350
	0    1    1    0   
$EndComp
Text GLabel 1950 2450 0    50   Input ~ 0
TXE
Text GLabel 1950 2550 0    50   Input ~ 0
TXD
$Comp
L Interface_CAN_LIN:SN65HVD230 U5
U 1 1 6175EF02
P 7050 4450
F 0 "U5" H 7250 4800 50  0000 C CNN
F 1 "SN65HVD230" H 7300 4700 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 7050 3950 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn65hvd230.pdf" H 6950 4850 50  0001 C CNN
F 4 "C12084" H 7050 4450 50  0001 C CNN "LCSC"
	1    7050 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 61755FCF
P 1700 4500
F 0 "#PWR0111" H 1700 4250 50  0001 C CNN
F 1 "GND" V 1705 4372 50  0000 R CNN
F 2 "" H 1700 4500 50  0001 C CNN
F 3 "" H 1700 4500 50  0001 C CNN
	1    1700 4500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 6175530B
P 1500 5000
F 0 "#PWR0110" H 1500 4750 50  0001 C CNN
F 1 "GND" V 1505 4872 50  0000 R CNN
F 2 "" H 1500 5000 50  0001 C CNN
F 3 "" H 1500 5000 50  0001 C CNN
	1    1500 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 61754E42
P 1500 5500
F 0 "#PWR0109" H 1500 5250 50  0001 C CNN
F 1 "GND" V 1505 5372 50  0000 R CNN
F 2 "" H 1500 5500 50  0001 C CNN
F 3 "" H 1500 5500 50  0001 C CNN
	1    1500 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 617541B7
P 1700 6000
F 0 "#PWR0108" H 1700 5750 50  0001 C CNN
F 1 "GND" V 1705 5872 50  0000 R CNN
F 2 "" H 1700 6000 50  0001 C CNN
F 3 "" H 1700 6000 50  0001 C CNN
	1    1700 6000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 61752AB9
P 3100 6000
F 0 "#PWR0107" H 3100 5750 50  0001 C CNN
F 1 "GND" V 3105 5872 50  0000 R CNN
F 2 "" H 3100 6000 50  0001 C CNN
F 3 "" H 3100 6000 50  0001 C CNN
	1    3100 6000
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 61751895
P 3100 4500
F 0 "#PWR0105" H 3100 4250 50  0001 C CNN
F 1 "GND" V 3105 4372 50  0000 R CNN
F 2 "" H 3100 4500 50  0001 C CNN
F 3 "" H 3100 4500 50  0001 C CNN
	1    3100 4500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3400 4400 3100 4400
$Comp
L MCU_RaspberryPi_and_Boards:Pico U2
U 1 1 6171CBAA
P 2400 5250
F 0 "U2" H 2400 6465 50  0000 C CNN
F 1 "Pico" H 2400 6374 50  0000 C CNN
F 2 "footprints:RPi_Pico_SMD_TH" V 2400 5250 50  0001 C CNN
F 3 "" H 2400 5250 50  0001 C CNN
	1    2400 5250
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0112
U 1 1 61756F32
P 3100 4700
F 0 "#PWR0112" H 3100 4550 50  0001 C CNN
F 1 "+3V3" V 3100 4800 50  0000 L CNN
F 2 "" H 3100 4700 50  0001 C CNN
F 3 "" H 3100 4700 50  0001 C CNN
	1    3100 4700
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 61752145
P 3100 5500
F 0 "#PWR0106" H 3100 5250 50  0001 C CNN
F 1 "GND" V 3105 5372 50  0000 R CNN
F 2 "" H 3100 5500 50  0001 C CNN
F 3 "" H 3100 5500 50  0001 C CNN
	1    3100 5500
	0    -1   -1   0   
$EndComp
Text GLabel 3100 5800 2    50   Output ~ 0
SPI_MOSI
Text GLabel 3100 5900 2    50   Output ~ 0
SPI_CLK
Text GLabel 3100 6100 2    50   Output ~ 0
SPI_CS
Text GLabel 3100 6200 2    50   Input ~ 0
SPI_MISO
Text GLabel 7450 4550 2    50   BiDi ~ 0
CANL
Text GLabel 7450 4450 2    50   BiDi ~ 0
CANH
Text GLabel 3100 5600 2    50   Output ~ 0
8Mhz
$Comp
L power:GND #PWR0114
U 1 1 6179D267
P 2400 6400
F 0 "#PWR0114" H 2400 6150 50  0001 C CNN
F 1 "GND" H 2405 6227 50  0000 C CNN
F 2 "" H 2400 6400 50  0001 C CNN
F 3 "" H 2400 6400 50  0001 C CNN
	1    2400 6400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0117
U 1 1 617A3AEF
P 7050 4150
F 0 "#PWR0117" H 7050 4000 50  0001 C CNN
F 1 "+3V3" H 7065 4323 50  0000 C CNN
F 2 "" H 7050 4150 50  0001 C CNN
F 3 "" H 7050 4150 50  0001 C CNN
	1    7050 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 617A47A7
P 7050 4850
F 0 "#PWR0118" H 7050 4600 50  0001 C CNN
F 1 "GND" H 7055 4677 50  0000 C CNN
F 2 "" H 7050 4850 50  0001 C CNN
F 3 "" H 7050 4850 50  0001 C CNN
	1    7050 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 617A7038
P 6650 4650
F 0 "#PWR0119" H 6650 4400 50  0001 C CNN
F 1 "GND" V 6655 4522 50  0000 R CNN
F 2 "" H 6650 4650 50  0001 C CNN
F 3 "" H 6650 4650 50  0001 C CNN
	1    6650 4650
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0123
U 1 1 61749E96
P 3400 4100
F 0 "#PWR0123" H 3400 3950 50  0001 C CNN
F 1 "+5V" H 3415 4273 50  0000 C CNN
F 2 "" H 3400 4100 50  0001 C CNN
F 3 "" H 3400 4100 50  0001 C CNN
	1    3400 4100
	1    0    0    -1  
$EndComp
Text GLabel 3100 5700 2    50   Input ~ 0
CAN_INT
Text GLabel 5250 3050 0    50   BiDi ~ 0
D+
Text GLabel 5250 3150 0    50   BiDi ~ 0
D-
Text GLabel 7100 5300 0    50   BiDi ~ 0
CANL
Text GLabel 7100 5400 0    50   BiDi ~ 0
CANH
$Comp
L Device:C C9
U 1 1 6177FB88
P 8150 5450
F 0 "C9" H 8265 5496 50  0000 L CNN
F 1 "100n" H 8265 5405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8188 5300 50  0001 C CNN
F 3 "~" H 8150 5450 50  0001 C CNN
F 4 "C14663" H 8150 5450 50  0001 C CNN "LCSC"
	1    8150 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 617802D1
P 8500 5450
F 0 "C10" H 8615 5496 50  0000 L CNN
F 1 "100n" H 8615 5405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8538 5300 50  0001 C CNN
F 3 "~" H 8500 5450 50  0001 C CNN
F 4 "C14663" H 8500 5450 50  0001 C CNN "LCSC"
	1    8500 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 61780656
P 8850 5450
F 0 "C11" H 8965 5496 50  0000 L CNN
F 1 "100n" H 8965 5405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8888 5300 50  0001 C CNN
F 3 "~" H 8850 5450 50  0001 C CNN
F 4 "C14663" H 8850 5450 50  0001 C CNN "LCSC"
	1    8850 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 5600 8500 5600
Connection ~ 8500 5600
Wire Wire Line
	8500 5600 8850 5600
Wire Wire Line
	8150 5300 8500 5300
Connection ~ 8500 5300
Wire Wire Line
	8500 5300 8850 5300
$Comp
L power:+3V3 #PWR0124
U 1 1 6178B9C2
P 8500 5200
F 0 "#PWR0124" H 8500 5050 50  0001 C CNN
F 1 "+3V3" H 8515 5373 50  0000 C CNN
F 2 "" H 8500 5200 50  0001 C CNN
F 3 "" H 8500 5200 50  0001 C CNN
	1    8500 5200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 6178C12D
P 8500 5700
F 0 "#PWR0125" H 8500 5450 50  0001 C CNN
F 1 "GND" H 8505 5527 50  0000 C CNN
F 2 "" H 8500 5700 50  0001 C CNN
F 3 "" H 8500 5700 50  0001 C CNN
	1    8500 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 5200 8500 5300
Wire Wire Line
	8500 5600 8500 5700
$Comp
L Device:D_Schottky D3
U 1 1 61756641
P 3400 4250
F 0 "D3" V 3354 4330 50  0000 L CNN
F 1 "SS54" V 3445 4330 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 3400 4250 50  0001 C CNN
F 3 "~" H 3400 4250 50  0001 C CNN
F 4 "C22452" V 3400 4250 50  0001 C CNN "LCSC"
	1    3400 4250
	0    -1   -1   0   
$EndComp
Text GLabel 3100 5200 2    50   Output ~ 0
TXD
Text GLabel 3100 5100 2    50   Output ~ 0
TXE
Text GLabel 3100 4900 2    50   Input ~ 0
RXD
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 6189B0FC
P 7300 5400
F 0 "J2" H 7380 5392 50  0000 L CNN
F 1 "Conn_01x04" H 7380 5301 50  0000 L CNN
F 2 "footprints:SM04B1-CPTK-1A" H 7300 5400 50  0001 C CNN
F 3 "~" H 7300 5400 50  0001 C CNN
	1    7300 5400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 6189BF98
P 7100 5600
F 0 "#PWR07" H 7100 5350 50  0001 C CNN
F 1 "GND" V 7105 5472 50  0000 R CNN
F 2 "" H 7100 5600 50  0001 C CNN
F 3 "" H 7100 5600 50  0001 C CNN
	1    7100 5600
	0    1    1    0   
$EndComp
$Comp
L power:+12V #PWR08
U 1 1 6189C53F
P 7100 5500
F 0 "#PWR08" H 7100 5350 50  0001 C CNN
F 1 "+12V" V 7115 5628 50  0000 L CNN
F 2 "" H 7100 5500 50  0001 C CNN
F 3 "" H 7100 5500 50  0001 C CNN
	1    7100 5500
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 618A0D6F
P 5450 3150
F 0 "J1" H 5530 3142 50  0000 L CNN
F 1 "Conn_01x04" H 5530 3051 50  0000 L CNN
F 2 "footprints:SM04B1-CPTK-1A" H 5450 3150 50  0001 C CNN
F 3 "~" H 5450 3150 50  0001 C CNN
	1    5450 3150
	1    0    0    -1  
$EndComp
NoConn ~ 5250 3250
NoConn ~ 5250 3350
$Comp
L power:+3V3 #PWR0120
U 1 1 61901B1C
P 3250 2000
F 0 "#PWR0120" H 3250 1850 50  0001 C CNN
F 1 "+3V3" H 3265 2173 50  0000 C CNN
F 2 "" H 3250 2000 50  0001 C CNN
F 3 "" H 3250 2000 50  0001 C CNN
	1    3250 2000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 6168311D
P 3500 2000
F 0 "R1" V 3300 1950 50  0000 L CNN
F 1 "1k" V 3400 1950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3430 2000 50  0001 C CNN
F 3 "~" H 3500 2000 50  0001 C CNN
F 4 "C21190" V 3500 2000 50  0001 C CNN "LCSC"
	1    3500 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	3250 2000 3350 2000
$Comp
L power:GND #PWR01
U 1 1 6166A073
P 3150 3300
F 0 "#PWR01" H 3150 3050 50  0001 C CNN
F 1 "GND" H 3155 3127 50  0000 C CNN
F 2 "" H 3150 3300 50  0001 C CNN
F 3 "" H 3150 3300 50  0001 C CNN
	1    3150 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 3200 3150 3200
Wire Wire Line
	3150 3200 3150 3300
Connection ~ 3150 3200
$Comp
L Device:D_Zener D2
U 1 1 61660051
P 3150 3050
F 0 "D2" V 3104 3130 50  0000 L CNN
F 1 "5v TVS" V 3195 3130 50  0000 L CNN
F 2 "footprints:SOD882" H 3150 3050 50  0001 C CNN
F 3 "~" H 3150 3050 50  0001 C CNN
F 4 "C85401" V 3150 3050 50  0001 C CNN "LCSC"
	1    3150 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	3150 2250 3650 2250
Wire Wire Line
	2750 2250 3150 2250
Connection ~ 3150 2250
Wire Wire Line
	3150 2900 3150 2250
Wire Wire Line
	2900 2550 3650 2550
Wire Wire Line
	2900 2550 2750 2550
Connection ~ 2900 2550
Wire Wire Line
	2900 2900 2900 2550
$Comp
L Device:D_Zener D1
U 1 1 61668A6B
P 2900 3050
F 0 "D1" V 2850 2950 50  0000 R CNN
F 1 "5v TVS" V 2950 2950 50  0000 R CNN
F 2 "footprints:SOD882" H 2900 3050 50  0001 C CNN
F 3 "~" H 2900 3050 50  0001 C CNN
F 4 "C85401" V 2900 3050 50  0001 C CNN "LCSC"
	1    2900 3050
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR0101
U 1 1 61996986
P 3100 5400
F 0 "#PWR0101" H 3100 5250 50  0001 C CNN
F 1 "+3V3" V 3100 5500 50  0000 L CNN
F 2 "" H 3100 5400 50  0001 C CNN
F 3 "" H 3100 5400 50  0001 C CNN
	1    3100 5400
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 6199AFD3
P 7050 3550
F 0 "H1" H 7150 3553 50  0000 L CNN
F 1 "MountingHole" H 7150 3508 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 7050 3550 50  0001 C CNN
F 3 "~" H 7050 3550 50  0001 C CNN
	1    7050 3550
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 6199C740
P 7350 3550
F 0 "H2" H 7450 3553 50  0000 L CNN
F 1 "MountingHole" H 7450 3508 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 7350 3550 50  0001 C CNN
F 3 "~" H 7350 3550 50  0001 C CNN
	1    7350 3550
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 6199C8D6
P 7650 3550
F 0 "H3" H 7750 3553 50  0000 L CNN
F 1 "MountingHole" H 7750 3505 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 7650 3550 50  0001 C CNN
F 3 "~" H 7650 3550 50  0001 C CNN
	1    7650 3550
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 6199CA74
P 7950 3550
F 0 "H4" H 8050 3553 50  0000 L CNN
F 1 "MountingHole" H 8050 3505 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 7950 3550 50  0001 C CNN
F 3 "~" H 7950 3550 50  0001 C CNN
	1    7950 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 3650 7350 3650
Connection ~ 7350 3650
Wire Wire Line
	7350 3650 7650 3650
Connection ~ 7650 3650
Wire Wire Line
	7650 3650 7950 3650
$Comp
L power:GND #PWR0126
U 1 1 619B5460
P 7950 3650
F 0 "#PWR0126" H 7950 3400 50  0001 C CNN
F 1 "GND" H 7955 3477 50  0000 C CNN
F 2 "" H 7950 3650 50  0001 C CNN
F 3 "" H 7950 3650 50  0001 C CNN
	1    7950 3650
	1    0    0    -1  
$EndComp
Connection ~ 7950 3650
$Comp
L Regulator_Switching:TPS62160DGK U3
U 1 1 6174E98B
P 7150 2550
F 0 "U3" H 7150 3117 50  0000 C CNN
F 1 "TPS62160DGK" H 7150 3026 50  0000 C CNN
F 2 "Package_SO:MSOP-8_3x3mm_P0.65mm" H 7300 2200 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps62160.pdf" H 7150 3100 50  0001 C CNN
F 4 "C60726" H 7150 2550 50  0001 C CNN "LCSC"
	1    7150 2550
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0121
U 1 1 6174FDA2
P 6550 2150
F 0 "#PWR0121" H 6550 2000 50  0001 C CNN
F 1 "+12V" H 6565 2323 50  0000 C CNN
F 2 "" H 6550 2150 50  0001 C CNN
F 3 "" H 6550 2150 50  0001 C CNN
	1    6550 2150
	1    0    0    -1  
$EndComp
$Comp
L Device:L L2
U 1 1 61754A5C
P 7800 2250
F 0 "L2" V 7619 2250 50  0000 C CNN
F 1 "2.2u" V 7710 2250 50  0000 C CNN
F 2 "footprints:1227ASH100MP2" H 7800 2250 50  0001 C CNN
F 3 "~" H 7800 2250 50  0001 C CNN
F 4 "C435389" V 7800 2250 50  0001 C CNN "LCSC"
	1    7800 2250
	0    1    1    0   
$EndComp
$Comp
L Device:C C3
U 1 1 61755D56
P 6550 2500
F 0 "C3" H 6665 2546 50  0000 L CNN
F 1 "10u" H 6665 2455 50  0000 L CNN
F 2 "Capacitor_SMD:C_1210_3225Metric" H 6588 2350 50  0001 C CNN
F 3 "~" H 6550 2500 50  0001 C CNN
F 4 "C77102" H 6550 2500 50  0001 C CNN "LCSC"
	1    6550 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 617571F5
P 8500 2400
F 0 "C4" H 8615 2446 50  0000 L CNN
F 1 "22u" H 8615 2355 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 8538 2250 50  0001 C CNN
F 3 "~" H 8500 2400 50  0001 C CNN
F 4 "C97950" H 8500 2400 50  0001 C CNN "LCSC"
	1    8500 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 6175E11C
P 7200 3050
F 0 "#PWR0122" H 7200 2800 50  0001 C CNN
F 1 "GND" H 7205 2877 50  0000 C CNN
F 2 "" H 7200 3050 50  0001 C CNN
F 3 "" H 7200 3050 50  0001 C CNN
	1    7200 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 3050 7200 3000
Wire Wire Line
	7200 3000 7150 3000
Wire Wire Line
	7150 3000 7150 2950
Wire Wire Line
	7250 2950 7250 3000
Wire Wire Line
	7250 3000 7200 3000
Connection ~ 7200 3000
Wire Wire Line
	6550 2150 6550 2250
Wire Wire Line
	6550 2250 6750 2250
Wire Wire Line
	6550 2350 6550 2250
Connection ~ 6550 2250
Wire Wire Line
	6750 2350 6550 2350
Connection ~ 6550 2350
Wire Wire Line
	6550 2650 6550 3000
Wire Wire Line
	6550 3000 7150 3000
Connection ~ 7150 3000
Wire Wire Line
	7550 2250 7650 2250
$Comp
L Device:R R6
U 1 1 6177B994
P 8100 2400
F 0 "R6" H 8170 2446 50  0000 L CNN
F 1 "220k" H 8170 2355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8030 2400 50  0001 C CNN
F 3 "~" H 8100 2400 50  0001 C CNN
F 4 "C22961" H 8100 2400 50  0001 C CNN "LCSC"
	1    8100 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 6177C2E2
P 8100 2800
F 0 "R7" H 8170 2846 50  0000 L CNN
F 1 "47k" H 8170 2755 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8030 2800 50  0001 C CNN
F 3 "~" H 8100 2800 50  0001 C CNN
F 4 "C25819" H 8100 2800 50  0001 C CNN "LCSC"
	1    8100 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 3000 8100 3000
Connection ~ 7250 3000
Wire Wire Line
	8100 3000 8100 2950
Wire Wire Line
	7550 2450 7950 2450
Wire Wire Line
	7950 2450 7950 2600
Wire Wire Line
	7950 2600 8100 2600
Wire Wire Line
	8100 2600 8100 2550
Wire Wire Line
	8100 2600 8100 2650
Connection ~ 8100 2600
Wire Wire Line
	7950 2250 8000 2250
Wire Wire Line
	8000 2250 8000 2350
Connection ~ 8000 2250
Wire Wire Line
	8000 2250 8100 2250
Wire Wire Line
	7550 2350 8000 2350
Wire Wire Line
	8100 3000 8500 3000
Wire Wire Line
	8500 3000 8500 2550
Connection ~ 8100 3000
Wire Wire Line
	8100 2250 8500 2250
Connection ~ 8100 2250
$Comp
L power:+5V #PWR0127
U 1 1 6179C7AD
P 8500 2150
F 0 "#PWR0127" H 8500 2000 50  0001 C CNN
F 1 "+5V" H 8515 2323 50  0000 C CNN
F 2 "" H 8500 2150 50  0001 C CNN
F 3 "" H 8500 2150 50  0001 C CNN
	1    8500 2150
	1    0    0    -1  
$EndComp
Connection ~ 8500 2250
Wire Wire Line
	8500 2150 8500 2250
$Comp
L Device:R R5
U 1 1 617A40C5
P 6350 2800
F 0 "R5" V 6143 2800 50  0000 C CNN
F 1 "100k" V 6234 2800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6280 2800 50  0001 C CNN
F 3 "~" H 6350 2800 50  0001 C CNN
F 4 "C25803" V 6350 2800 50  0001 C CNN "LCSC"
	1    6350 2800
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0128
U 1 1 617A550B
P 6100 2700
F 0 "#PWR0128" H 6100 2550 50  0001 C CNN
F 1 "+5V" H 6115 2873 50  0000 C CNN
F 2 "" H 6100 2700 50  0001 C CNN
F 3 "" H 6100 2700 50  0001 C CNN
	1    6100 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 2700 6100 2800
Wire Wire Line
	6100 2800 6200 2800
Wire Wire Line
	6500 2800 6650 2800
Wire Wire Line
	6650 2800 6650 2650
Wire Wire Line
	6650 2650 6750 2650
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 6175689E
P 1050 5000
F 0 "J3" H 968 4675 50  0000 C CNN
F 1 "CONF1" H 968 4766 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1050 5000 50  0001 C CNN
F 3 "~" H 1050 5000 50  0001 C CNN
	1    1050 5000
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 61757EB6
P 1050 5500
F 0 "J4" H 968 5175 50  0000 C CNN
F 1 "CONF2" H 968 5266 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1050 5500 50  0001 C CNN
F 3 "~" H 1050 5500 50  0001 C CNN
	1    1050 5500
	-1   0    0    1   
$EndComp
Wire Wire Line
	1250 5500 1500 5500
Connection ~ 1500 5500
Wire Wire Line
	1500 5500 1700 5500
Wire Wire Line
	1250 5400 1700 5400
Wire Wire Line
	1250 5000 1500 5000
Connection ~ 1500 5000
Wire Wire Line
	1500 5000 1700 5000
Wire Wire Line
	1250 4900 1700 4900
Text GLabel 8250 4300 2    50   BiDi ~ 0
CANL
Text GLabel 8250 4400 2    50   BiDi ~ 0
CANH
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 61783CE3
P 8200 4700
F 0 "J5" V 8072 4780 50  0000 L CNN
F 1 "CAN_TERM" V 8163 4780 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8200 4700 50  0001 C CNN
F 3 "~" H 8200 4700 50  0001 C CNN
	1    8200 4700
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 61784F42
P 8050 4300
F 0 "R8" V 8257 4300 50  0000 C CNN
F 1 "120" V 8166 4300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 7980 4300 50  0001 C CNN
F 3 "~" H 8050 4300 50  0001 C CNN
F 4 "C17909" V 8050 4300 50  0001 C CNN "LCSC"
	1    8050 4300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8250 4400 8200 4400
Wire Wire Line
	8200 4400 8200 4500
Wire Wire Line
	8100 4500 7900 4500
Wire Wire Line
	7900 4500 7900 4300
Wire Wire Line
	8200 4300 8250 4300
$Comp
L Interface_CAN_LIN:MCP2515-xST U4
U 1 1 61732E24
P 5200 4950
F 0 "U4" H 5450 5800 50  0000 C CNN
F 1 "MCP2515-xST" H 5500 5700 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 5200 4050 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf" H 5300 4150 50  0001 C CNN
F 4 "C15193" H 5200 4950 50  0001 C CNN "LCSC"
	1    5200 4950
	1    0    0    -1  
$EndComp
Text GLabel 5800 4950 2    50   Output ~ 0
CAN_INT
$Comp
L power:+3V3 #PWR0116
U 1 1 617A14DE
P 5200 4150
F 0 "#PWR0116" H 5200 4000 50  0001 C CNN
F 1 "+3V3" H 5215 4323 50  0000 C CNN
F 2 "" H 5200 4150 50  0001 C CNN
F 3 "" H 5200 4150 50  0001 C CNN
	1    5200 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 6179DF89
P 5200 5750
F 0 "#PWR0115" H 5200 5500 50  0001 C CNN
F 1 "GND" H 5205 5577 50  0000 C CNN
F 2 "" H 5200 5750 50  0001 C CNN
F 3 "" H 5200 5750 50  0001 C CNN
	1    5200 5750
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0113
U 1 1 6179A3A6
P 5800 5550
F 0 "#PWR0113" H 5800 5400 50  0001 C CNN
F 1 "+3V3" V 5800 5650 50  0000 L CNN
F 2 "" H 5800 5550 50  0001 C CNN
F 3 "" H 5800 5550 50  0001 C CNN
	1    5800 5550
	0    1    1    0   
$EndComp
Text GLabel 4600 4450 0    50   Output ~ 0
SPI_MISO
Text GLabel 4600 4350 0    50   Input ~ 0
SPI_MOSI
Text GLabel 4600 4650 0    50   Input ~ 0
SPI_CLK
Text GLabel 4600 4550 0    50   Input ~ 0
SPI_CS
Text GLabel 4600 5250 0    50   Input ~ 0
8Mhz
Text GLabel 6650 4350 0    50   Input ~ 0
TXCAN
Text GLabel 6650 4450 0    50   Output ~ 0
RXCAN
Text GLabel 5800 4450 2    50   Output ~ 0
TXCAN
Text GLabel 5800 4350 2    50   Input ~ 0
RXCAN
Text Notes 2850 1650 0    50   ~ 0
RS348 for differential communication with battery module
Text Notes 6950 1650 0    50   ~ 0
12v to 4.5v power supply
Text Notes 6100 4050 0    50   ~ 0
CAN
Wire Wire Line
	4050 2250 4150 2250
Wire Wire Line
	4150 2250 4150 2300
Wire Wire Line
	4150 2300 4250 2300
Wire Wire Line
	4050 2550 4150 2550
Wire Wire Line
	4150 2550 4150 2500
Wire Wire Line
	4150 2500 4250 2500
Wire Wire Line
	4900 2300 4650 2300
Wire Wire Line
	4650 2500 4900 2500
$EndSCHEMATC
