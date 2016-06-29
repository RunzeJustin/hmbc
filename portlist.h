//               Projekt :           L122
//               Musterstand :       B-Sample
//               Autor :
//               Datum :
//               Vorlage :           HW L122 (3554)
//               C-File Name :       portlist_L122.h
// ---------------------------------------------------------------------------------------


//               Name               Port  Bit   Init Dir Init Lev PWD Dir  PWD Lev  Kommentar
OD_PORTDEF       (                  0)
OD_IOPORT        (   P0_0,          0,    0,    IN,      LOW,     IN,      LOW     ) // WDGDISABLE
OD_IOPORT        (   P0_1,          0,    1,    IN,      LOW,     IN,      LOW     ) // FLMD1
OD_IOPORT        (   P0_2,          0,    2,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P0_3,          0,    3,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P0_4,          0,    4,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P0_5,          0,    5,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P0_6,          0,    6,    PP,      HIGH,    IN,      LOW     ) // serial UART TX
OD_IOPORT        (   P0_7,          0,    7,    IN,      LOW,     IN,      LOW     ) // serial UART RX
OD_IOPORT        (   P0_8,          0,    8,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P0_9,          0,    9,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P0_10,         0,    10,   PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P0_11,         0,    11,   PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P0_12,         0,    12,   PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P0_13,         0,    13,   PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P0_14,         0,    14,   PP,      HIGH,    IN,      LOW     ) // Reserved
OD_IOPORT        (   P0_15,         0,    15,   PP,      HIGH,    IN,      LOW     ) // Reserved
OD_ENDPORTDEF    (                  0)


//               Name               Port  Bit   Init Dir Init Lev PWD Dir  PWD Lev  Kommentar
OD_PORTDEF       (                  1)
OD_IOPORT        (   P1_1,          1,    1,    IN,      LOW,     IN,      LOW     ) // AMP_AB_CLIO timer input
OD_IOPORT        (   P1_2,          1,    2,    PP,      HIGH,    IN,      LOW     ) // MCU_MOST-3dB
OD_IOPORT        (   P1_3,          1,    3,    IN,      LOW,     IN,      LOW     ) // AMP_CLSD_CLIO timer input
OD_IOPORT        (   P1_4,          1,    4,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P1_5,          1,    5,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P1_6,          1,    6,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P1_7,          1,    7,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P1_8,          1,    8,    IN,      HIGH,    IN,      LOW     ) // PSD_STP_N
OD_IOPORT        (   P1_9,          1,    9,    PP,      HIGH,    IN,      LOW     ) // MCU_MIFRST_N
OD_IOPORT        (   P1_10,         1,    10,   IN,      LOW,     IN,      LOW     ) // AMP_AB_CLIP interupt input
OD_IOPORT        (   P1_11,         1,    11,   IN,      LOW,     IN,      LOW     ) // AMP_CLSD_CLIP interupt input
OD_IOPORT        (   P1_12,         1,    12,   IN,      LOW,     IN,      LOW     ) // DSP_MCUBUSY
OD_IOPORT        (   P1_13,         1,    13,   PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P1_14,         1,    14,   IN,      LOW,     IN,      LOW     ) // MIF_I2C1INT_N
OD_IOPORT        (   P1_15,         1,    15,   IN,      LOW,     IN,      LOW     ) // MIF_SPI0_INT_N
OD_ENDPORTDEF    (                  1)

//	Name	Port	Bit	Init Dir	Init Lev	PWD Dir	PWD Lev		Kommentar
OD_PORTDEF		 (					2)
OD_IOPORT		 (   P2_0,          2,	  0,	PP,	     HIGH,  IN,	   LOW	   ) //Open
OD_ENDPORTDEF	 (		            2)

//               Name               Port  Bit   Init Dir Init Lev PWD Dir  PWD Lev  Kommentar
OD_PORTDEF       (                  3)
OD_IOPORT        (   P3_2,          3,    2,    IN,      LOW,     IN,      LOW     ) // MIF_ERROR_BOOT
OD_IOPORT        (   P3_3,          3,    3,    PP,      HIGH,    IN,      LOW     ) // MCU_SPI0_EEP_CS_N
OD_IOPORT        (   P3_4,          3,    4,    PP,      HIGH,    IN,      LOW     ) // MCU_SPI0_MIF_CS_N
OD_IOPORT        (   P3_5,          3,    5,    PP,      LOW,     IN,      LOW     ) // MCU_SPI0_CLK
OD_IOPORT        (   P3_6,          3,    6,    PP,      LOW,     IN,      LOW     ) // MCU_SPI0_MOSI
OD_IOPORT        (   P3_7,          3,    7,    IN,      LOW,     IN,      LOW     ) // MCU_SPI0_MISO
OD_ENDPORTDEF    (                  3)


//               Name               Port  Bit   Init Dir Init Lev PWD Dir  PWD Lev  Kommentar
OD_PORTDEF       (                  4)
OD_IOPORT        (   P4_0,          4,    0,    IN,      LOW,     IN,      LOW     ) // parallel
OD_IOPORT        (   P4_1,          4,    1,    IN,      LOW,     IN,      LOW     ) // parallel
OD_IOPORT        (   P4_2,          4,    2,    IN,      LOW,     IN,      LOW     ) // parallel
OD_IOPORT        (   P4_3,          4,    3,    IN,      LOW,     IN,      LOW     ) // parallel
OD_IOPORT        (   P4_4,          4,    4,    IN,      LOW,     IN,      LOW     ) // parallel
OD_IOPORT        (   P4_5,          4,    5,    IN,      LOW,     IN,      LOW     ) // parallel
OD_IOPORT        (   P4_6,          4,    6,    IN,      LOW,     IN,      LOW     ) // parallel
OD_IOPORT        (   P4_7,          4,    7,    IN,      LOW,     IN,      LOW     ) // parallel
OD_IOPORT        (   P4_8,          4,    8,    PP,      LOW,     IN,      LOW     ) // parallel
OD_IOPORT        (   P4_9,          4,    9,    IN,      LOW,     IN,      LOW     ) // parallel
OD_IOPORT        (   P4_10,         4,    10,   IN,      LOW,     IN,      LOW     ) // parallel
OD_IOPORT        (   P4_11,         4,    11,   PP,      HIGH,    IN,      LOW     ) // parallel
OD_ENDPORTDEF    (                  4)


//               Name               Port  Bit   Init Dir Init Lev PWD Dir  PWD Lev  Kommentar
OD_PORTDEF       (                  10)
OD_IOPORT        (   P10_6,         10,   6,    PP,      LOW,    IN,      LOW     ) // MCU_DSPAPP(0)
OD_IOPORT        (   P10_7,         10,   7,    PP,      LOW,    IN,      LOW     ) // MCU_DSPAPP(1)
OD_IOPORT        (   P10_8,         10,   8,    PP,      LOW,     IN,      LOW     ) // MCU_DSPDOCMD
OD_IOPORT        (   P10_9,         10,   9,    PP,      LOW,    IN,      LOW     ) // MCU_DSPWRACK_N
OD_IOPORT        (   P10_10,        10,   10,   IN,      LOW,     IN,      LOW     ) // DSP_MCUWRITE
OD_IOPORT        (   P10_11,        10,   11,   IN,      LOW,     IN,      LOW     ) // DSP_MCUWDG
OD_IOPORT        (   P10_12,        10,   12,   PP,      LOW,     IN,      LOW     ) // MCU_DSPRST_N
OD_IOPORT        (   P10_13,        10,   13,   PP,      HIGH,    IN,      LOW     ) // FPB_MCURESERVED(1)
OD_IOPORT        (   P10_14,        10,   14,   PP,      LOW,     IN,      LOW     ) // MCU_DSPSELECT
OD_IOPORT        (   P10_15,        10,   15,   PP,      HIGH,    IN,      LOW     ) // open
OD_ENDPORTDEF    (                  10)


//               Name               Port  Bit   Init Dir Init Lev PWD Dir  PWD Lev  Kommentar
OD_PORTDEF       (                  11)
OD_IOPORT        (   P11_0,         11,   0,    PP,      HIGH,    IN,      LOW     ) // MCU_FOTON
OD_IOPORT        (   P11_1,         11,   1,    IN,      LOW,     IN,      LOW     ) // WDG_EVENT
OD_IOPORT        (   P11_2,         11,   2,    PP,      LOW,     IN,      LOW     ) // MCU_WDGTRG
OD_IOPORT        (   P11_3,         11,   3,    PP,      LOW,     IN,      LOW     ) // MCU_HOLDPOWER_N
OD_IOPORT        (   P11_4,         11,   4,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P11_5,         11,   5,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P11_6,         11,   6,    PP,      HIGH,    IN,      LOW     ) // AMP_HWMUTE_N
OD_IOPORT        (   P11_7,         11,   7,    PP,      HIGH,    IN,      LOW     ) // PWC_AMPSTBY_N
OD_ENDPORTDEF    (                  11)

//               Name               Port  Bit   Init Dir Init Lev PWD Dir  PWD Lev  Kommentar
OD_PORTDEF       (                  21)
OD_IOPORT        (   P21_2,         21,   2,    IN,      LOW,     IN,      LOW     ) // Hardware version info
OD_IOPORT        (   P21_3,         21,   3,    IN,      LOW,     IN,      LOW     ) // Hardware version info
OD_IOPORT        (   P21_4,         21,   4,    IN,      LOW,     IN,      LOW     ) // Hardware version info
OD_IOPORT        (   P21_5,         21,   5,    PP,      HIGH,    IN,      LOW     ) // CLK_BUCKCLK_P
OD_IOPORT        (   P21_6,         21,   6,    PP,      HIGH,    IN,      LOW     ) // CLK_BUCKCLK_N
OD_IOPORT        (   P21_7,         21,   7,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P21_8,         21,   8,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P21_9,         21,   9,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P21_10,        21,   10,   PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P21_11,        21,   11,   PP,      HIGH,    IN,      LOW     ) // open
OD_ENDPORTDEF    (                  21)

//               Name               Port  Bit   Init Dir Init Lev PWD Dir  PWD Lev  Kommentar
OD_PORTDEF       (                  25)
OD_IOPORT        (   P25_0,         25,   0,    PP,      LOW,     IN,      LOW     ) // MCU_UBAT_OC
OD_IOPORT        (   P25_1,         25,   1,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P25_2,         25,   2,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P25_3,         25,   3,    IN,      LOW,     IN,      LOW     ) // MCU_SPI4_MISO
OD_IOPORT        (   P25_4,         25,   4,    PP,      LOW,     IN,      LOW     ) // MCU_SPI4_MOSI
OD_IOPORT        (   P25_5,         25,   5,    PP,      LOW,     IN,      LOW     ) // MCU_SPI4_CLK
OD_IOPORT        (   P25_6,         25,   6,    PP,      HIGH,    IN,      LOW     ) // MCU_SPI4DSPCS_N
OD_IOPORT        (   P25_7,         25,   7,    PP,      HIGH,    IN,      LOW     ) // MCU_SPI4FPBCS_N
OD_IOPORT        (   P25_8,         25,   8,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P25_9,         25,   9,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P25_10,        25,   10,   PP,      LOW,     IN,      LOW     ) // MCU_I2C2SDA
OD_IOPORT        (   P25_11,        25,   11,   PP,      LOW,     IN,      LOW     ) // MCU_I2C2SCL
OD_IOPORT        (   P25_12,        25,   12,   PP,      LOW,     IN,      LOW     ) // MCU_I2C1SDA
OD_IOPORT        (   P25_13,        25,   13,   PP,      LOW,     IN,      LOW     ) // MCU_I2C1SCL
OD_IOPORT        (   P25_14,        25,   14,   PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P25_15,        25,   15,   PP,      HIGH,    IN,      LOW     ) // open
OD_ENDPORTDEF    (                  25)

//               Name               Port  Bit   Init Dir Init Lev PWD Dir  PWD Lev  Kommentar
OD_PORTDEF       (                  27)
OD_IOPORT        (   P27_0,         27,   0,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P27_1,         27,   1,    PP,      HIGH,    IN,      LOW     ) // open
OD_IOPORT        (   P27_2,         27,   2,    PP,      HIGH,    IN,      LOW     ) // open
OD_ENDPORTDEF    (                  27)

//               Name               Port  Bit   Init Dir Init Lev PWD Dir  PWD Lev  Kommentar
//JOD_PORTDEF      (                  0)
//JOD_IOPORT       (   JP0_0,         0,    0,    IN,      LOW,     IN,      LOW     ) // INTP0
//JOD_IOPORT       (   JP0_1,         0,    1,    IN,      LOW,     IN,      LOW     ) // INTP1
//JOD_IOPORT       (   JP0_2,         0,    2,    IN,      LOW,     IN,      LOW     ) // INTP2
//JOD_IOPORT       (   JP0_3,         0,    3,    IN,      LOW,     IN,      LOW     ) // INTP3
//JOD_IOPORT       (   JP0_4,         0,    4,    IN,      LOW,     IN,      LOW     ) // ?
//JOD_IOPORT       (   JP0_5,         0,    5,    IN,      LOW,     IN,      LOW     ) // ?
//JOD_ENDPORTDEF   (                  0)


