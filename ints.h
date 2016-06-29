//****************************************************************************
//*                                                                          *
//*                   _____   _____   _   _   ___                            *
//*                  |  ___| |  ___| | | | | |   `                           *
//*                  | |_    | |_    | |_| | | [] |                          *
//*                  |  _|   |  _|   |  _  | |  _ <                          *
//*                  | |___  | |___  | | | | | |_| |                         *
//*                  |_____| |_____| |_| |_| |____/                          *
//*                                                                          *
//*                                                                          *
//*  Project:          BIOSControl uPD70Fxxxx                                *
//*  (c) copyright     Harman-Becker Automotive Systems                      *
//****************************************************************************

#ifndef _INTS_H
#define _INTS_H


#define INT_DISABLE    0x0080
#define INT_ENABLE     0x0000
#define INT_REQFLAG    0x1000

//*--------------------------------------------------------------------------*
//* Tabelle für Interrupt-Initialisierung                                    *
//*                                                                          *
//* 0 : highest priority   15: lowest priority                               *
//*--------------------------------------------------------------------------*

//        Name          Enable/Disable | ILVL
#define INIT_INTOSTM0      INT_ENABLE  | 3   // File: timer.c     Systemtimer

#define INIT_LMA3IR        INT_DISABLE | 6   // File: ser.c
#define INIT_LMA3IS        INT_DISABLE | 6   // File: ser.c
#define INIT_LMA5IR        INT_DISABLE | 6   // File: ser.c
#define INIT_LMA5IS        INT_DISABLE | 6   // File: ser.c
#define INIT_LMA10IR       INT_DISABLE | 6   // File: ser.c
#define INIT_LMA10IS       INT_DISABLE | 6   // File: ser.c
#define INIT_LMA11IR       INT_DISABLE | 6   // File: ser.c
#define INIT_LMA11IS       INT_DISABLE | 6   // File: ser.c

#define INIT_ADCA0LLT      INT_ENABLE  | 10   // File: adc.c

#define INIT_FCNWUP        INT_ENABLE  | 7   // File: can3.c      Wakeup
#define INIT_FCN3ERR       INT_ENABLE  | 7   // File: can3.c      Error
#define INIT_FCN3REC       INT_ENABLE  | 7   // File: can3.c      Receive
#define INIT_FCN3TRX       INT_ENABLE  | 7   // File: can3.c      Transmit

#if 0
#define INIT_INTP0         INT_DISABLE  | 5  // File: hpipc.c     ext. Interrupt
#define INIT_INTP1         INT_DISABLE  | 5  // File: hpipc.c     ext. Interrupt
#define INIT_INTP2         INT_DISABLE  | 5  // File: hpipc.c     ext. Interrupt
#define INIT_INTP3         INT_DISABLE  | 5  // File: hpipc.c     ext. Interrupt
#define INIT_INTP4         INT_DISABLE  | 5  // File: hpipc.c     ext. Interrupt
#define INIT_INTP5         INT_DISABLE  | 5  // File: hpipc.c     ext. Interrupt
#define INIT_INTP6         INT_DISABLE  | 5  // File: hpipc.c     ext. Interrupt
#define INIT_INTP7         INT_DISABLE  | 5  // File: hpipc.c     ext. Interrupt
#define INIT_INTP8         INT_DISABLE  | 5  // File: hpipc.c     ext. Interrupt

#define INIT_INTTP0CC0     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP0CC1     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP1CC0     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP1CC1     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP2CC0     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP2CC1     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP3CC0     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP3CC1     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP4CC0     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP4CC1     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP5CC0     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP5CC1     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP6CC0     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP6CC1     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP7CC0     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP7CC1     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP8CC0     INT_DISABLE | 5   // File: ?           ext. Interrupt
#define INIT_INTTP8CC1     INT_DISABLE | 5   // File: ?           ext. Interrupt

#define INIT_INTTQ0CC1     INT_DISABLE | 5   // File: rds.c       FPGA-Int

#define INIT_INTCB0R       INT_DISABLE | 5   // File: spi.c
#define INIT_INTCB0T       INT_DISABLE | 5   // File: spi.c
#define INIT_INTCB1R       INT_DISABLE | 5   // File: spi.c
#define INIT_INTCB1T       INT_DISABLE | 5   // File: spi.c
#define INIT_INTCB2R       INT_DISABLE | 5   // File: spi.c
#define INIT_INTCB2T       INT_DISABLE | 5   // File: spi.c
#define INIT_INTCB3R       INT_DISABLE | 5   // File: spi.c
#define INIT_INTCB3T       INT_DISABLE | 5   // File: spi.c
#define INIT_INTCB4R       INT_DISABLE | 5   // File: spi.c
#define INIT_INTCB4T       INT_DISABLE | 5   // File: spi.c
#define INIT_INTCB5R       INT_DISABLE | 5   // File: spi.c
#define INIT_INTCB5T       INT_DISABLE | 5   // File: spi.c
#endif


//*--------------------------------------------------------------------------*
//* einige universelle Interruptmakros                                       *
//*--------------------------------------------------------------------------*

#define CONFIG_INTTP0CC1_P33_FALLINGEDGE  {TP0CTL0=0;TP0CTL1=0x05;TP0IOC1=0x0A;TP0OPT0=0x30;TP0CTL0=0x80;_PMC33=1;_PFC33=0;_PFCE33=0;TP0CCIC1=INIT_INTTP0CC1;}

#define CONFIG_INTTP6CC0_P01_FALLINGEDGE  {TP6CTL0=0;TP6CTL1=0x05;TP6IOC1=0x0A;TP6OPT0=0x30;TP6CTL0=0x80;_PFC01=0;_PMC01=1;TP6CCIC0=INIT_INTTP6CC0;}
#define CONFIG_INTTP6CC1_P00_FALLINGEDGE  {TP6CTL0=0;TP6CTL1=0x05;TP6IOC1=0x0A;TP6OPT0=0x30;TP6CTL0=0x80;_PFC00=0;_PMC00=1; TP6CCIC1=INIT_INTTP6CC1;}

#define CONFIG_INTTP7CC0_P69_FALLINGEDGE  {TP7CTL0=0;TP7CTL1=0x05;TP7IOC1=0x0A;TP7OPT0=0x30;TP7CTL0=0x80;_PFC69=0;_PMC69=1;TP7CCIC0=INIT_INTTP7CC0;}
#define CONFIG_INTTP7CC1_P610_FALLINGEDGE {TP7CTL0=0;TP7CTL1=0x05;TP7IOC1=0x0A;TP7OPT0=0x30;TP7CTL0=0x80; _PMC610=1; TP7CCIC1=INIT_INTTP7CC1;}

#define CONFIG_INTTP8CC0_P612_FALLINGEDGE {TP8CTL0=0;TP8CTL1=0x05;TP8IOC1=0x0A;TP8OPT0=0x30;TP8CTL0=0x80;_PFC612=0;_PMC612=1;TP8CCIC0=INIT_INTTP8CC0;}

#define CONFIG_INTTQ0CC1_P50_FALLINGEDGE  {TQ0CTL0=0;TQ0CTL1=0x05;TQ0IOC1=0x08;TQ0OPT0=0x20;TQ0CTL0=0x80;_PFCE50=0;_PFC50=1;_PMC50=1;TQ0CCIC1=INIT_INTTQ0CC1;}

#define CONFIG_INTP0_P03_FALLINGEDGE      {_PMC03=1;              _PFC03=0;  _INTF03=1;  _INTR03=0;  PIC0=INIT_INTP0;}
#define CONFIG_INTP1_P04_FALLINGEDGE      {_PMC04=1;                         _INTF04=1;  _INTR04=0;  PIC1=INIT_INTP1;}
#define CONFIG_INTP3_P06_FALLINGEDGE      {_PMC06=1;                         _INTF06=1;  _INTR06=0;  PIC3=INIT_INTP3;}
#define CONFIG_INTP6_P915_FALLINGEDGE     {_PMC915=1; _PFCE915=0; _PFC915=1; _INTF915=1; _INTR915=0; PIC6=INIT_INTP6;}



#endif // _INTS_H


