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


#ifndef _TIMER_H
#define _TIMER_H

//*--------------------------------------------------------------------------*
//* Definitionen zur Delaysteuerung                                          *
//*--------------------------------------------------------------------------*

#define TIMER_WAIT_MS	      0     // exclusiv fuer func wait_ms

#define TIMER_WAIT_SIGNAL	   1     // globaler Signaltimer
                                    // nur in abgeschlossenen Funktionen
                                    // nicht im Interrupt

#define TIMER_TX_GLOBAL	      2     // globaler TX-Wartetimer
                                    // nur in Sendefunktionen
                                    // nicht im Interrupt

#define TIMER_BYPASSCTL       3     // exclusiv fuer uMost
#define TIMER_IPC             4     // exclusiv fuer HPIPC
#define TIMER_HOSTWAIT        5     // warten auf Kommunikation

#define TIMER_RS232_DEV_3     6     // exclusiv im RS232 RX-Interrupt
#define TIMER_RS232_DEV_5     7     // exclusiv im RS232 RX-Interrupt
#define TIMER_RS232_DEV_10    8     // exclusiv im RS232 RX-Interrupt
#define TIMER_RS232_DEV_11    9     // exclusiv im RS232 RX-Interrupt

#define TIMER_SPI_DEV_0       11    // exclusiv im SPI-Interrupt
#define TIMER_SPI_DEV_1       12    // exclusiv im SPI-Interrupt
#define TIMER_SPI_DEV_2       13    // exclusiv im SPI-Interrupt
#define TIMER_SPI_DEV_3       14    // exclusiv im SPI-Interrupt
#define TIMER_SPI_DEV_4       15    // exclusiv im SPI-Interrupt
#define TIMER_SPI_DEV_5       16    // exclusiv im SPI-Interrupt

#define TIMER_MOST_DEV_0      17    // exclusiv im MOST-Interrupt



#define SYS_TICK              1     // Zeitbasis in ms fuer alle Delays

#define TIM_MAX_MRK           18    // max. number of markers (0-3) free for SH compatible code



//*--------------------------------------------------------------------------*
//* external functions                                                       *
//*--------------------------------------------------------------------------*

extern void 			timWait        (UINT32 x);
extern BOOL          timTimeout     (UINT8 marker, UINT32 timeout);
extern UINT32        timGetTime     (void);
extern UINT32        timDifference  (UINT32 time);
extern BOOL          timSetMarker   (UINT8 marker);
extern UINT32        get_ms         (void);

// Legacy, wegen  alter Timer-Off Funktionalitaet :  dlyhnd(Timerxx,0,OFF);
extern BOOL          timStopTimer   (UINT8 marker);

//*--------------------------------------------------------------------------*
//* externe Funktionen                                                       *
//*--------------------------------------------------------------------------*

extern void          dlyini         (void);
extern void 	      dlyoff         (void);


extern void Delay_us (UINT32 Wert);

#if (CP_DEBUG == 17)
   extern void Debug_Info (void);
#endif

#endif // _TIMER_H

