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

#ifndef _global_h
#define _global_h


//*--------------------------------------------------------------------------*
//* Definitionen                                                             *
//*--------------------------------------------------------------------------*

// für die Statusverwaltung
extern UINT32*          modePtr;
extern UINT8            CPU_Running ;

// für die Begrüssungsmeldung
extern UINT8            verDate[];
extern UINT8            verTime[];
extern UINT8            verStr[];

//---- sprintf ----
extern INT8             str[256];

extern BOOL             debug_msg_on;
extern BOOL             intern_cmd;

extern UINT32*          errorPtr;

extern UINT32           modePtrAdd;
extern UINT32           errorPtrAdd;

extern UINT8 * volatile resultPtr;
extern UINT8 * volatile commandPtr;


#define RESULT_LENGTH(x) {resultPtr[0] = x;}


#endif // ifndef _global_h
