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

#ifndef _PWM_H
#define _PWM_H

//*--------------------------------------------------------------------------*
//* PWM Error Messages                                                       *
//*--------------------------------------------------------------------------*

#define ERROR_PWM_MODE           0x00000001U
#define ERROR_PWM_BIT            0x00000002U
#define ERROR_PWM_RANGE          0x00000004U

//*--------------------------------------------------------------------------*
//* Funktionen                                                               *
//*--------------------------------------------------------------------------*

extern void PWM             (void);


#endif // _PWM_H

