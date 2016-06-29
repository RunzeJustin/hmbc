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


//*--------------------------------------------------------------------------*
//* Include files                                                            *
//*--------------------------------------------------------------------------*

#include "config.h"
#include "define.h"
#include "hbbios.h"

#include "global.h"                 // globale Datenstrukturen

#if (CP_IIC0 == 1)
   #include "iic_0.h"
#endif

#if (CP_IIC1 == 1)
   #include "iic_1.h"
#endif

#if (CP_IIC2 == 1)
   #include "iic_2.h"
#endif

#if (CP_IIC3 == 1)
   #include "iic_3.h"
#endif

#if (CP_IIC4 == 1)
   #include "iic_4.h"
#endif

#if (CP_IIC5 == 1)
   #include "iic_5.h"
#endif

#if (CP_IIC6 == 1)
   #include "iic_6.h"
#endif

#if (CP_IIC7 == 1)
   #include "iic_7.h"
#endif

#if (CP_IIC10 == 1)
   #include "iic_10.h"
#endif

#if (CP_IIC11 == 1)
   #include "iic_11.h"
#endif

#include "iic.h"
#include "tool.h"

//*--------------------------------------------------------------------------*
//*	Hauptschalter für das ganze File ab hier                               *
//*--------------------------------------------------------------------------*

#if (CP_IIC0 || CP_IIC1 || CP_IIC2 || CP_IIC3 || CP_IIC4 || CP_IIC5 || CP_IIC6 || CP_IIC7 || CP_IIC10 || CP_IIC11)

//*--------------------------------------------------------------------------*
//* Prototypes                                                               *
//*--------------------------------------------------------------------------*

void IIC_RD (void);
void IIC_WR (void);

//****************************************************************************
//* Function     : IIC_RD                                                    *
//****************************************************************************
//* Description  : ermittelt die ausgewählte IIC-Bus-Nr. und verzweigt in    *
//*                die entsprechende Routine für das Lesen vom IIC-Bus       *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

void IIC_RD(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

BOOL     Init_IIC;
UINT32   Error;
UINT8    Addr;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

      // Dut to AD read format is ||03 | 'i' | D2||, so 1st command should be 3, D2 is i2c-bus number(I2C0/1/2/x)
      // Or start from 5(D4 is I2C address, D5-Dn is the optional data to be written)!!!
   if ((commandPtr[0] < 3) || (commandPtr[0] == 4))
   {  // evaluate and confirm Error
      writeLong(resultPtr+4, ERROR_I2C | ERROR_COMMAND);
      return;
   }

   // Read Bit mask which is I2C adress of selected component
   Addr =  commandPtr[4] & 0xFE;

   // if 1st command is 03, it means initialized
   if (commandPtr[0] == 3)
      Init_IIC = TRUE;
   else
      Init_IIC = FALSE;
//printf("@@@@@@@@@@@@@@@@@@@@@@1st COMMAND IS: %d\n", commandPtr[0]);   //Is 6 for Default 06 i 00 02 90 00
   // Default Error
   Error = NO_ERROR;

   switch (commandPtr[2])
   {
#if CP_IIC0
      case 0:     if (!Init_IIC)
                    //                       Addr     *WR_Buf       WR_Length         *RD_Buf        RD_Length   Mode
                    Error = IIC_Read_0_Str(Addr, commandPtr+5, commandPtr[0]-5, resultPtr+8, commandPtr[3], 0);                    
                  else
                    IIC_Init_0();
                  break;
#endif
#if CP_IIC1
      case 1:     
                  printf("Here is the IIC1 function\n");

                  if (!Init_IIC)
                     //                     Addr          *WR_Buf      WR_Length       *RD_Buf        RD_Length     Mode
                     Error = IIC_Read_1_Str(Addr, commandPtr+5, commandPtr[0]-5, resultPtr+8, commandPtr[3], 0);
                  else
                     IIC_Init_1();
                  break;
#endif
#if CP_IIC2
      case 2:     if (!Init_IIC)
                     //                     Addr          *WR_Buf      WR_Length       *RD_Buf        RD_Length     Mode
                     Error = IIC_Read_2_Str(Addr, commandPtr+5, commandPtr[0]-5, resultPtr+8, commandPtr[3], 0);
                  else
                     IIC_Init_2();
                  break;
#endif
#if CP_IIC3
      case 3:     if (!Init_IIC)
                     //                     Addr          *WR_Buf      WR_Length       *RD_Buf        RD_Length     Mode
                     Error = IIC_Read_3_Str(Addr, commandPtr+5, commandPtr[0]-5, resultPtr+8, commandPtr[3], 0);
                  else
                     IIC_Init_3();
                  break;
#endif
#if CP_IIC4
      case 4:     if (!Init_IIC)
                     //                     Addr          *WR_Buf      WR_Length       *RD_Buf        RD_Length     Mode
                     Error = IIC_Read_4_Str(Addr, commandPtr+5, commandPtr[0]-5, resultPtr+8, commandPtr[3], 0);
                  else
                     IIC_Init_4();
                  break;
#endif
#if CP_IIC5
      case 5:     if (!Init_IIC)
                     //                     Addr          *WR_Buf      WR_Length       *RD_Buf        RD_Length     Mode
                     Error = IIC_Read_5_Str(Addr, commandPtr+5, commandPtr[0]-5, resultPtr+8, commandPtr[3], 0);
                  else
                     IIC_Init_5();
                  break;
#endif
#if CP_IIC6
      case 6:     if (!Init_IIC)
                     //                     Addr          *WR_Buf      WR_Length       *RD_Buf        RD_Length     Mode
                     Error = IIC_Read_6_Str(Addr, commandPtr+5, commandPtr[0]-5, resultPtr+8, commandPtr[3], 0);
                  else
                     IIC_Init_6();
                  break;
#endif
#if CP_IIC7
      case 7:     if (!Init_IIC)
                     //                     Addr          *WR_Buf      WR_Length       *RD_Buf        RD_Length     Mode
                     Error = IIC_Read_7_Str(Addr, commandPtr+5, commandPtr[0]-5, resultPtr+8, commandPtr[3], 0);
                  else
                     IIC_Init_7();
                  break;
#endif
#if CP_IIC10
      case 10:    if (!Init_IIC)
                     //                     Addr          *WR_Buf      WR_Length       *RD_Buf        RD_Length     Mode
                     Error = IIC_Read_10_Str(Addr, commandPtr+5, commandPtr[0]-5, resultPtr+8, commandPtr[3], 0);
                  else
                     IIC_Init_10();
                  break;
#endif
#if CP_IIC11
      case 11:    if (!Init_IIC)
                     //                     Addr          *WR_Buf      WR_Length       *RD_Buf        RD_Length     Mode
                     Error = IIC_Read_11_Str(Addr, commandPtr+5, commandPtr[0]-5, resultPtr+8, commandPtr[3], 0);
                  else
                     IIC_Init_11();
                  break;
#endif

      default:    // Error auswerten und rückmelden
                  Error = ERROR_I2C | ERROR_I2C_WRONG_CH;
                  break;
   }

   if ((!Init_IIC) && (Error == NO_ERROR))
      // Insert result - length ; otherwise the default length applies 8, it can be inference from P22
      resultPtr [0] = commandPtr[3] + 8;

   // evaluate and confirm Error
   writeLong(resultPtr+4, Error);

} // end of function


//****************************************************************************
//* Function     : IIC_WR                                                    *
//****************************************************************************
//* Description  : ermittelt die ausgewählte IIC-Bus-Nr. und verzweigt in    *
//*                die entsprechende Routine für das Schreiben auf IIC-Bus   *
//*                                                                          *
//* Parameter    :                                                           *
//*                                                                          *
//* Returnvalue  :                                                           *
//*                                                                          *
//* Changed Var. :                                                           *
//*                                                                          *
//* Comment      :                                                           *
//*--------------------------------------------------------------------------*
//* Quality      :     ( ) not tested  ( ) partly tested  (x) fully tested   *
//****************************************************************************

void IIC_WR(void)
{
//*--------------------------------------------------------------------------*
//* Local variables                                                          *
//*--------------------------------------------------------------------------*

BOOL     Init_IIC;
UINT8    Mode;
UINT32   Error;
UINT8    Addr;

//*--------------------------------------------------------------------------*
//* Start of function                                                        *
//*--------------------------------------------------------------------------*

   // Check IIC COMMAND LENGTH AND FORMAT
   if ((commandPtr[0] < 3) || ((commandPtr[0] > 3) && ((commandPtr[0]-4) != commandPtr[3])))
   {  writeLong(resultPtr+4, ERROR_I2C | ERROR_COMMAND);
      return;
   }

   // Read Bit Mask
   Addr =  commandPtr[4] & 0xFE;

   if (commandPtr[0] == 3)       // It's initialize for IIC
      Init_IIC = TRUE;
   else
      Init_IIC = FALSE;

   // to the stop bit to be suppressed ???
   if ((commandPtr[2] >= 0x10) && (commandPtr[2] <= 0x1F))
   {  Mode = 0x10;
      // clean channel number
      commandPtr[2] &= 0x0F;
   }
   else
      Mode = 0;

   // Default Error
   Error = NO_ERROR;

   switch (commandPtr[2])
   {
#if CP_IIC0
      case 0:     if (!Init_IIC)
                     //                      Addr          *WR_Buf      WR_Length
                     Error = IIC_Write_0_Str(Addr, commandPtr+5, commandPtr[3]-1, Mode);
                  else
                     IIC_Init_0();
                  break;
#endif
#if CP_IIC1
      case 1:     if (!Init_IIC)
                     //                      Addr          *WR_Buf      WR_Length
                     Error = IIC_Write_1_Str(Addr, commandPtr+5, commandPtr[3]-1, Mode);
                  else
                     IIC_Init_1();
                  break;
#endif
#if CP_IIC2
      case 2:     if (!Init_IIC)
                     //                      Addr          *WR_Buf      WR_Length
                     Error = IIC_Write_2_Str(Addr, commandPtr+5, commandPtr[3]-1, Mode);
                  else
                     IIC_Init_2();
                  break;
#endif
#if CP_IIC3
      case 3:     if (!Init_IIC)
                     //                      Addr          *WR_Buf      WR_Length
                     Error = IIC_Write_3_Str(Addr, commandPtr+5, commandPtr[3]-1, Mode);
                  else
                     IIC_Init_3();
                  break;
#endif
#if CP_IIC4
      case 4:     if (!Init_IIC)
                     //                      Addr          *WR_Buf      WR_Length
                     Error = IIC_Write_4_Str(Addr, commandPtr+5, commandPtr[3]-1, Mode);
                  else
                     IIC_Init_4();
                  break;
#endif
#if CP_IIC5
      case 5:     if (!Init_IIC)
                     //                      Addr          *WR_Buf      WR_Length
                     Error = IIC_Write_5_Str(Addr, commandPtr+5, commandPtr[3]-1, Mode);
                  else
                     IIC_Init_5();
                  break;
#endif
#if CP_IIC6
      case 6:     if (!Init_IIC)
                     //                      Addr          *WR_Buf      WR_Length
                     Error = IIC_Write_6_Str(Addr, commandPtr+5, commandPtr[3]-1, Mode);
                  else
                     IIC_Init_6();
                  break;
#endif
#if CP_IIC7
      case 7:     if (!Init_IIC)
                     //                      Addr          *WR_Buf      WR_Length
                     Error = IIC_Write_7_Str(Addr, commandPtr+5, commandPtr[3]-1, Mode);
                  else
                     IIC_Init_7();
                  break;
#endif
#if CP_IIC10
      case 10:    if (!Init_IIC)
                     //                      Addr          *WR_Buf      WR_Length
                     Error = IIC_Write_10_Str(Addr, commandPtr+5, commandPtr[3]-1, Mode);
                  else
                     IIC_Init_10();
                  break;
#endif
#if CP_IIC11
      case 11:    if (!Init_IIC)
                     //                      Addr          *WR_Buf      WR_Length
                     Error = IIC_Write_11_Str(Addr, commandPtr+5, commandPtr[3]-1, Mode);
                  else
                     IIC_Init_11();
                  break;
#endif
      default:    Error = ERROR_I2C | ERROR_I2C_WRONG_CH;
                  break;
   }

   // Error auswerten und rückmelden
   writeLong(resultPtr+4, Error);

} // end of function


#endif // #if (CP_IIC0 || CP_IIC1 || CP_IIC2 || CP_IIC3 || CP_IIC4 || CP_IIC5 || CP_IIC6 || CP_IIC7 || CP_IIC10 || CP_IIC11)



