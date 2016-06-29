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



#ifndef _DF3558_DEFS_H
#define _DF3558_DEFS_H

//*--------------------------------------------------------------------------*
//* Businterface                                                             *
//*--------------------------------------------------------------------------*

// Register BSC
#define BS20     (1 << 4)
#define BS21     (1 << 5)

// Register DEC
#define DE2      (1 << 4)

// Register BCT0
#define BCT20    (1 << 8)
#define BCT21    (1 << 9)
#define ME2      (1 << 11)

// Register DWC0
#define DW20     (1 << 8)
#define DW21     (1 << 9)
#define DW22     (1 << 10)
#define DW23     (1 << 11)

// Register DHC
#define DH20     (1 << 4)
#define DH21     (1 << 5)

// Register DSC
#define DS20     (1 << 4)
#define DS21     (1 << 5)

// Register AWC0
#define ASW20    (1 << 8)
#define ASW21    (1 << 9)
#define AHW20    (1 << 10)
#define AHW21    (1 << 11)

// Register ICC0
#define RIC20    (1 << 8)
#define RIC21    (1 << 9)
#define WIC20    (1 << 10)
#define WIC21    (1 << 11)

// Register EWC
#define EW2      (1 << 4)

//*--------------------------------------------------------------------------*
//* A/D Converter                                                            *
//*--------------------------------------------------------------------------*

// Register ADCAnCTL0
#define ADCAnCE      (1 << 7)

// Register ADCAnCTL1
#define ADCAnTiETS(A) (A << 26)
#define ADCAnCRAC    (1 << 24)
#define ADCAnMD1     (1 << 21)
#define ADCAnMD0     (1 << 20)
#define ADCAnDISC    (1 << 17)
#define ADCAnRCL     (1 << 16)
#define ADCAnCTYP    (1 << 15)
#define ADCAnFR(A)   (A << 8)
#define ADCAnTRM2    (1 << 6)
#define ADCAnTRM1    (1 << 5)
#define ADCAnTRM0    (1 << 4)
#define ADCAnBPC     (1 << 3)
#define ADCAnGPS     (1 << 0)

// Register ADCAnTRGi
#define ADCAnSTTi    (1 << 0)

// Register ADCAnLCR
#define ADCAnLCN     (0x1F << 16)

//*--------------------------------------------------------------------------*
//* CAN                                                                      *
//*--------------------------------------------------------------------------*

// Register FCNnGMCLCTL
#define FCNnGMCLSESR    (1 << 12)
#define FCNnGMCLSORF    (1 << 4)
#define FCNnGMCLECCF    (1 << 5)
#define FCNnGMCLPWOM    (1 << 0)

// Register FCNnCMISCTL
#define FCNnCMISITSF0   (1 << 0)
#define FCNnCMISITSF1   (1 << 1)
#define FCNnCMISITSF2   (1 << 2)
#define FCNnCMISITSF3   (1 << 3)
#define FCNnCMISITSF4   (1 << 4)
#define FCNnCMISITSF5   (1 << 5)
#define FCNnCMISITSF6   (1 << 6)

// Register FCNnMmCTL
#define FCNnMmCLRY      (1 << 0)
#define FCNnMmCLTR      (1 << 1)
#define FCNnMmCLDN      (1 << 2)
#define FCNnMmCLIE      (1 << 3)
#define FCNnMmCLMW      (1 << 4)
#define FCNnMmCLNH      (1 << 6)

#define FCNnMmRDYF      (1 << 0)
#define FCNnMmTRQF      (1 << 1)
#define FCNnMmDTNF      (1 << 2)
#define FCNnMmMOWF      (1 << 4)
#define FCNnMmMUCF      (1 << 13)

// Register FCNnCMCLCTL
#define FCNnCMCLCLOP0   (1 << 0)
#define FCNnCMCLCLOP1   (1 << 1)
#define FCNnCMCLCLOP2   (1 << 2)
#define FCNnCMSECLPS0   (1 << 3)
#define FCNnCMSECLPS1   (1 << 4)
#define FCNnCMCLCLVL    (1 << 5)
#define FCNnCMCLCLAL    (1 << 6)
#define FCNnCMCLSERC    (1 << 15)

// Register FCNnMmSTRB
#define FCNnMmSSAM      (1 << 0)
#define FCNnMmSSMT0     (1 << 3)
#define FCNnMmSSMT1     (1 << 4)
#define FCNnMmSSMT2     (1 << 5)
#define FCNnMmSSMT3     (1 << 6)

// Register FCNnMmMID1H
#define FCNnMmSSIE      (1 << 15)

   // Register FCNnCMRGRX
#define FCNnCMRGRVFF    (1 << 0)
#define FCNnCMRGSSPM    (1 << 1)

#define FCNnCMRGCLRV    (1 << 0)

// Register FCNnCMINSTR
#define FCNnCMINBOFF    (1 << 4)

//*--------------------------------------------------------------------------*
//* Timer TAUA0                                                              *
//*--------------------------------------------------------------------------*

// Register TAUAnTPS
#define PRS0(A)         (A << 0)
#define PRS1(A)         (A << 4)
#define PRS2(A)         (A << 8)
#define PRS3(A)         (A << 12)


// Register TAUAnCMORm
#define MD(A)           (A << 0)
#define COS(A)          (A << 6)
#define STS(A)          (A << 8)
#define MAS             (1 << 11)
#define CCS(A)          (A << 12)
#define CKS(A)          ((A & 3) << 14)



#endif // _DF3558_DEFS_H

