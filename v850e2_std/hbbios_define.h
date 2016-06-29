/*
//-------------------------------------------------------------------------
//
//        _____   _____   _   _    ___
//       |  ___| |  ___| | | | |  |   `
//       | |_    | |_    | |_| |  | [] |
//       |  _|   |  _|   |  _  |  |  _ <
//       | |___  | |___  | | | |  | |_| |
//       |_____| |_____| |_| |_|  |____/
//
//
//   Project:          H/B BIOS -- fundamental definitions
//   (c) copyright     Harman-Becker Automotive Systems
//-----------------------------------------------------------------------
//   $Header: //BiosGroup/trunk/BC_Projects/0000/0000-0000-00XY/cpu/src/sh_std/define.h#12 $
//   $Change: 946377 $
//-----------------------------------------------------------------------
*/

#ifndef _hbbios_define_h
#define _hbbios_define_h

/*
**  Interface channels. Must be consistent between all BIOSs, BiosControl!
*/
typedef enum
{
   ifRS232   = 0x00,
   ifRS232b  = 0x01,
   ifMOST    = 0x10,
   ifIPC     = 0x20,
   ifI2C     = 0x30,
   ifCAN     = 0x40,
   ifJTAG    = 0x50,
   ifScript  = 0x60,
   ifSalieri = 0x70,
   ifSPI     = 0x80,
   ifShrdMem = 0x90,
   ifDSPIPC  = 0x98,
   ifTCP     = 0xa0,
   ifUSBdev  = 0xb0,
   ifNone    = 0xFF
} InterfaceType;

// Interfaces
#define MSG_LENGTH_MAX     256U
#define IF_MAXCOUNT        13U  /* ifRS232 .. ifUSBdev */

// handshaking characters -------------------------------------------------
// (to be used with DLE/STXprotocol on serial ports)

#define NUL       0x00U
#define STX       0x02U
#define ETX       0x03U
#define ACK       0x06U
#define DLE       0x10U
#define NAK       0x15U

/*
**  Memory access types. Must be consistent between BIOS, BiosControl.
*/
#define MEM_CACHEABLE     0x10U
#define MEM_READ          0x04U
#define MEM_WRITE         0x02U
#define MEM_EXECUTE       0x01U



/*
**  CPU types. Must be consistent between BIOS, BiosControl.
*/
#define CPU_80C161CI           0U
#define CPU_80C515C            1U
#define CPU_NEC_K0             2U
#define CPU_NEC_K4             3U
#define CPU_ST10F168           4U
#define CPU_ST10F269           5U
#define CPU_80C161CS           6U
#define CPU_80C161JC           7U
#define CPU_80C167             8U
#define CPU_SH7709A            9U
#define CPU_SH7709S           10U
#define CPU_SH7729            11U
#define CPU_SH7750            12U
#define CPU_SH7751R           13U
#define CPU_ST10F276          14U
#define CPU_ST10F280          15U
#define CPU_NEC_V850_DF3281Y  16U
#define CPU_NEC_V850_DF3288Y  17U
#define CPU_SH7780            18U
#define CPU_SH7760            19U
#define CPU_NEC_V850_DF3231   20U
#define CPU_SH7785            21U
#define CPU_TI_DA710          22U
#define CPU_ARM9_GEN          23U
#define CPU_TI_DM644X_DSP     24U
#define CPU_TI_DM642          25U
#define CPU_TI_DA610          26U
#define CPU_NEC_V850_DF3366   27U
#define CPU_TI_DRA300         28U
//#define CPU_TI_DRA446_ARM9    29U      // use CPU:ARM9_GEN + SOC-defines below
#define CPU_TI_DRA44X_DSP     30U      // Jacinto DSP Core
#define CPU_NEC_V850_DF3365   31U
#define CPU_NEC_V850_DF3239   32U
#define CPU_NEC_V850_DF3370   33U
#define CPU_NEC_V850_DF3371   34U
#define CPU_IA32_PENTIUM      35U
#define CPU_IA32_POULSBO          CPU_IA32_PENTIUM
#define CPU_IA32_TUNNEL_CREEK     CPU_IA32_PENTIUM
#define CPU_NEC_V850_DF3376   36U
#define CPU_NEC_V850_DF3375   37U
#define CPU_NEC_V850_DF3479   38U
#define CPU_TI_DA8X0_DSP      39U
#define CPU_NEC_V850_DF3476   40U
#define CPU_SH7786            41U
#define CPU_NEC_V850_DF3335   42U
#define CPU_TI_DRA6XX_DSP     43U
#define CPU_NV_ESOC3          44U
#define CPU_ATMEGA164         45U
#define CPU_SHARC21469        46U
#define CPU_SHARC21479        47U
#define CPU_NEC_V850_DF3488   48U
#define CPU_NEC_V850_DF3351   49U
#define CPU_NEC_V850E2_DF3558 50U
#define CPU_NEC_V850_DF3629   51U
#define CPU_SHARC21489        52U
#define CPU_NEC_V850E2_DF3580 53U
#define CPU_NEC_V850E2_DF3554 54U

#define CPU_UNKNOWN           255U

/*
**  CPU_SOC needed to handle common modules on multicore chips like Jacinto
*/
#define CPU_SOC_TI_DRA446     0U
#define CPU_SOC_TI_DRA443     1U
#define CPU_SOC_TI_DM6446     2U
#define CPU_SOC_TI_DM6437     3U    // DM6446 without ARM
#define CPU_SOC_TI_DA810      4U    // DA830 without ARM
#define CPU_SOC_TI_DA830      5U
#define CPU_SOC_TI_DRA6XX     6U

/*
**  CPU_FAMILY needed to handle compiler specific differences
*/
#define CPU_FAMILY_SH         0U
#define CPU_FAMILY_C6000      1U
#define CPU_FAMILY_ARM9       2U
#define CPU_FAMILY_IA32       3U
#define CPU_FAMILY_NECV850    4U

/*
**  CPU_SUB_FAMILY needed to handle includes for cpu specific differences within cpu families
*/
#define CPU_SUB_FAMILY_NONE   0U
#define CPU_SUB_FAMILY_SH3    1U
#define CPU_SUB_FAMILY_SH4    2U


#if ((CPU_TYPE == CPU_SH7709A) || (CPU_TYPE == CPU_SH7709S) || (CPU_TYPE == CPU_SH7729))
#define CPU_FAMILY        CPU_FAMILY_SH
#define CPU_SUB_FAMILY    CPU_SUB_FAMILY_SH3
#endif

#if ((CPU_TYPE == CPU_SH7785)  || (CPU_TYPE == CPU_SH7780)  || (CPU_TYPE == CPU_SH7760) ||   \
     (CPU_TYPE == CPU_SH7750)  || (CPU_TYPE == CPU_SH7751R) || (CPU_TYPE == CPU_SH7729) ||   \
     (CPU_TYPE == CPU_SH7786))
#define CPU_FAMILY        CPU_FAMILY_SH
#define CPU_SUB_FAMILY    CPU_SUB_FAMILY_SH4
#endif

#if ((CPU_TYPE == CPU_TI_DA610) || (CPU_TYPE == CPU_TI_DA710)      || (CPU_TYPE == CPU_TI_DRA300)     ||     \
     (CPU_TYPE == CPU_TI_DM642) || (CPU_TYPE == CPU_TI_DM644X_DSP) || (CPU_TYPE == CPU_TI_DRA44X_DSP) ||     \
     (CPU_TYPE == CPU_TI_DA8X0_DSP) || (CPU_TYPE == CPU_TI_DRA6XX_DSP) )

#define CPU_FAMILY CPU_FAMILY_C6000
#endif

#if (CPU_TYPE == CPU_ARM9_GEN)
#define CPU_FAMILY CPU_FAMILY_ARM9
#endif

#if ((CPU_TYPE == CPU_IA32_PENTIUM) || (CPU_TYPE == CPU_IA32_POULSBO) || (CPU_TYPE == CPU_IA32_TUNNEL_CREEK))
#define CPU_FAMILY CPU_FAMILY_IA32
#endif

#if ((CPU_TYPE == CPU_NEC_V850_DF3281Y) || (CPU_TYPE == CPU_NEC_V850_DF3288Y) || (CPU_TYPE == CPU_NEC_V850_DF3231)   ||   \
     (CPU_TYPE == CPU_NEC_V850_DF3366)  || (CPU_TYPE == CPU_NEC_V850_DF3365)  || (CPU_TYPE == CPU_NEC_V850_DF3239)   ||   \
     (CPU_TYPE == CPU_NEC_V850_DF3375)  || (CPU_TYPE == CPU_NEC_V850_DF3479)  || (CPU_TYPE == CPU_NEC_V850_DF3476)   ||   \
     (CPU_TYPE == CPU_NEC_V850_DF3370)  || (CPU_TYPE == CPU_NEC_V850_DF3371)  || (CPU_TYPE == CPU_NEC_V850_DF3376)   ||   \
     (CPU_TYPE == CPU_NEC_V850_DF3335)  || (CPU_TYPE == CPU_NEC_V850_DF3488)  || (CPU_TYPE == CPU_NEC_V850E2_DF3558) ||   \
     (CPU_TYPE == CPU_NEC_V850_DF3629)  || (CPU_TYPE == CPU_NEC_V850E2_DF3580) || (CPU_TYPE == CPU_NEC_V850E2_DF3554))
#define CPU_FAMILY CPU_FAMILY_NECV850
#endif


/*
** BIOS status and mode register; must be consistent w/ BiosControl.
*/
#define STATUS_INIT     0             // Status BIOS-Initialisation
#define STATUS_RUNNING  1             // Status BIOS running
#define STATUS_DIAG     2             // Status BIOS Diagnostics

//                     Size in Bits   Location within MODE-Register
#define MODE_TRM_SIZE   2U
#define MODE_TRM                      0U
#define MODE_CM_SIZE    2U
#define MODE_CM                       2U
#define MODE_EM_SIZE    1U
#define MODE_EM                       4U
#define MODE_SA_SIZE    1U
#define MODE_SA                       5U
#define MODE_IC_SIZE    1U
#define MODE_IC                       6U
#define MODE_SD_SIZE    1U
#define MODE_SD                       7U
#define MODE_WKU_SIZE   1U
#define MODE_WKU                      8U
#define MODE_FE_SIZE    1U
#define MODE_FE                       16U
#define MODE_STOP_SIZE  1U
#define MODE_STOP                     17U
#define MODE_FST_SIZE   2U
#define MODE_FST                      18U
#define MODE_ZIP_SIZE   1U
#define MODE_ZIP                      20U
#define MODE_SER0_SIZE  1U
#define MODE_SER0                     21U
#define MODE_SER1_SIZE  1U
#define MODE_SER1                     22U
#define MODE_SER2_SIZE  1U
#define MODE_SER2                     23U
#define MODE_VW_SIZE    1U
#define MODE_VW                       24U

#endif // ifndef _hbbios_define_h
