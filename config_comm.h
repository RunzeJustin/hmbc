// nr of simultaneous operational CMD and gateway interfaces
#define COMM_INTERFACES 4

// number of buffers (consider 4 buffers for the system + 2*fifDepth per interface
// must be a multiple of 2
// 0x40: 10 interfaces @ depth of 3    |   7 interfaces @ depth of 4
// 0x20:  4 interfaces @ depth of 3    |   3 interfaces @ depth of 4
// 0x10:  2 interfaces @ depth of 3    |   1 interfaces @ depth of 4
#define COMM_FREE_SIZE 0x40

// default for fifoDepth (if 0), can be overwritten during msgRegister()
#define COMM_BUF_DEPTH 3

// Define these if the main loop should announce liveness via a status LED
//#define COMM_LED_STATUS_CHANNEL	0	      // LED channel to use
#define COMM_LED_STATUS_COUNT	   10000U	// adjusts frequency

// communication log feature and cmd enable (disable only, when code size is a real issue!)
#define COMM_CMD_ENABLE   1   // switch on statistics
#define COMM_ERROR_GEN    0   // needs (COMM_CMD_ENABLE == 1)
#define COMM_LOG_ENABLE   1   // 2 ihe  // needs (COMM_CMD_ENABLE == 1); 2 tiny logging only
#define COMM_GATEWAY      1
//#define COMM_QUEUE_ASCII  1

// enables plausibility check for msgRegister() and clears unused queue entries with "0xDEADBEEF"
// disable, when you system is running
#define COMM_DEBUG        0

// define address for startup logging; needs (COMM_CMD_ENABLE == 1) and (COMM_LOG_ENABLE == 1)
#define COMM_STARTUP_LOGGING        0xFEDF0000
#define COMM_STARTUP_LOGGING_SIZE   0x00000F00  // stay 0x100 under real limit for safety


#define COMM_JPRINTF def
//#define COMM_DPRINTF def
// alternative synchronisation mechanism for queues (for test only)
//#define COMM_SYNC_QUEUE def

// set to 1 if the system is using cthread
#define COMM_USE_CTHREAD 0
// define lock/unlock operation to secure atomic operations
// for SH, ARM or Intel this is done with a c_thread mutex
// for DSPs you can use LCK_pend() or LCK_post
// COMM_ATOM_INIT can be used to initialize the synchonization function during msgInit()
#define COMM_ATOM_INIT
#define COMM_ATOM_LOCK     __DI()
#define COMM_ATOM_UNLOCK   __EI()


//-------------------------------------------------------------------------
//  Misc
//-------------------------------------------------------------------------

// Define these if the main loop should announce liveness via a status LED
#define SW_LED          0                      // switch for status LED's

// external gateway command interpreter
// undef if not used
#define COMM_GATEWAY_USR_INTERPRETER     {Gateway_Interpreter ();return(true);}

// yellow Led especially for UMOST II
// undef if not used
#undef COMM_GATEWAY_USR_STATUSLED

