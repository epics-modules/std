/* devScalerAsyn.h

    Author: Mark Rivers
    22-Nov-2006

    This is device support for the scaler record with asyn drivers.
  
*/

/* These are the strings that device support passes to drivers via the asynDrvUser interface.
 * Drivers must return a value in pasynUser->reason that is unique for that command.
 */

#define SCALER_RESET_COMMAND_STRING       "SCALER_RESET"        /* int32, write */
#define SCALER_CHANNELS_COMMAND_STRING    "SCALER_CHANNELS"     /* int32, read */
#define SCALER_READ_COMMAND_STRING        "SCALER_READ"         /* int32Array, read */
#define SCALER_READ_SINGLE_COMMAND_STRING "SCALER_READ_SINGLE"  /* int32, read */
#define SCALER_PRESET_COMMAND_STRING      "SCALER_PRESET"       /* int32, write */
#define SCALER_ARM_COMMAND_STRING         "SCALER_ARM"          /* int32, write */
#define SCALER_DONE_COMMAND_STRING        "SCALER_DONE"         /* int32, read */

#define MAX_SCALER_COMMANDS 7
