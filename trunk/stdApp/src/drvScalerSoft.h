/* File:    drvScalerSoft.h
 * Author:  Mark Rivers
 * Date:    22-May-2007
 *
 * Purpose: 
 * This module provides the driver support for scaler using database links as the inputs.
 */


int drvScalerSoftConfigure(char  *portName,
                           int   maxChans,
                           char  *pvTemplate);
