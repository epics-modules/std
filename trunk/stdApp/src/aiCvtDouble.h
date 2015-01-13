/* $Id: aiCvtDouble.h,v 1.1.1.1 2001-07-03 20:05:25 sluiter Exp $ */

/* DISCLAIMER: This software is provided `as is' and without _any_ kind of
 *             warranty. Use it at your own risk - I won't be responsible
 *			   if your dog drowns as a consequence of using my software
 *			   blah & blah...
 */

/*
 * The aiCvtDouble() routine is an exact copy of the conversion routine of
 * the AnalogIn record.
 * What I don't understand about this (and other records) is the fact that
 * the `raw' value which is what the built in conversion operates on is
 * an integer i.e. `raw values' of the `analog' (sic!) type of records are
 * in fact integers at the device support layer.
 * IMHO, it would have made more sense to do the double <-> int casting
 * in the device support layer, because only the device support module 
 * knows if the device provides ints or floats.
 *
 * What if the quantity read from the device is a floating point
 * value and we still want to use the conversion feature? No way :-(
 * 
 * Therefore, I copied the conversion code, so it can be explicitely called by
 * device support's read_ai() routine after getting the (double) value from
 * the device. read_ai() should then return 2 to flag that no further conversion
 * should be done. 
 *
 * Author: Till Straumann (PTB, 1999)
 *
 * $Log: not supported by cvs2svn $
 * Revision 1.3  1999/04/20 19:25:17  strauman
 * *** empty log message ***
 *
 * Revision 1.2  1999/04/20 10:52:09  strauman
 *  - added comment
 *
 */
#ifndef AI_CVT_DOUBLE_H
#define AI_CVT_DOUBLE_H

void aiCvtDouble(aiRecord *prec);

#endif
