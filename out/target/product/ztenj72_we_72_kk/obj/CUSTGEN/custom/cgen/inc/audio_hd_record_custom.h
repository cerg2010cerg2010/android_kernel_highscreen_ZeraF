/*******************************************************************************
 *
 * Filename:
 * ---------
 * aud_hd_record_custom.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 * This file is the header of audio customization related function or definition.
 *
 * Author:
 * -------
 * Charlie Lu.
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#ifndef AUDIO_HD_RECORD_CUSTOM_H
#define AUDIO_HD_RECORD_CUSTOM_H

//hd_rec_fir
/* 0: Input FIR coefficients for Normal mode */
/* 1: Input FIR coefficients for Headset mode */
/* 2: Input FIR coefficients for Handfree mode */
/* 3: Input FIR coefficients for BT mode */
/* 4: Input FIR coefficients for Normal mode */
/* 5: Input FIR coefficients for Handfree mode */
/* 6: Input FIR coefficients for Normal mode 2nd Mic*/
/* 7: Input FIR coefficients for Voice Recognition*/
/* 8: Input FIR coefficients for Voice Unlock Main Mic */
/* 9: Input FIR coefficients for Voice Unlock 2nd Mic */
/* 10: Input FIR coefficients for 1st customization Main Mic*/
/* 11: Input FIR coefficients for 1st customization 2nd Mic*/
/* 12: Input FIR coefficients for 2nd customization Main Mic*/
/* 13: Input FIR coefficients for 2nd customization 2nd Mic*/
/* 14: Input FIR coefficients for 3rd customization Main Mic*/
/* 15: Input FIR coefficients for 3rd customization Ref Mic*/

#if defined(MTK_AUDIO_BLOUD_CUSTOMPARAMETER_V3) //for old version
#define DEFAULT_HD_RECORD_FIR_Coeff \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0

#else
#define DEFAULT_HD_RECORD_FIR_Coeff \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0,  \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
    0,     0,     0,     0,     0,     0,     0,     0,     0,     0

#endif

#define DEFAULT_HD_RECORD_MODE_FIR_MAPPING_CH1 \
    7, 4, \
    0, 1, \
    0, 1, \
    0, 1, \
    0, 1, \
    0, 1, \
    8, 1, \
    10, 1, \
    12, 1, \
    14, 1, \
    0, 1, \
    0, 1, \
    0, 1, \
    0, 1, \
    0, 3

#define DEFAULT_HD_RECORD_MODE_FIR_MAPPING_CH2 \
    6, 6, \
    6, 6, \
    6, 6, \
    6, 6, \
    6, 6, \
    6, 6, \
    9, 6, \
    11, 6, \
    13, 6, \
    15, 6, \
    6, 6, \
    6, 6, \
    6, 6, \
    6, 6, \
    6, 3

#endif
