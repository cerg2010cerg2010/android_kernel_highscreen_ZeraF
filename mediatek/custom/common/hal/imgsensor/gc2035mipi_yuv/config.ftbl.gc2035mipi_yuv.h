/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to MediaTek Inc. and/or its licensors. Without
 * the prior written permission of MediaTek inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of MediaTek Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 * 
 * MediaTek Inc. (C) 2010. All rights reserved.
 * 
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE
 * MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek
 * Software") have been modified by MediaTek Inc. All revisions are subject to
 * any receiver's applicable license agreements with MediaTek Inc.
 */

//#ifndef _MTK_CUSTOM_PROJECT_HAL_IMGSENSOR_SRC_CONFIGFTBL__H_
//#define _MTK_CUSTOM_PROJECT_HAL_IMGSENSOR_SRC_CONFIGFTBL__H_
#if 1
//


/*******************************************************************************
 *
 ******************************************************************************/
FTABLE_DEFINITION(SENSOR_DRVNAME_GC2035MIPI_YUV)
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
FTABLE_SCENE_INDEP()
    //==========================================================================
#if 1
    //  Scene Mode
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_SCENE_MODE), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::SCENE_MODE_AUTO), 
            ITEM_AS_VALUES_(
                MtkCameraParameters::SCENE_MODE_AUTO,           
                MtkCameraParameters::SCENE_MODE_NIGHT,          
            )
        ), 
    )
#endif
    //==========================================================================
#if 1
    //  Effect
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_EFFECT), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::EFFECT_NONE), 
            ITEM_AS_VALUES_(
                MtkCameraParameters::EFFECT_NONE,       
                MtkCameraParameters::EFFECT_MONO,   
                MtkCameraParameters::EFFECT_NEGATIVE,
                MtkCameraParameters::EFFECT_SEPIA,      
            )
        ), 
    )
#endif
    //====MtkCameraParameters::EFFECT_SEPIA_BLUE, ======================================================================
#if 1
    //  Picture Size
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_PICTURE_SIZE), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_("1600x1200"), 
            ITEM_AS_VALUES_(
               /***5  : 3   ********4  : 3****/
               "320x192",      "320x240",        //  QVGA         76800
               "640x384",      "640x480",        //  HVGA         307200
               "1280x768",     "1024x768",       //  1 M          786432
               "1520x912",     "1280x960",       //  1.3M         1228800
               "1840x1104",    "1600x1200",      //  2M           1920000
               "2240x1344",    "2048x1536",      //  3M           3145728
               "2880x1728",    "2560x1920",      //  5M
            )
        ), 
    )
#endif
    //==========================================================================
#if 1
    //  Preview Size
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_PREVIEW_SIZE), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_("640x480"), 
            ITEM_AS_VALUES_(
                "176x144",      "320x240",      "352x288",      "480x320",      "480x368", 
                "640x480",      "720x480",      "800x480",      "800x600",      "864x480", 
                "960x540",      "1280x720",     "1920x1080", 
            )
        ), 
    )
#endif
    //==========================================================================
#if 1
    //  Video Size
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_VIDEO_SIZE), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_("640x480"), 
            ITEM_AS_VALUES_(
                "176x144",      "480x320",      "640x480", 
                "864x480",      "1280x720",     "1920x1080", 
            )
        ), 
    )
#endif
    //==========================================================================
#if 1
    //  Preview Frame Rate Range
    FTABLE_CONFIG_AS_TYPE_OF_USER(
        KEY_AS_(MtkCameraParameters::KEY_PREVIEW_FPS_RANGE), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_("30000,30000"), 
            ITEM_AS_USER_LIST_(
                "(15000,15000)", 
                "(30000,30000)", 
            )
        ), 
    )
#endif
    //==========================================================================
#if 1
    //  Exposure Compensation
    FTABLE_CONFIG_AS_TYPE_OF_USER(
        KEY_AS_(MtkCameraParameters::KEY_EXPOSURE_COMPENSATION), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_("0"), 
            ITEM_AS_USER_LIST_(
                "-1",       //min exposure compensation index
                "1",        //max exposure compensation index
                "1.0",      //exposure compensation step; EV = step x index
            )
        ), 
    )
#endif
    //==========================================================================
#if 1
    //  Anti-banding (Flicker)
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_ANTIBANDING), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::ANTIBANDING_OFF), 
            ITEM_AS_VALUES_(
                MtkCameraParameters::ANTIBANDING_OFF, 
                MtkCameraParameters::ANTIBANDING_50HZ, 
                MtkCameraParameters::ANTIBANDING_60HZ, 
//                MtkCameraParameters::ANTIBANDING_AUTO, 
            )
        ), 
    )
#endif
    //==========================================================================
#if 1
    //  Video Stabilization (EIS)
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_SUPPORTED(
        KEY_AS_(MtkCameraParameters::KEY_VIDEO_STABILIZATION), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::FALSE), 
            ITEM_AS_SUPPORTED_(
            #if 1
                MtkCameraParameters::FALSE
            #else
                MtkCameraParameters::TRUE
            #endif
            )
        ), 
    )
#endif
    //==========================================================================
#if 1
    //  Zoom
    FTABLE_CONFIG_AS_TYPE_OF_USER(
        KEY_AS_(MtkCameraParameters::KEY_ZOOM), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_("0"),  //Zoom Index
            ITEM_AS_USER_LIST_(
                //Zoom Ratio
                "100", "114", "132", "151", "174", 
                "200", "229", "263", "303", "348", 
                "400", 
            )
        ), 
    )
#endif
    //==========================================================================
#if 0
    //  Zsd
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_ZSD_MODE), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::OFF), 
            ITEM_AS_VALUES_(
                MtkCameraParameters::OFF, 
            )
        ), 
    )
#endif
    //==========================================================================
#if 1
    //  (Shot) Capture Mode
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_CAPTURE_MODE), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::CAPTURE_MODE_NORMAL), 
            ITEM_AS_VALUES_(
                MtkCameraParameters::CAPTURE_MODE_NORMAL,
                MtkCameraParameters::CAPTURE_MODE_CONTINUOUS_SHOT,
                MtkCameraParameters::CAPTURE_MODE_SMILE_SHOT,
                MtkCameraParameters::CAPTURE_MODE_AUTO_PANORAMA_SHOT,
            )
        ), 
    )
#endif
    //==========================================================================
END_FTABLE_SCENE_INDEP()
//------------------------------------------------------------------------------
/*******************************************************************************
 *
 ******************************************************************************/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
FTABLE_SCENE_DEP()
    //==========================================================================
#if 1
    //  Focus Mode
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_FOCUS_MODE), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::FOCUS_MODE_INFINITY), 
            ITEM_AS_VALUES_(
              //  MtkCameraParameters::FOCUS_MODE_AUTO,   
                MtkCameraParameters::FOCUS_MODE_INFINITY, 
                MtkCameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE, 
                MtkCameraParameters::FOCUS_MODE_CONTINUOUS_VIDEO, 
            )
        ), 
        //......................................................................
    )
#endif
    //==========================================================================
#if 0
    //  ISO
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_ISO_SPEED), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_("auto"), 
            ITEM_AS_VALUES_(
                "auto", 
            )
        ), 
        //......................................................................
    )
#endif
    //==========================================================================
#if 1
    //  White Balance.
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_WHITE_BALANCE), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::WHITE_BALANCE_AUTO), 
            ITEM_AS_VALUES_(
                MtkCameraParameters::WHITE_BALANCE_AUTO,            MtkCameraParameters::WHITE_BALANCE_INCANDESCENT, 
                MtkCameraParameters::WHITE_BALANCE_DAYLIGHT,        MtkCameraParameters::WHITE_BALANCE_CLOUDY_DAYLIGHT, 
                MtkCameraParameters::WHITE_BALANCE_FLUORESCENT,     MtkCameraParameters::WHITE_BALANCE_TUNGSTEN, 
            )
        ), 
        //......................................................................
    )
#endif
    //==========================================================================
#if 0
    //  ISP Edge
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_EDGE), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::MIDDLE), 
            ITEM_AS_VALUES_(
                MtkCameraParameters::LOW, MtkCameraParameters::MIDDLE, MtkCameraParameters::HIGH, 
            )
        ), 
        //......................................................................
        //......................................................................
    )
#endif
    //==========================================================================
#if 0
    //  ISP Hue
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_HUE), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::MIDDLE), 
            ITEM_AS_VALUES_(
                MtkCameraParameters::LOW, MtkCameraParameters::MIDDLE, MtkCameraParameters::HIGH, 
            )
        ), 
        //......................................................................
        //......................................................................
    )
#endif
    //==========================================================================
#if 0
    //  ISP Saturation
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_SATURATION), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::MIDDLE), 
            ITEM_AS_VALUES_(
                MtkCameraParameters::LOW, MtkCameraParameters::MIDDLE, MtkCameraParameters::HIGH, 
            )
        ), 
        //......................................................................
        //......................................................................
    )
#endif
    //==========================================================================
#if 0
    //  ISP Brightness
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_BRIGHTNESS), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::MIDDLE), 
            ITEM_AS_VALUES_(
                MtkCameraParameters::LOW, MtkCameraParameters::MIDDLE, MtkCameraParameters::HIGH, 
            )
        ), 
        //......................................................................
        //......................................................................
    )
#endif
    //==========================================================================
#if 0
    //  ISP Contrast
    FTABLE_CONFIG_AS_TYPE_OF_DEFAULT_VALUES(
        KEY_AS_(MtkCameraParameters::KEY_CONTRAST), 
        SCENE_AS_DEFAULT_SCENE(
            ITEM_AS_DEFAULT_(MtkCameraParameters::MIDDLE), 
            ITEM_AS_VALUES_(
                MtkCameraParameters::LOW, MtkCameraParameters::MIDDLE, MtkCameraParameters::HIGH, 
            )
        ), 
        //......................................................................
        //......................................................................
    )
#endif
    //==========================================================================
END_FTABLE_SCENE_DEP()
//------------------------------------------------------------------------------
END_FTABLE_DEFINITION()


/*******************************************************************************
 *
 ******************************************************************************/
#endif
//#endif //_MTK_CUSTOM_PROJECT_HAL_IMGSENSOR_SRC_CONFIGFTBL__H_

