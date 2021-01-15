/*
TEXAS INSTRUMENTS TEXT FILE LICENSE

Copyright (c) [2018] – [2019] Texas Instruments Incorporated

All rights reserved not granted herein.

Limited License.  

Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive license under copyrights and patents it now or hereafter owns or controls to make, have made, use, import, offer to sell and sell ("Utilize") this software subject to the terms herein.  With respect to the foregoing patent license, such license is granted  solely to the extent that any such patent is necessary to Utilize the software alone.  The patent license shall not apply to any combinations which include this software, other than combinations with devices manufactured by or for TI (“TI Devices”).  No hardware patent is licensed hereunder.

Redistributions must preserve existing copyright notices and reproduce this license (including the above copyright notice and the disclaimer and (if applicable) source code license limitations below) in the documentation and/or other materials provided with the distribution

Redistribution and use in binary form, without modification, are permitted provided that the following conditions are met:

*	No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any software provided in binary form.

*	any redistribution and use are licensed by TI for use only with TI Devices.

*	Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.

If software source code is provided to you, modification and redistribution of the source code are permitted provided that the following conditions are met:

*	any redistribution and use of the source code, including any resulting derivative works, are licensed by TI for use only with TI Devices.

*	any redistribution and use of any object code compiled from the source code and any resulting derivative works, are licensed by TI for use only with TI Devices.

Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or promote products derived from this software without specific prior written permission.

DISCLAIMER.

THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifndef __AAA_AWB_DEFS_H__
#define __AAA_AWB_DEFS_H__

#ifndef NULL
#define NULL    ((void *) 0)
#endif

#include <stdint.h>

#define NUM_OF_REF_1        30  //maximal number of reference 1, actual use 17                          
#define NUM_OF_REF_2        15  //maximal number of reference 2, actual use 7
#define NUM_OF_GRAY          4  //maximal number of gray 2 used, actual use 4
#define NUM_BLK_1        (NUM_OF_REF_1 * NUM_OF_GRAY)
#define NUM_BLK_2        (NUM_OF_REF_2 * NUM_OF_GRAY)
#define NUM_BLK_3        (1120)
#define AWB_SCRATCH_MEM_SIZE (4 + 26 * NUM_OF_REF_1 + 16 * NUM_BLK_1 + 13 * NUM_OF_REF_2 + 16 * NUM_BLK_2 + 6 * NUM_BLK_3)
#define AWB_AVG_BUF_LENGTH   6  //maximal length, actually used 6

typedef struct
{
    // Sensor specific calibration data
    uint32_t    blue_index;
    uint32_t    red_index;
    uint32_t    green1_index;
    uint32_t    green2_index;

    uint32_t    num_of_ref_1;
    uint32_t    num_of_ref_2;
    uint32_t    num_of_gray;
    uint32_t    radius; 

    uint16_t    color_temp_1[NUM_OF_REF_1];

    int16_t     wbReferenceCb[NUM_OF_REF_1][NUM_OF_GRAY];
    int16_t     wbReferenceCr[NUM_OF_REF_1][NUM_OF_GRAY];

    uint16_t    ref_gray_R_1[NUM_OF_REF_1][NUM_OF_GRAY];
    uint16_t    ref_gray_G_1[NUM_OF_REF_1][NUM_OF_GRAY];
    uint16_t    ref_gray_B_1[NUM_OF_REF_1][NUM_OF_GRAY];

    uint8_t     ref_index_2[NUM_OF_REF_2];
    uint16_t    color_temp_2[NUM_OF_REF_2];

    int16_t     referencesCb_2[NUM_OF_REF_2][NUM_OF_GRAY];
    int16_t     referencesCr_2[NUM_OF_REF_2][NUM_OF_GRAY];

    uint16_t    ref_gray_R_2[NUM_OF_REF_2][NUM_OF_GRAY];
    uint16_t    ref_gray_G_2[NUM_OF_REF_2][NUM_OF_GRAY];
    uint16_t    ref_gray_B_2[NUM_OF_REF_2][NUM_OF_GRAY];

    uint16_t    img_ref[NUM_OF_REF_2 * 1120];

    // Sensor specific tuning paramaters
    int32_t     luma_awb_min;
    int32_t     luma_awb_max;

    uint32_t    low_color_temp_thresh;
    uint32_t    apply_rgb_adjust;
    int32_t     R_adjust;
    int32_t     B_adjust;

    uint32_t    SB_1;
    uint32_t    SB_2;
    uint32_t    SB_low_bound;

    uint32_t    default_T_H;
    uint32_t    default_T_MH;
    uint32_t    default_T_ML;
    uint32_t    default_T_L;

    uint32_t    default_T_H_index;
    uint32_t    default_T_MH_index;
    uint32_t    default_T_ML_index;
    uint32_t    default_T_L_index;  

    uint32_t    best_gray_index_default;

} awb_calc_data_t;


/****************************************************************
* TYPES
****************************************************************/

typedef enum {
    AWB_WB_MODE_AUTO = 0,      // 0 Auto AWB mode
    AWB_WB_MODE_MANUAL = 1     // 1 Manual AWB mode
} AWB_MODE_VALUES;

typedef struct {
    uint16_t red;   // Average value for red pixels in current paxel
    uint16_t green; // Average value for green pixels in current paxel
    uint16_t blue;  // Average value for blue pixels in current paxel
    uint16_t ext;
} h3a_aewb_paxel_data_t;

typedef struct {
    h3a_aewb_paxel_data_t* h3a_res;     // poiter to input data for AWB
    uint16_t               h3a_data_x;  // X size of H3A data, blocks (paxels)
    uint16_t               h3a_data_y;  // Y size of H3A data, blocks (paxels)
    uint16_t               pix_in_pax;
} awb_frame_data_t;

typedef struct {
    uint8_t   *faces; // Array with face-tracking information (0 means no face; 1 means face)
} awb_alg_data_in_t;

typedef struct {
    uint8_t             flash_used;
    awb_frame_data_t    frame_data;
    uint8_t             preview_mode;
    awb_alg_data_in_t   is_face;
} ti_awb_data_in_t;

//----------------------------------------------------------------------------------
// This is the input data structure for AWB function TI_AWB_do(),
//----------------------------------------------------------------------------------

typedef struct {
    uint16_t  mode;                     // 0: auto, 1: manual, 2 : disabled
    uint16_t  manl_tmpr;                // manual white balance color temperature
    ti_awb_data_in_t ti_awb_data_in;    // H3A data for input

    uint32_t  init_done;                // Flag for TI_AWB_init done
    uint8_t   *AWB_ScratchMemory;       // [AWB_SCRATCH_MEM_SIZE]: share among all AWB instances
    awb_calc_data_t *sen_awb_calc_data; // sensor calibration data: set before calling TI_AWB_init
    uint32_t  v_img_ref[NUM_OF_REF_2];  // filled by TI_AWB_init

    uint8_t   stab_init_cnt;            // TI_AWB_stab states
    uint16_t  stab_gain_R;
    uint16_t  stab_gain_G;
    uint16_t  stab_gain_B;
    uint8_t   stab_awb_idx;
    uint8_t   stab_idx_hist[AWB_AVG_BUF_LENGTH];
    uint32_t  stab_color_temp;

    uint32_t  sb_total_exp;             // SB control: set to 999999 to disable

    uint32_t  dbg_sb_gain[NUM_OF_REF_1+2];
    uint32_t  dbg_tap[32];
} awbprm_t;

typedef struct {
    uint16_t     gain_Gr;                    // WB Gain for Gr. Format U16Q8
    uint16_t     gain_R;                     // WB Gain for R.  Format U16Q8
    uint16_t     gain_Gb;                    // WB Gain for Gb. Format U16Q8
    uint16_t     gain_B;                     // WB Gain for B.  Format U16Q8
    uint8_t      awb_idx;                    // AWB index : awb_mode-1(DL.FLOUR,etc)
    uint32_t     color_temperature_estim;    // color temperature estimation
    uint16_t     SB_count;                   // moved from awbprm_t to awb_data_out_t, renamed it to SB_count
} awb_data_out_t;

typedef enum {
    TI_AWB_ERROR_OK        = 0,   // no error
    TI_AWB_ERROR_CONFIGURE = 1,   // error algorithm is not configured
    TI_AWB_ERROR_TUNE      = 2    // error algorithm tuning data is not correct
} TI_AWB_ERROR;

// ===========================================================================
// TI_AWB_init
// INPUT:
//      awb_param: pointer to input data
// NOTE:
//   initialize the AWB algorithm,
//   calculates the settings to configures the AWB hardware
//   returns the initial values for AWB controlled variables
// ===========================================================================
TI_AWB_ERROR TI_AWB_init(awbprm_t *awb_param);

// ===========================================================================
// TI_AWB_do
// INPUT:
//      awbprm_t: input parameters to AWB
// OUTPUT:
//      awb_data_out_t : AWB output data structure
// NOTE:
//     executes 1 iteration of the AWB algorithm
// ===========================================================================
TI_AWB_ERROR TI_AWB_do(awbprm_t *awb_param, awb_data_out_t  *data_out);

// ===========================================================================
// void TI_AWB_stab (awbprm_t *awb_param, awb_data_out_t  *data_out)
// INPUT:
//      awbprm_t, ti_awb_data_in_t  : input parameters
// OUTPUT:
//      awb_data_out_t : update AWB output data structure
//                      int the location set in AWBInit() - awb_param->data_out
// NOTE:
//     executes 1 iteration of the AWB algorithm
// ===========================================================================
void TI_AWB_stab(awbprm_t *awb_param, awb_data_out_t  *data_out);

//extern awb_calc_data_t ts1_5_awb_calc_data;

#endif // __AAA_AWB_DEFS_H__
#ifdef __cplusplus
}
#endif /* __cplusplus */

