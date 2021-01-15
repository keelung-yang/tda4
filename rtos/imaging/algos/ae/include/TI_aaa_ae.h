/*
 *                                                                    
 * Copyright (c) 2020 Texas Instruments Incorporated
 *                                                                    
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifndef __TI_AAA_AE_HEADER_H__
#define __TI_AAA_AE_HEADER_H__

#include <stdint.h>

#define TIAE_MAX_RANGES 16
#define TIAE_MAX_HIST   16

#ifndef __AAA_AWB_DEFS_H__
typedef struct {
    uint16_t red;   // Average value for red pixels in current paxel
    uint16_t green; // Average value for green pixels in current paxel
    uint16_t blue;  // Average value for blue pixels in current paxel
    uint16_t ext;
} h3a_aewb_paxel_data_t;
#endif

typedef struct {
    int32_t aperture_size;  // linear: e.g., keep constant 1 if not used
    int32_t exposure_time;  // linear: e.g., use 1000 for 1ms
    int32_t analog_gain;    // linear: e.g., use 1024 for 1.0x gain
    int32_t digital_gain;   // linear: e.g., use 256 for 1.0x gain
} tiae_exp_t;

typedef struct {
    int32_t min;
    int32_t max;
} tiae_range_t;

typedef struct {
    int32_t      target_brightness;                     // linear: e.g., 45 (45/256 = 17.6%)
    tiae_range_t target_brightness_range;               // linear: e.g., (40, 50)
    int32_t      exposure_time_step_size;               // linear: use 1000 for 1ms
    int32_t      num_ranges;                            // <= TIAE_RANGE_T
    tiae_range_t aperture_size_range[TIAE_MAX_RANGES];  // linear as in tiae_exp_t
    tiae_range_t exposure_time_range[TIAE_MAX_RANGES];  // linear as in tiae_exp_t
    tiae_range_t analog_gain_range  [TIAE_MAX_RANGES];  // linear as in tiae_exp_t
    tiae_range_t digital_gain_range [TIAE_MAX_RANGES];  // linear as in tiae_exp_t
    uint32_t     enableBLC;  // BackLightCompensation
} tiae_exp_prog_t;

typedef struct {
    uint16_t        mode;                     // 0: auto, 1: manual, 2 : disabled
    tiae_exp_prog_t exposure_program;
    int32_t         num_history;
    int32_t         history_brightness[TIAE_MAX_HIST];
    int32_t         avg_y;
    int32_t         locked;
    int32_t         lock_cnt;
    int32_t         lock_thrld;
    int32_t         blc_enable;
    int32_t         blc_comp;
    tiae_exp_t      prev_ae;
    int32_t         frame_num_count;
    int32_t         frame_num_start;
    int32_t         frame_num_period;
} tiae_prm_t;

void TI_AE_init(tiae_prm_t *h, tiae_exp_prog_t *prg);

int TI_AE_do(                                // return 1: AE change; 0: no change
        tiae_prm_t            * h,
        h3a_aewb_paxel_data_t * h3a_data,    //must have been normalized by pix_in_pax
        int32_t                 h3a_data_x,  //H3A WINH
        int32_t                 h3a_data_y,  //H3A WINV
        uint8_t               * weight,      //relative weights for H3A windows (U8) or NULL
        uint16_t                r_gain,      //WB R gain (U16Q8)
        uint16_t                g_gain,      //WB G gain (U16Q8)
        uint16_t                b_gain,      //WB B gain (U16Q8)
        tiae_exp_t            * ae_out
);

#endif
#ifdef __cplusplus
}
#endif /* __cplusplus */

