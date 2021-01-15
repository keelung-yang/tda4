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

/*
********************************************************************************
 * DCC API
 *
 * "DCC API" is software module developed for TI's ISS based SOCs.
 * This module provides APIs for programming of ISS hardware accelerators
 * which can be used for Imaging and video applications
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
********************************************************************************
*/
/**
********************************************************************************
 * @file  idcc.c
 *
 * @brief DCC Interface, contains defination of structures and functions,
 *        which are called by algo plugin layer
 *
********************************************************************************
*/

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef _I_DCC_
#define _I_DCC_

/*******************************************************************************
*                             INCLUDE FILES
*******************************************************************************/

/*---------------------- data declarations -----------------------------------*/

#define DCC_RGB2RGB1_MAX_PHOTO_SPACE_INST       (10U)
#define DCC_NSF4V_MAX_PHOTO_SPACE_INST          (10U)
#define DCC_YEE_MAX_PHOTO_SPACE_INST            (10U)

#include "dcc_defs.h"
#include <TI_aaa_awb.h>
#include "dcc_iss_module_def.h"

/* Mesh LDC Table Size, allocated at create time.
   Current implementation supports 1080p frame down scaled by 16
   in both direction */
#define DCC_MESH_LDC_TABLE_SIZE                 ((1920/16 + 1) *               \
                                                 (1080/16 + 1) * 2 * 2)

/* ISIF 2D LSC gain and offset table size, allocated at create time.
   Current implementation supports 1080p frame down scaled by 8
   in both direction */
#define DCC_ISIF_2D_LSC_GAIN_TABLE_SIZE         ((1920/8 + 1) *                \
                                                 (1080/8 + 1) * 4)

/**
 *******************************************************************************
 *  @struct dcc_parser_input_params_t
 *  @brief  This structure contains input parameters
 *
 *  @param  dcc_buf           : pointer to the buffer where dcc profile
                                are stored
 *  @param  dcc_buf_size      : Size of the dcc profile buffer
 *  @param  color_temparature : Color temperature of the scene
 *  @param  exposure_time     : exposure time use gad for the current scene
 *  @param  analog_gain       : analog gain used used in the current scene
 *
 *******************************************************************************
*/
typedef struct
{
    uint8_t  *dcc_buf;
    uint32_t dcc_buf_size;
    uint32_t color_temparature;
    uint32_t exposure_time;
    uint32_t analog_gain;
    uint32_t cameraId;
} dcc_parser_input_params_t;

/**
 *******************************************************************************
 *  @struct dcc_parser_input_params_t
 *  @brief  This structure contains output parameters
 *
 *  @param  iss_drv_config           : Pointer to iss drivers config
 *  @param  dcc_buf_size      : Size of the dcc profile buffer
 *  @param  color_temparature : Color temperature of the scene
 *  @param  exposure_time     : exposure time use gad for the current scene
 *  @param  analog_gain       : analog gain used used in the current scene
 *
 *******************************************************************************
*/
typedef struct {
    uint32_t                  useAwbCalbCfg;
    uint32_t                  useH3aCfg;
    uint32_t                  useNsf4Cfg;
    uint32_t                  useBlcCfg;
    uint32_t                  useCfaCfg;
    uint32_t                  useCcmCfg;
    uint32_t                  useH3aMuxCfg;
    uint32_t                  useRfeDcmpCfg;
    awb_calc_data_t           awbCalbData;
    iss_ipipe_h3a_aewb        ipipeH3A_AEWBCfg;
    iss_black_level_subtraction vissBLC;
    viss_ipipe_cfa_flxd       vissCFACfg; 
    uint32_t                  useVpacLdcCfg;
    vpac_ldc_dcc_cfg_t        vpacLdcCfg;
    /* Modules supporting multiple photospace */
    dcc_parser_dim_range      phPrmsRgb2Rgb1[DCC_RGB2RGB1_MAX_PHOTO_SPACE_INST][DCC_MAX_PHOTO_SPACE];
    uint32_t                  ipipeNumRgb2Rgb1Inst;
    iss_ipipe_rgb2rgb         ipipeRgb2Rgb1Cfg[DCC_RGB2RGB1_MAX_PHOTO_SPACE_INST];
    dcc_parser_dim_range      phPrmsNSF4[DCC_NSF4V_MAX_PHOTO_SPACE_INST][DCC_MAX_PHOTO_SPACE];
    uint32_t                  vissNumNSF4Inst;
    viss_nsf4                 vissNSF4Cfg[DCC_NSF4V_MAX_PHOTO_SPACE_INST];  
    iss_h3a_mux_luts          issH3aMuxLuts;
    iss_rfe_decompand         issRfeDecompand;
    uint32_t                  useVissGlbceCfg;
    viss_glbce_dcc_cfg_t      vissGlbceCfg;
    uint32_t                  useVissLscCfg;
    viss_lsc_dcc_cfg_t        vissLscCfg;
    uint32_t                  useVissYeeCfg;
    dcc_parser_dim_range      phPrmsYee[DCC_YEE_MAX_PHOTO_SPACE_INST][DCC_MAX_PHOTO_SPACE];
    uint32_t                  vissNumYeeInst;
    viss_yee_dcc_cfg_t        vissYeeCfg[DCC_YEE_MAX_PHOTO_SPACE_INST];
} dcc_parser_output_params_t;

/*******************************************************************************
*                         FUNCTION DEFINITIONS
*******************************************************************************/

/**
********************************************************************************
 * @fn      dcc_update(dcc_parser_input_params_t * input_params,
 *                     iss_drv_config_t *iss_drv_config
 *                    )
 *
 * @brief   This function identfies the dcc profile from input params structure
 *          and updates the iss driver configuration
 *          In the current implementation, it parses input bit file to
 *          get the ISP configuration and returns isp configuration
 *          in the output parameters
 *
 * @param   input_params
 *          input parameters for the dcc parser
 *
 *
 * @return  int
 *          sucess/failure
********************************************************************************
*/

int dcc_update(dcc_parser_input_params_t * input_params, dcc_parser_output_params_t *output_params);
uint32_t calc_dcc_outbuf_size();
int32_t Dcc_Create(dcc_parser_output_params_t * p_output_params, uint8_t * out_dcc_buf);
int32_t Dcc_Delete(dcc_parser_output_params_t * p_output_params);

int dcc_interp_CCM(
    dcc_parser_dim_range dim_range[][DCC_MAX_PHOTO_SPACE],
    int n_regions,
    int color_temp,
    iss_ipipe_rgb2rgb ccm_in[],
    iss_ipipe_rgb2rgb *p_ccm_int);

int dcc_search_NSF4(
    dcc_parser_dim_range dim_range[][DCC_MAX_PHOTO_SPACE],
    int n_regions,
    int analog_gain_in_ev,
    viss_nsf4 nsf4_matrix_in[],
    viss_nsf4 *p_nsf4_out);

int dcc_search_YEE(
    dcc_parser_dim_range dim_range[][DCC_MAX_PHOTO_SPACE],
    int n_regions,
    int analog_gain_in_ev);

#endif

#ifdef __cplusplus
}
#endif

