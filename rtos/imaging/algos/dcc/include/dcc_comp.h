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

#ifndef _DCC_COMP_H_
#define _DCC_COMP_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/* ======================================================================= */
/* DCC_COMPLISTTABLE_TYPE - Table listing the available DCC components in
 * the system
 *
 * @param sCompName        : A NULL terminated string with max 128 characters
 *
 * @param pCompParse        : Component's Parser function
 */
/* ======================================================================= */
typedef struct
{
    char    *sCompName;
    int     (*comp_parse) (/*UInt8* b_sys_prm,
                           uint8_t* b_uc_prm,
                           uint8_t* b_parpack,*/
                           dcc_ptrs_t *dcc_ptrs,
                           void* sys_prm,
                           void* uc_prm,
                           void* parpack
                           /*UInt32 crc,
                           dcc_descriptor_id_type descId*/);
    void     (*comp_free) (void* sys_prm,
                           void* uc_prm,
                           void* parpack);
    void     (*comp_update) (void* dcc_data,
                             void* driver_data);
    void *  driver_ptr;
    int     struct_size;
    unsigned int isMultiPhSpaceSupported;
}DCC_COMPLISTTABLE_TYPE;

#define TRUE 1
#define FALSE 0

DCC_COMPLISTTABLE_TYPE tDCCCompList[] =
{
    {"DCC_ID_ISIF_CSC", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_BLACK_CLAMP", blc_dcc_bin_parse, NULL, dcc_update_blc, NULL, sizeof(iss_black_level_subtraction), FALSE},
    {"DCC_ID_H3A_MUX_LUTS", h3a_mux_luts_bin_parse, NULL, dcc_update_h3a_mux_luts, NULL, sizeof(iss_h3a_mux_luts), FALSE},
    {"DCC_ID_H3A_AEWB_CFG", h3a_aewb_dcc_bin_parse, NULL, dcc_update_h3a_aewb, NULL, sizeof(iss_ipipe_h3a_aewb), FALSE},
    {"DCC_ID_RFE_DECOMPAND", iss_rfe_decompand_bin_parse, NULL, dcc_update_iss_rfe_decompand, NULL, sizeof(iss_rfe_decompand), FALSE},
    {"DCC_ID_MESH_LDC_J7", vpac_ldc_bin_parse, NULL, dcc_update_vpac_ldc, NULL, sizeof(vpac_ldc_dcc_cfg_t), FALSE},
    {"DCC_ID_VISS_GLBCE", viss_glbce_bin_parse, NULL, dcc_update_viss_glbce, NULL, sizeof(viss_glbce_dcc_cfg_t), FALSE},
    {"DCC_ID_VISS_LSC", viss_lsc_bin_parse, NULL, dcc_update_viss_lsc, NULL, sizeof(viss_lsc_dcc_cfg_t), FALSE},
    {"DCC_ID_IPIPE_GIC", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_IPIPE_CFA", cfa_dcc_bin_parse, NULL, dcc_update_cfa, NULL, sizeof(viss_ipipe_cfa_flxd), FALSE},
    {"DCC_ID_IPIPE_RGB_RGB_1", ipipe_rgb2rgb_dcc_bin_parse, NULL, dcc_update_ipipe_rgb2rgb, NULL, sizeof(iss_ipipe_rgb2rgb), TRUE},
    {"DCC_ID_IPIPE_GAMMA", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_IPIPE_RGB_RGB_2", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_IPIPE_3D_LUT", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_IPIPE_GBCE", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_IPIPE_RGB_TO_YUV", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_VISS_YEE", viss_yee_bin_parse, NULL, dcc_update_viss_yee, NULL, sizeof(viss_yee_dcc_cfg_t), FALSE},
    {"DCC_ID_IPIPE_CAR", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_IPIPE_CGS", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_IPIPE_YUV444_YUV422", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_IPIPE_RSZ", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_NSF4", viss_nsf4_dcc_bin_parse, NULL, dcc_update_viss_nsf4, NULL, sizeof(viss_nsf4), FALSE},
    {"DCC_ID_LDC_ODC", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_LDC_CAC", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_LBCE_1", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_ADJUST_RGB2RGB", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_VNF_CFG", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_AAA_ALG_AE", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_AAA_ALG_AWB", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_AAA_ALG_AF_HLLC", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_AAA_ALG_AF_AFFW", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_AAA_ALG_AF_SAF", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_AAA_ALG_AF_CAF", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_ISS_SCENE_MODES", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_ISS_EFFECT_MODES", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_ISS_GLBCE3", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_ISS_GBCE2", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_IPIPE_DPC_LUT", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_3D_MMAC_SAC", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_IPIPE_LSC", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_AAA_ALG_AWB_TI3", awb_alg_dcc_tuning_dcc_bin_parse, NULL, NULL, NULL, sizeof(dcc_awb_supp2_alg3_t), FALSE},
    {"DCC_ID_CAMERA_CAPABILITIES", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_KGENERATOR", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_VSTAB", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DUMMY3", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_ALG_MMS_GBCE", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_MCTNF_CFG", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_IPIPEIF_VP_DECOMPANDING", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_IPIPEIF_WDR_MERGE", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_CNF", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_NSF3V", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_GLBCE", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_GLBCE_FORWARD_PERCEPT", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_GLBCE_REVERSE_PERCEPT", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_GLBCE_WDR", NULL, NULL, NULL, NULL, 0, FALSE},
    {"DCC_ID_MESH_LDC", NULL, NULL, NULL, NULL, 0, FALSE},
};

static const dcc_descriptor_id_type gDccModuleList[ISS_DCC_NUM_SUPPORT_MODULES] =
{
    //DCC_ID_ISIF_BLACK_CLAMP,
    //DCC_ID_IPIPE_CFA,
    //DCC_ID_IPIPE_3D_LUT,

    DCC_ID_H3A_MUX_LUTS,
    DCC_ID_H3A_AEWB_CFG,
    DCC_ID_RFE_DECOMPAND,
    DCC_ID_IPIPE_RGB_RGB_1,
    DCC_ID_NSF4,
    DCC_ID_AAA_ALG_AWB_TI3,
    DCC_ID_BLACK_CLAMP,
    DCC_ID_IPIPE_CFA,
    DCC_ID_MESH_LDC_J7,
    DCC_ID_VISS_GLBCE,
    DCC_ID_VISS_LSC,
    DCC_ID_VISS_YEE,
    //DCC_ID_CNF,
    //DCC_ID_NSF3V,
    //DCC_ID_IPIPEIF_VP_DECOMPANDING,
    //DCC_ID_IPIPEIF_WDR_MERGE,
    //DCC_ID_IPIPE_GAMMA,
    //DCC_ID_IPIPE_EE,
    //DCC_ID_IPIPE_RGB_TO_YUV,
    //DCC_ID_IPIPE_DPC_OTF,
    //DCC_ID_IPIPE_NOISE_FILTER_1,
    //DCC_ID_IPIPE_NOISE_FILTER_2,
    //DCC_ID_IPIPE_GIC,
    //DCC_ID_GLBCE,
    //DCC_ID_GLBCE_FORWARD_PERCEPT,
    //DCC_ID_GLBCE_REVERSE_PERCEPT,
    //DCC_ID_GLBCE_WDR,
    //DCC_ID_H3A_AEWB_CFG,
    //DCC_ID_MESH_LDC,
    //DCC_ID_LSC,
};


#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*_DCC_COMP_H_*/
