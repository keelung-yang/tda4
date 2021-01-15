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

/* =============================================================================
*                                  INCLUDE FILES
* =========================================================================== */

#ifndef __PARSE_DATA_H__
#define __PARSE_DATA_H__

#include "dcc_funcs.h"
#include "dcc_defs.h"


/* ========================================================================== */
/**
*  void parse_header_data(uint8_t **dcc,
*                       void *out,
*                       uint32_t num_use_cases)
*  parses header data and number of use cases from DCC Profile
*
*  @param   dcc - pointer to dcc data
*
*  @param   out - pointer for the parsed data
*
*  @param   num_use_cases - number of use cases
*
*  @return  nothing.
*/
/* ========================================================================== */
void parse_header_data(uint8_t **dcc,
                       dcc_component_header_type* header_data);


/* ========================================================================== */
/**
*  uint32_t find_use_case_block()
*  parses use case table of contents data in DCC Profile
*
*  @param   dcc - pointer to dcc data
*
*  @param   out - pointer for the parsed data
*
*  @return  nothing.
*/
/* ========================================================================== */
uint32_t find_use_case_block(uint8_t **p_uc_desc,
                         dcc_use_case_id_type use_case_id,
                         uint32_t* num_use_cases);


/* ========================================================================== */
/**
* find_parpack()
*  finds the index of the proper photospace class based on the passed values in
*  dim_values
*
*  @param   use_case_block - pointer to the current use case data
*
*  @param   dim_values - points to struct containing the current values for all
* possible dimension
*
*  @param   parpack_bytes - size of the parameter packet (get from the dcc
*  header)
*
*  @param   out - pointer to struct with determined pointers during executing of
* find_parpack routine
*
*  @return  nothing.
*/
/* ========================================================================== */
uint8_t* find_parpack(uint8_t    *use_case_block, //ptr to the use case block
                    uint32_t  parpack_bytes,
                    dcc_ptrs_t *dcc_ptrs);

int ipipe_rgb2rgb_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    void *sys_prm, void *uc_prm, void *parpack
);
void dcc_update_ipipe_rgb2rgb (void * dcc_data,
                               void * driver_data
);

int h3a_mux_luts_bin_parse(
    dcc_ptrs_t *dcc_ptrs, void *sys_prm, void *uc_prm, void *parpack);
void dcc_update_h3a_mux_luts(void *dcc_data, void *driver_data);

int iss_rfe_decompand_bin_parse(
    dcc_ptrs_t *dcc_ptrs, void *sys_prm, void *uc_prm, void *parpack);
void dcc_update_iss_rfe_decompand(void * dcc_data,
                               void * driver_data);

int h3a_aewb_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    void* sys_prm, void* uc_prm, void* parpack);
void h3a_aewb_dcc_bin_free(
    void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_h3a_aewb(void *dcc_data,
                          void *driver_data);

int viss_nsf4_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    void *sys_prm, void *uc_prm, void *parpack);

void dcc_update_viss_nsf4(void * dcc_data,
                               void * driver_data);

void dcc_update_cfa(void * dcc_data, void * driver_data);

int awb_alg_dcc_tuning_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    void* sys_prm, void* uc_prm, void* parpack
);

int blc_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    void *sys_prm, void *uc_prm, void *parpack);

void dcc_update_blc(void * dcc_data,
                               void * driver_data);

int cfa_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    void *sys_prm, void *uc_prm, void *parpack);

int vpac_ldc_bin_parse(dcc_ptrs_t *dcc_ptrs, void *sys_prm, void *uc_prm, void *parpack);
void dcc_update_vpac_ldc(void * dcc_data, void * driver_data);

int viss_glbce_bin_parse(dcc_ptrs_t *dcc_ptrs, void *sys_prm, void *uc_prm, void *parpack);
void dcc_update_viss_glbce(void * dcc_data, void * driver_data);

int viss_lsc_bin_parse(dcc_ptrs_t *dcc_ptrs, void *sys_prm, void *uc_prm, void *parpack);
void dcc_update_viss_lsc(void * dcc_data, void * driver_data);

int viss_yee_bin_parse(dcc_ptrs_t *dcc_ptrs, void *sys_prm, void *uc_prm, void *parpack);
void dcc_update_viss_yee(void * dcc_data, void * driver_data);

#if 0
int ipipe_dpc_otf_dcc_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t* b_sys_prm, uint8_t* b_uc_prm, uint8_t* b_parpack,
    void* sys_prm, void* uc_prm, void* parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void ipipe_dpc_otf_dcc_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_ipipe_dpc_otf(void* dcc_data,
                              void* driver_data);


int ipipe_nf1_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t *b_sys_prm, uint8_t *b_uc_prm, uint8_t *b_parpack,
    void *sys_prm, void *uc_prm, void *parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void ipipe_nf1_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_ipipe_noise_filter(void *dcc_data,
                                   void *driver_data);

int ipipe_gic_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t *b_sys_prm, uint8_t *b_uc_prm, uint8_t *b_parpack,
    void *sys_prm, void *uc_prm, void *parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void ipipe_gic_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_ipipe_gic(void *dcc_data,
                          void *driver_data);

int ipipe_cfa_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t *b_sys_prm, uint8_t *b_uc_prm, uint8_t *b_parpack,
    void *sys_prm, void *uc_prm, void *parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void ipipe_cfa_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_ipipe_cfa(void *dcc_data,
                          void *driver_data);

int cnf_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t *b_sys_prm, uint8_t *b_uc_prm, uint8_t *b_parpack,
    void *sys_prm, void *uc_prm, void *parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void cnf_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_cnf (void * dcc_data,
                     void * driver_data);

int nsf3v_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t *b_sys_prm, uint8_t *b_uc_prm, uint8_t *b_parpack,
    void *sys_prm, void *uc_prm, void *parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void nsf3v_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_nsf3v (void * dcc_data,
                       void * driver_data);

int ipipe_3d_lut_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t *b_sys_prm, uint8_t *b_uc_prm, uint8_t *b_parpack,
    void *sys_prm, void *uc_prm, void *parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void ipipe_3d_lut_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_ipipe_3d_lut (void * dcc_data,
                              void * driver_data);

int ipipe_gamma_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t *b_sys_prm, uint8_t *b_uc_prm, uint8_t *b_parpack,
    void *sys_prm, void *uc_prm, void *parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void ipipe_gamma_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_ipipe_gamma (void * dcc_data,
                     void * driver_data);

int ipipe_rgb2yuv_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t *b_sys_prm, uint8_t *b_uc_prm, uint8_t *b_parpack,
    void *sys_prm, void *uc_prm, void *parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void ipipe_rgb2yuv_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_ipipe_rgb2yuv(void *dcc_data,
                              void *driver_data);

int ipipe_ee_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t *b_sys_prm, uint8_t *b_uc_prm, uint8_t *b_parpack,
    void *sys_prm, void *uc_prm, void *parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void ipipe_ee_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_ipipe_ee (void *dcc_data,
                          void *driver_data);

int ipipeif_vp_decompand_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t *b_sys_prm, uint8_t *b_uc_prm, uint8_t *b_parpack,
    void *sys_prm, void *uc_prm, void *parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void ipipeif_vp_decompand_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_ipipeif_vp_decompand(void *dcc_data,
                          void *driver_data);

int ipipeif_wdr_merge_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t *b_sys_prm, uint8_t *b_uc_prm, uint8_t *b_parpack,
    void *sys_prm, void *uc_prm, void *parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void ipipeif_wdr_merge_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_ipipeif_wdr_merge(void *dcc_data,
                          void *driver_data);

int isif_clamp_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    //uint8_t* b_sys_prm, uint8_t* b_uc_prm, uint8_t* b_parpack,
    void* sys_prm, void* uc_prm, void* parpack
    /*uint32_t crc,
    dcc_descriptor_id_type descId,
    Ptr allocMem,
    uint32_t num_regions*/);
void isif_clamp_dcc_bin_free(void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_isif_clamp(void *dcc_data,
                          void *driver_data);


int glbce_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    void* sys_prm, void* uc_prm, void* parpack);
void glbce_dcc_bin_free(
    void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_glbce(void *dcc_data,
                      void *driver_data);

int glbce_fwd_per_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    void* sys_prm, void* uc_prm, void* parpack);
void glbce_fwd_per_dcc_bin_free(
    void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_glbce_fwd_per(void *dcc_data,
                      void *driver_data);

int glbce_rev_per_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    void* sys_prm, void* uc_prm, void* parpack);
void glbce_rev_per_dcc_bin_free(
    void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_glbce_rev_per(void *dcc_data,
                      void *driver_data);

int glbce_wdr_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    void* sys_prm, void* uc_prm, void* parpack);
void glbce_wdr_dcc_bin_free(
    void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_glbce_wdr(void *dcc_data,
                          void *driver_data);


int mesh_ldc_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    void* sys_prm, void* uc_prm, void* parpack);
void mesh_ldc_dcc_bin_free(
    void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_mesh_ldc(void *dcc_data,
                          void *driver_data);

int isif_2d_lsc_dcc_bin_parse(
    dcc_ptrs_t *dcc_ptrs,
    void* sys_prm, void* uc_prm, void* parpack);
void isif_2d_lsc_dcc_bin_free(
    void* sys_prm, void* uc_prm, void* parpack);
void dcc_update_isif_2d_lsc(void *dcc_data,
                            void *driver_data);
#endif

#endif //__PARSE_DATA_H__
