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

#ifndef _DCC_ISS_MODULE_DEF_H_
#define _DCC_ISS_MODULE_DEF_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef x86_64
#include "FLXD_demosaic.h"
#include "FLXD_config_reader.h"
#endif

#define ISS_DCC_NUM_SUPPORT_MODULES        (12U)

/* IPIPE RGB2RGB Module Structure */
typedef struct
{
    int16_t matrix[3][4];
    int16_t offset[3];
} iss_ipipe_rgb2rgb;


/* VISS H3A_AEWB Module Structure */
typedef struct
{
    uint8_t enable;
    uint8_t mode;
    uint16_t v_start;
    uint16_t h_start;
    uint8_t v_size;
    uint8_t h_size;
    uint8_t v_count;
    uint8_t h_count;
    uint8_t v_skip;
    uint8_t h_skip;
    uint16_t saturation_limit;
    uint16_t blk_win_numlines;
    uint16_t blk_row_vpos;
    uint8_t sum_shift;
    uint8_t ALaw_En;
    uint8_t MedFilt_En;
} iss_ipipe_h3a_aewb;

/* VISS BLC Module Structure */
typedef struct
{
    int16_t vs_dcoffset[4];
    int16_t s_dcoffset[4];
    int16_t l_dcoffset[4];
} iss_black_level_subtraction;

/* RAWFE H3A MUX and LUTs */
typedef struct
{
    uint16_t enable;
    uint16_t h3a_mux_lut_num;
    uint16_t h3a_mux_lut[3][639];
} iss_h3a_mux_luts;

/* RAWFE wdr decompand with vshort */
typedef struct
{
    uint16_t enable;
    uint16_t mask;
    uint16_t shift;
    uint16_t bit_depth;
    uint16_t clip;
    uint16_t lut[639];
} iss_rfe_decompand;

/* VISS NSF4 Module Structure */
typedef struct
{
#if 0
    uint16_t enable;
    uint16_t mode;
    uint16_t shading_gain;
    uint16_t u1_knee;
    uint16_t tn1;
    uint16_t tn2;
    uint16_t tn3;
    uint16_t color0_lut[36];
    uint16_t color1_lut[36];
    uint16_t color2_lut[36];
    uint16_t color3_lut[36];
    uint16_t shd_x;
    uint16_t shd_y;
    uint16_t shd_t;
    uint16_t shd_kh;
    uint16_t shd_kv;
    uint16_t shd_gmax;
    uint16_t shd_set_sel;
    uint16_t shd_lut0[48];
    uint16_t shd_lut1[48];
    uint16_t wb_gains[4];
#else
    int32_t enable;
    int32_t mode;
    int32_t shading_gain;

    int32_t u1_knee;
    int32_t tn1;
    int32_t tn2;
    int32_t tn3;

    int32_t noise_thr_x[4][12];
    int32_t noise_thr_y[4][12];
    int32_t noise_thr_s[4][12];

    int32_t shd_x;
    int32_t shd_y;
    int32_t shd_t;
    int32_t shd_kh;
    int32_t shd_kv;
    int32_t shd_gmax;
    int32_t shd_set_sel;

    int32_t shd_lut_x[2][16];
    int32_t shd_lut_y[2][16];
    int32_t shd_lut_s[2][16];

    int32_t wb_gains[4];

#endif

}viss_nsf4;

/* VISS RFE CFA Module Structure */

#define FLXD_NUM_PHASE        4
#define FLXD_FIRSIZE_H        6
#define FLXD_FIRSIZE_W        6
#define FLXD_NUM_FIR          4
#define FLXD_NUM_THR          7
#define FLXD_NUM_BLEND        5
#define FLXD_NUM_BLENDINPUT   3
#define FLXD_DATAPATH_BITS    12 //TDA4x Implements 12 bit data pipe
#define FLXD_COMPAND_SEGMENTS 4
#define FLXD_LUT_SIZE        (639)


typedef struct
{
  int32_t matrix[FLXD_NUM_PHASE][FLXD_FIRSIZE_H * FLXD_FIRSIZE_W];
} FLXD_FirCoefs_DCC;

typedef struct
{
  uint32_t             bitWidth;
  uint32_t             lut_enable;

  uint32_t             Set0GradHzMask[4];
  uint32_t             Set0GradVtMask[4];
  uint32_t             Set0IntensityMask[4];
  uint32_t             Set0IntensityShift[4];
  uint32_t             Set0Thr[FLXD_NUM_THR];

  uint32_t             Set1GradHzMask[4];
  uint32_t             Set1GradVtMask[4];
  uint32_t             Set1IntensityMask[4];
  uint32_t             Set1IntensityShift[4];
  uint32_t             Set1Thr[FLXD_NUM_THR];
  uint32_t             blendMode[FLXD_NUM_FIR];
  uint32_t             bitMaskSel[FLXD_NUM_FIR];

  FLXD_FirCoefs_DCC   FirCoefs[FLXD_NUM_FIR *3];
  uint32_t  ToneLut[FLXD_LUT_SIZE];

} viss_ipipe_cfa_flxd;

typedef struct
{
    uint16_t en;              // LD enable
    uint16_t ldmapen;         // LD back mapping enable
    uint16_t data_mode;       // LD input data mode
    uint16_t out_in_420;      // LD 422 to 420 conversion
    uint16_t ip_dfmt;         // LD input pixel format
    uint16_t pwarpen;         // PWARP enable
    uint16_t ld_yint_typ;     // Interpolation method for Y data.  0: Bicubic, 1: Bilinear
    uint16_t regmode_en;      // Region mode enable.  0: off, 1: on
    uint16_t table_m;         // Table horizontal subsampling factor, 2^m
    uint16_t mesh_frame_w;    // mesh frame window height
    uint16_t mesh_frame_h;    // mesh frame window width
    uint16_t compute_sizew;   // compute window height, in pixels
    uint16_t compute_sizeh;   // compute window width, in pixels
    uint16_t ld_initx;        // compute window starting y, in pixels
    uint16_t ld_inity;        // compute window starting x, in pixels
    uint16_t iw;              // source (distorted) image width, in pixels
    uint16_t ih;              // source (distorted) image height, in pixels
    uint16_t ld_obw;          // output block height, in pixels, for block processing
    uint16_t ld_obh;          // output block height, in pixels, for block processing
    uint16_t ld_pad;          // pixel padding to determine input block for block processing

    int16_t affine_a;
    int16_t affine_b;
    int16_t affine_c;
    int16_t affine_d;
    int16_t affine_e;
    int16_t affine_f;
    int16_t affine_g;
    int16_t affine_h;

    uint16_t ld_sf_width[3];     // subframe width
    uint16_t ld_sf_height[3];    // subframe height
    uint16_t ld_sf_en [3][3];    // subframe enable
    uint16_t ld_sf_obw[3][3];    // output block height, in pixels, for block processing
    uint16_t ld_sf_obh[3][3];    // output block height, in pixels, for block processing
    uint16_t ld_sf_pad[3][3];    // pixel padding to determine input block for block processing

    uint16_t ylut_en;
    uint16_t yin_bits;
    uint16_t yout_bits;
    uint16_t clut_en;
    uint16_t cin_bits;
    uint16_t cout_bits;
    uint16_t ylut[513];
    uint16_t clut[513];
    uint32_t mesh_table_pitch;   // table row pitch in bytes
    uint32_t mesh_table_size;    // # of elements in "uint16_t mesh_table[]"
    uint32_t mesh_table_dccsize; // size from DCC
} vpac_ldc_dcc_params_t;

typedef struct
{
    vpac_ldc_dcc_params_t ldc_dcc_params;
    uint16_t  *mesh_table;
} vpac_ldc_dcc_cfg_t;

typedef struct
{
    uint32_t strength;
    uint32_t intensity_var;
    uint32_t space_var;
    uint32_t slope_min_lim;
    uint32_t slope_max_lim;
    uint32_t fwd_prcpt_en;
    uint32_t fwd_prcpt_lut[65];
    uint32_t rev_prcpt_en;
    uint32_t rev_prcpt_lut[65];
    uint32_t asym_lut[33];
} viss_glbce_dcc_cfg_t;

typedef struct
{
    uint32_t enable;
    uint32_t gain_mode_m;
    uint32_t gain_mode_n;
    uint32_t gain_mode_format;
    uint32_t lut_size_in_bytes;
    uint32_t lut_size_in_dcc;
} viss_lsc_dcc_params_t;

typedef struct
{
    viss_lsc_dcc_params_t lsc_params;
    uint8_t               * lsc_table;
} viss_lsc_dcc_cfg_t;

/* YEE Module Structure */
typedef struct
{
    uint16_t enable;
    uint16_t halo_reduction_enable;
    int16_t  ee_2d_filter_coeff[9];
    uint16_t merge_select;
    uint16_t shift_amount;
    uint16_t threshold_before_lut;
    uint16_t edge_sharpener_gain;
    uint16_t edge_sharpener_hpf_low_thresh;
    uint16_t edge_sharpener_hpf_high_thresh;
    uint16_t edge_sharpener_gradient_gain;
    uint16_t edge_sharpener_gradient_offset;
    int16_t  edge_intensity_lut[4096];
} viss_yee_dcc_cfg_t;

#if 0
/* ISIF CSC Module structure */
typedef struct
{
    uint8_t update;
    uint8_t enable;
    uint8_t csc_matrix[16];
} iss_isif_csc;

/* ISIF Black Clamping Module structure */
typedef struct
{
    uint16_t v_pos;
    uint8_t  v_size;
    uint16_t h_pos;
    uint8_t  h_size;
    uint8_t  pixel_value_limit;
    uint8_t  right_window;
    uint8_t  window_count_per_color;
} iss_isif_clamp_h_black_params;

typedef struct
{
    uint16_t v_pos;
    uint16_t v_size;
    uint16_t h_pos;
    uint8_t  h_size;
    uint8_t  line_avg_coef;
    int32_t  reset_mode;
    uint16_t reset_value;
} iss_isif_clamp_v_black_params;

typedef struct
{
    uint8_t update;
    uint8_t enable;
    uint8_t hclamp_mode;
    uint8_t black_clamp_mode;
    int16_t dcoffset_clamp_value;
    uint16_t black_clamp_v_start_pos;
    iss_isif_clamp_h_black_params horizontal_black;
    iss_isif_clamp_v_black_params vertical_black;
} iss_isif_clamp;

/* ISIF LSC Module structure */
typedef struct
{
    uint8_t enable;
    uint16_t use_calib;
    uint16_t lsc_hsize;
    uint16_t lsc_vsize;
    uint16_t hdirection_data_offset;
    uint16_t vdirection_data_offset;
    uint8_t hposin_paxel;
    uint8_t vposin_paxel;
    uint8_t pax_height;
    uint8_t pax_length;
    uint8_t gain_format;
    uint8_t offset_scaling_factor;
    int8_t offset_shift_value;
    uint8_t offset_enable;
    uint16_t gain_table_size;
    uint32_t lsc_table_uarr_size;
    uint8_t (*lsc_table);
    uint16_t offset_table_size;
    uint32_t offset_table_uarr_size;
    uint8_t (*offset_table);
} isif_lsc_dcc_descriptor_t;

/* IPIPE 3DLUT Module Structure */
typedef struct
{
    uint8_t update;
    uint8_t enable;
    uint16_t ddd_table_r[729];
    uint16_t ddd_table_g[729];
    uint16_t ddd_table_b[729];
} iss_ipipe_3d_lut;

/* IPIPE CAR Module Structure */
typedef struct
{
    uint8_t update;
    uint8_t enable;
    uint8_t car_type;
    uint8_t sw0_thr;
    uint8_t sw1_thr;
    uint8_t hpf_type;
    uint8_t hpf_shift;
    uint8_t hpf_thr;
    uint8_t gn1_gain;
    uint8_t gn1_shift;
    uint8_t gn1_min;
    uint8_t gn2_gain;
    uint8_t gn2_shift;
    uint8_t gn2_min;
} iss_ipipe_car;

/* IPIPE CFA Module Structure */
typedef struct
{
    uint16_t hpf_thr;
    uint16_t hpf_slope;
    uint16_t mix_thr;
    uint16_t mix_slope;
    uint16_t dir_thr;
    uint16_t dir_slope;
    uint16_t dir_ndwt;
} iss_ipipe_cfa_dir;

typedef struct
{
    uint8_t mono_hue_fra;
    uint8_t mono_edg_thr;
    uint16_t mono_thr_min;
    uint16_t mono_thr_slope;
    uint16_t mono_slp_min;
    uint16_t mono_slp_slp;
    uint16_t mono_lpwt;
} iss_ipipe_cfa_daa;

typedef struct
{
    uint8_t update;
    uint8_t cfai_mode;
    iss_ipipe_cfa_dir dir;
    iss_ipipe_cfa_daa daa;
} iss_ipipe_cfa;

/* IPIPE CGS Module Structure */
typedef struct
{
    uint8_t thr;
    uint8_t gain;
    uint8_t shift;
    uint8_t min;
} iss_ipipe_cgs_chroma_params;

typedef struct
{
    uint8_t update;
    uint8_t enable;
    iss_ipipe_cgs_chroma_params y_chroma_low;
    iss_ipipe_cgs_chroma_params y_chroma_high;
    iss_ipipe_cgs_chroma_params c_chroma;
} iss_ipipe_cgs;

/* IPIPE DPC Lut Module Structure */
typedef struct
{
    uint16_t horizontal_position;
    uint16_t vertical_position;
    uint8_t correction_method;
} iss_ipipe_dpc_lookuptbl;

typedef struct
{
    uint8_t update;
    uint8_t enable;
    uint32_t eeprom_table;
    uint16_t defect_cnt;
    uint8_t replace_type;
    iss_ipipe_dpc_lookuptbl dpc_lut[256];
} iss_ipipe_dpc_lut;

/* IPIPE DPC OTF Module Structure */
typedef struct
{
    uint16_t thr_cor_r;
    uint16_t thr_cor_gr;
    uint16_t thr_cor_gb;
    uint16_t thr_cor_b;
    uint16_t thr_det_r;
    uint16_t thr_det_gr;
    uint16_t thr_det_gb;
    uint16_t thr_det_b;
} ipipe_dpc_otf_dpc2;

typedef struct
{
    uint8_t shift;
    uint16_t d_thr;
    uint8_t d_slp;
    uint16_t d_min;
    uint16_t d_max;
    uint16_t c_thr;
    uint8_t c_slp;
    uint16_t c_min;
    uint16_t c_max;
} ipipe_dpc_otf_dpc3;

typedef struct
{
    uint8_t update;
    uint8_t enable;
    uint8_t dpc_detection_alg_type;
    uint8_t dpc_min_max_type;
    ipipe_dpc_otf_dpc2 dpc2_params;
    ipipe_dpc_otf_dpc3 dpc3_params;
} iss_ipipe_dpc_otf;

/* IPIPE YEE Module Structure */
typedef struct
{
    uint8_t update;
    uint8_t enable;
    uint8_t halo_reduction_enable;
    int16_t ee_2d_filter_coeff[9];
    uint8_t merge_select;
    uint8_t shift_amount;
    uint8_t threshold_before_lut;
    uint8_t edge_sharpener_gain;
    uint16_t edge_sharpener_hpf_low_thresh;
    uint8_t edge_sharpener_hpf_high_thresh;
    uint8_t edge_sharpener_gradient_gain;
    uint8_t edge_sharpener_gradient_offset;
    int16_t edge_intensity_lut[1024];
} iss_ipipe_ee;

/* IPIPE Gamma Module Structure */
typedef struct
{
    uint8_t update;
    uint8_t enable;
    uint8_t table[1024];
} iss_ipipe_gamma;

/* IPIPE GBCE Module Structure */
typedef struct
{
    uint8_t update;
    uint8_t enable;
    uint8_t gbce_type;
    uint16_t gbce_lut[1024];
} iss_ipipe_gbce;

/* IPIPE GIC Module Structure */
typedef struct
{
    uint8_t update;
    uint8_t enable;
    uint8_t radial_lsc_enable;
    uint8_t difference_index_select;
    uint8_t thr_select;
    uint8_t gic_gain;
    uint8_t gic_nfgain;
    uint16_t gic_thr;
    uint16_t gic_slp;
} iss_ipipe_gic;

/* IPIPE LSC Module Structure */
typedef struct
{
    uint8_t update;
    uint8_t enable;
    uint16_t v_offset;
    int16_t v_linear_coeff;
    int16_t v_quadratic_coeff;
    uint8_t v_linear_shift;
    uint8_t v_quadratic_shift;
    uint16_t h_offset;
    int16_t h_linear_coeff;
    int16_t h_quadratic_coeff;
    uint8_t h_linear_shift;
    uint8_t h_quadratic_shift;
    uint8_t gain_r;
    uint8_t gain_gr;
    uint8_t gain_gb;
    uint8_t gain_b;
    uint8_t off_r;
    uint8_t off_gr;
    uint8_t off_gb;
    uint8_t off_b;
    uint8_t shift;
    uint16_t max;
} ipipe_lsc_dcc_descriptor_t;

/* IPIPE NF1 Module Structure */
typedef struct
{
    uint8_t update;
    uint8_t enable;
    uint8_t down_shift_val;
    uint8_t green_sample_type;
    uint8_t spread_select;
    uint8_t spread_reg_val;
    uint8_t radial_lsc_enable;
    uint16_t thr[8];
    uint8_t str[8];
    uint8_t spr[8];
    uint16_t edge_min;
    uint16_t edge_max;
} iss_ipipe_nf1;

/* IPIPE RGB2YUV Module Structure */
typedef struct
{
    uint8_t update;
    int16_t matrix[3][3];
    int16_t offset[3];
    uint8_t brightness;
    uint8_t contrast;
    uint8_t y_min;
    uint8_t y_max;
    uint8_t c_min;
    uint8_t c_max;
} iss_ipipe_rgb2yuv;

/* IPIPE YUV444 to YUV422 Module Structure */
typedef struct
{
    uint8_t update;
    uint8_t chrominance_position;
    uint8_t low_pass_filter_enable;
} iss_ipipe_yuv444_to_yuv422;

#endif


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DCC_ISS_MODULE_DEF_H_ */
