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

#ifndef __DCC_DEFS_H__
#define __DCC_DEFS_H__

#ifdef __STANDALONE
#define Rfile_printf printf
#endif

//#define __ENABLE_TRACE

#ifdef __ENABLE_TRACE
#define Trace_printf(...)   printf(__VA_ARGS__)
#else
#define Trace_printf(...)
#endif

/* This should be the max of all photospace instance */
#define DCC_MAX_PHOTO_SPACE_INST                (10U)

//
//  Following enumerated list represents main component/sub-component/algorithm IDs.
//  DCC Descriptor ID - enum dcc_descriptor_id_type
//
// !!!---!!!---!!!---!!!---!!!---!!!---!!!---!!!---!!!---!!!---!!!---!!!---!!!
// !!!                  ADD only AT the END                                !!!
// !!!            The valuues correspond with XMLs                         !!!
// !!!---!!!---!!!---!!!---!!!---!!!---!!!---!!!---!!!---!!!---!!!---!!!---!!!
//
typedef enum {
    DCC_ID_RESERVED_0,                    // 0
    DCC_ID_BLACK_CLAMP,            // 1
    DCC_ID_H3A_MUX_LUTS,          // 2
    DCC_ID_H3A_AEWB_CFG,                // 3
    DCC_ID_RFE_DECOMPAND,               // 4
    DCC_ID_MESH_LDC_J7,        // 5
    DCC_ID_VISS_GLBCE,        // 6
    DCC_ID_VISS_LSC,          // 7
    DCC_ID_RESERVED_8,                   // 8
    DCC_ID_IPIPE_CFA,                   // 9
    DCC_ID_IPIPE_RGB_RGB_1,             //10 - RGB2RGB before gamma - same parsers  and types as RGB_RGB_2
    DCC_ID_RESERVED_11,                 //11
    DCC_ID_RESERVED_12,             //12 - RGB2RGB after gamma - same parsers and types as RGB_RGB_1
    DCC_ID_RESERVED_13,                //13
    DCC_ID_RESERVED_14,                  //14
    DCC_ID_RESERVED_15,            //15
    DCC_ID_VISS_YEE,               //16 - YEE
    DCC_ID_RESERVED_17,                   //17
    DCC_ID_RESERVED_18,                   //18
    DCC_ID_RESERVED_19,         //19
    DCC_ID_RESERVED_20,                   //20
    DCC_ID_NSF4,                     //21 - NSF4
    DCC_ID_RESERVED_22,                     //22 - LDC for ODC in NSFLDC library - same parsers and types as LDC_CAC
    DCC_ID_RESERVED_23,                     //23 - LDC for CAC in NSFLDC library - same parsers and types as LDC_ODC
    DCC_ID_RESERVED_24,                      //24
    DCC_ID_RESERVED_25,              //25
    DCC_ID_RESERVED_26,                     //26
    DCC_ID_RESERVED_27,                  //27
    DCC_ID_RESERVED_28,                 //28 AWB2
    DCC_ID_RESERVED_29,             //29
    DCC_ID_RESERVED_30,             //30
    DCC_ID_RESERVED_31,              //31
    DCC_ID_RESERVED_32,              //32
    DCC_ID_RESERVED_33,             //33
    DCC_ID_RESERVED_34,            //34
    DCC_ID_RESERVED_35,                  //35
    DCC_ID_RESERVED_36,                   //36
    DCC_ID_RESERVED_37,               //37
    DCC_ID_RESERVED_38,                 //38 - 3D mechanical misalignement and stereo auto convergence
    DCC_ID_RESERVED_39,                   //39
    DCC_ID_AAA_ALG_AWB_TI3,             //40 AWB3
    DCC_ID_RESERVED_41,         //41 Camera capabilities
    DCC_ID_RESERVED_42,                  //42 KGenerator
    DCC_ID_RESERVED_43,                       //43 Video Stabilization
    DCC_ID_RESERVED_44,                             //44
    DCC_ID_RESERVED_45,                //45
    DCC_ID_RESERVED_46,                   //46
    DCC_ID_RESERVED_47,     //47
    DCC_ID_RESERVED_48,           //48
    DCC_ID_RESERVED_49,                         //49
    DCC_ID_RESERVED_50,                       //50
    DCC_ID_RESERVED_51,                       //51
    DCC_ID_RESERVED_52,       //52
    DCC_ID_RESERVED_53,       //53
    DCC_ID_RESERVED_54,                   //54
    DCC_ID_RESERVED_55,                    //55
    DCC_ID_COUNT,
    DCC_ID_INT  = 0x7FFFFFFF

} dcc_descriptor_id_type;
//
//  Following enumerated list represents main Algorithm vendor's ID.
//  Algorithm Vendor ID - enum dcc_algorithm_vendor_id_type
typedef enum {
    DCC_ALG_VENDOR_ID_1,
    DCC_ALG_VENDOR_ID_2,
    DCC_ALG_VENDOR_ID_3,
    DCC_ALG_VENDOR_ID_4,
    DCC_ALG_VENDOR_ID_5,
    DCC_ALG_VENDOR_ID_6,
    DCC_ALG_VENDOR_ID_7,
    DCC_ALG_VENDOR_ID_8,
    DCC_ALG_VENDOR_ID_9,
    DCC_ALG_VENDOR_ID_10
} dcc_algorithm_vendor_id_type;

//
//  Following enumerated list represents main DCC Use Case IDs.
//  DCC Use Case ID - enum dcc_use_case_id_type
typedef enum {
    DCC_USE_CASE_NONE = 0,
    DCC_USE_CASE_HIGH_SPEED_PREVIEW =           (1 << 0),
    DCC_USE_CASE_HIGH_QUALITY_PREVIEW =         (1 << 1),
    DCC_USE_CASE_HIGH_SPEED_STILL_CAPTURE =     (1 << 2),
    DCC_USE_CASE_HIGH_QUALITY_STILL_CAPTURE =   (1 << 3),
    DCC_USE_CASE_HIGH_SPEED_VIDEO_RECORD =      (1 << 4),
    DCC_USE_CASE_HIGH_QUALITY_VIDEO_RECORD =    (1 << 5),
    DCC_USE_CASE_VIDEO_TELECONFERENCE =         (1 << 6),
    DCC_USE_CASE_STILL_IMAGE_PLAYBACK =         (1 << 7),
    DCC_USE_CASE_STEREO_STILL_IMAGE_CAPTURE =   (1 << 8),
    DCC_USE_CASE_STEREO_VIDEO_CAPTURE =         (1 << 9)
} dcc_use_case_id_type;

typedef struct
{
    uint32_t min;
    uint32_t max;
} dcc_parser_dim_range;

//
//  Following enumerated list represents main Photo Space Dimension IDs.
//  Photo Space Dimension ID - enum dcc_photospace_dimension_id_type
typedef enum {
    DCC_PS_DIM_ID_AG,
    DCC_PS_DIM_ID_ET,
    DCC_PS_DIM_ID_CT,
    DCC_PS_DIM_ID_FLASH,
    DCC_PS_DIM_ID_FOCUS,
    DCC_PS_DIM_ID_TOTAL_EXP,
    DCC_PS_DIM_ID_FACE_DETECT,
    DCC_PS_DIM_ID_SCENE_MODE,
    DCC_PS_DIM_ID_EFFECTS_MODE,
    DCC_PS_DIM_ID_RESERVED_1,
    DCC_PS_DIM_ID_RESERVED_2,
    DCC_PS_DIM_ID_RESERVED_3,
    DCC_PS_DIM_ID_COUNT
} dcc_photospace_dimension_id_type;

//
//  Following structure represents metadata information related to the particular file instance.
//  Dynamic Camera Control Profile Header - struct dcc_component_header_type
typedef struct {
    uint32_t                              camera_module_id;
    dcc_descriptor_id_type                dcc_descriptor_id;
    dcc_algorithm_vendor_id_type          algorithm_vendor_id;
    uint32_t                              dcc_tuning_tool_version;
    uint32_t                              dcc_profile_time_stamp;
    uint32_t                              crc_checksum;
    uint32_t                              dcc_reserved_0;
    uint32_t                              dcc_reserved_1;
    uint32_t                              dcc_reserved_2;
    uint32_t                              dcc_reserved_3;
    uint32_t                              dcc_reserved_4;
    uint32_t                              dcc_reserved_5;
    uint32_t                              dcc_reserved_6;
    uint32_t                              dcc_reserved_7;
    uint32_t                              dcc_reserved_8;
    uint32_t                              dcc_reserved_9;
    uint32_t                              sz_comp_spec_gen_params;
    uint32_t                              sz_uc_spec_gen_params;
    uint32_t                              sz_x_dcc_descriptor;
    uint32_t                              total_file_sz;
} dcc_component_header_type;

typedef enum
{
    DCC_PHOTOSPACE_AG = 0,
    /* Analog Gain */
    DCC_PHOTOSPACE_ET,
    /* Exposure Time */
    DCC_PHOTOSPACE_CT,
    /* Color Temparature */
    DCC_MAX_PHOTO_SPACE
} dcc_photospace_dim_id;

typedef struct {
    uint8_t num_of_ref_1;
    uint8_t num_of_ref_2;
    uint8_t num_of_gray;
    uint16_t radius;
    uint32_t color_temp_1_uarr_size;
    uint16_t (*color_temp_1);
    uint32_t wbReferenceCb_uarr_size;
    int16_t (*wbReferenceCb)[4];
    uint32_t wbReferenceCr_uarr_size;
    int16_t (*wbReferenceCr)[4];
    uint32_t ref_gray_R_1_uarr_size;
    uint16_t (*ref_gray_R_1)[4];
    uint32_t ref_gray_G_1_uarr_size;
    uint16_t (*ref_gray_G_1)[4];
    uint32_t ref_gray_B_1_uarr_size;
    uint16_t (*ref_gray_B_1)[4];
    uint32_t ref_index_2_uarr_size;
    uint8_t (*ref_index_2);
    uint32_t color_temp_2_uarr_size;
    uint16_t (*color_temp_2);
    uint32_t referencesCb_2_uarr_size;
    int16_t (*referencesCb_2)[4];
    uint32_t referencesCr_2_uarr_size;
    int16_t (*referencesCr_2)[4];
    uint32_t ref_gray_R_2_uarr_size;
    uint16_t (*ref_gray_R_2)[4];
    uint32_t ref_gray_G_2_uarr_size;
    uint16_t (*ref_gray_G_2)[4];
    uint32_t ref_gray_B_2_uarr_size;
    uint16_t (*ref_gray_B_2)[4];
} dcc_awb_ref_gray_data_t;

typedef struct {
    dcc_awb_ref_gray_data_t awb_basic_ref;
    uint32_t img_ref_uarr_size;
    uint16_t (*img_ref);
    int32_t luma_awb_min;
    int32_t luma_awb_max;
    uint16_t low_color_temp_thresh;
    uint8_t apply_rgb_adjust;
    int16_t R_adjust;
    int16_t B_adjust;
    uint16_t SB_1;
    uint16_t SB_2;
    uint16_t SB_low_bound;
    uint16_t default_T_H;
    uint16_t default_T_MH;
    uint16_t default_T_ML;
    uint16_t default_T_L;
    uint8_t default_T_H_index;
    uint8_t default_T_MH_index;
    uint8_t default_T_ML_index;
    uint8_t default_T_L_index;
    uint8_t best_gray_index_default;
    dcc_awb_ref_gray_data_t flash_ref[4];
    uint32_t skin_img_ref_uarr_size;
    uint16_t (*skin_img_ref);
} dcc_awb_calc_data_t;

typedef struct {
    uint16_t dgain;
    uint16_t gainGr;
    uint16_t gainR;
    uint16_t gainGb;
    uint16_t gainB;
} dcc_ti2_color_gain_scalers_t;

typedef struct {
    uint32_t wbModeData_uarr_size;
    dcc_ti2_color_gain_scalers_t (*wbModeData);
} dcc_awb_ti2_data_t;

typedef struct {
    uint16_t use_calib;
    uint16_t blue_index;
    uint16_t red_index;
    uint16_t green1_index;
    uint16_t green2_index;
    uint8_t enable_opt;
    int16_t max_Cr;
    int16_t max_Cb;
    uint16_t awb_speed;
    dcc_awb_calc_data_t awb_calc_data;
    dcc_awb_ti2_data_t awb_data;
} dcc_awb_supp2_alg3_t;


typedef struct {
    //ptr Component Specific General Parameters
    uint8_t               *p_gen_data;

    //ptr to Use Case Specific General Parameters for requested use case
    uint8_t               *p_uc_data;

    //ptr to DCC Component Parameters for relevant Class according to requested
    //use case and photospace dimensions values
    uint8_t               *p_parpack;

    uint32_t              checksum;

    unsigned int          num_photospace;
    unsigned int          num_regions;
    unsigned int          photospace_id[DCC_MAX_PHOTO_SPACE];
    dcc_parser_dim_range  dim_range[DCC_MAX_PHOTO_SPACE_INST][DCC_MAX_PHOTO_SPACE];

    dcc_descriptor_id_type desc_id;
} dcc_ptrs_t;

#endif
