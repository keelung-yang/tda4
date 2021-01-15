/*
 *
 * Copyright (c) 2017 Texas Instruments Incorporated
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
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include "TI/tivx.h"
#include "TI/j7_vpac_viss.h"
#include "TI/j7_imaging_aewb.h"
#include "VX/vx.h"
#include "tivx_imaging_kernels_priv.h"
#include "tivx_kernel_aewb.h"
#include "TI/tivx_target_kernel.h"
#include "tivx_kernels_target_utils.h"
#include "TI_aaa_awb.h"
#include "TI_aaa_ae.h"
#include "AWB_config_hardware.h"
#include "idcc.h"
#include <app_ipc.h>
#include <app_remote_service.h>

#include <iss_sensors.h>
#include <itt_srvr_remote.h>
#include <TI/j7_viss_srvr_remote.h>

static vx_status AWB_TI_create(awbprm_t *p_awb_params, awb_calc_data_t* p_calib);
static vx_status AWB_TI_parse_H3a_buf(uint8_t * pH3a_out, awbprm_t *p_awb_params,
        h3a_aewb_paxel_data_t * awb_h3a_res, h3a_aewb_paxel_data_t * p_h3a_merge);
static vx_status AWB_TI_process(
        h3a_aewb_paxel_data_t * awb_h3a_res,
        awbprm_t              * p_awb_params,
        tivx_ae_awb_params_t  * aewb_prev,
        tivx_ae_awb_params_t  * aewb_result,
        uint8_t               * scratch_mem,
        uint8_t                 sensor_pre_gain);

#ifndef x86_64
static void map_sensor_ae_dynparams(IssAeDynamicParams * p_ae_dynPrms, tiae_exp_prog_t * p_ae_exp_prg);
static int32_t sendAewbToViss(uint32_t channel_id, tivx_ae_awb_params_t *pAewbPrms);
#endif

typedef struct
{
    vx_uint16 dcc_id;
    dcc_parser_input_params_t * dcc_input_params;
    dcc_parser_output_params_t * dcc_output_params;
    awbprm_t * p_awb_params;
    tiae_prm_t * p_ae_params;
    h3a_aewb_paxel_data_t      * p_h3a_merge;
    uint32_t      frame_count;
    int32_t       sensor_pre_wb_gain;
    tivx_ae_awb_params_t ae_awb_result_prev;
    h3a_aewb_paxel_data_t * awb_h3a_res;
} tivxAEWBParams;

static tivx_target_kernel vx_aewb_target_kernel = NULL;

static vx_status VX_CALLBACK tivxAewbProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxAewbCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxAewbDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg);
static vx_status VX_CALLBACK tivxAewbControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params,
       void *priv_arg);

#ifndef x86_64

extern AlgItt_IssAewb2AControlParams aewbCtrlPrms[ISS_SENSORS_MAX_SUPPORTED_SENSOR];
extern tivx_mutex                 itt_aewb_lock[ISS_SENSORS_MAX_SUPPORTED_SENSOR];

static vx_status AE_TI_process(h3a_aewb_paxel_data_t *awb_h3a_res, int h3a_data_x, int h3a_data_y,
                               tiae_prm_t *h, tivx_ae_awb_params_t *aewb_prev, tivx_ae_awb_params_t *aewb_result)
{
    if(!awb_h3a_res)
    {
        VX_PRINT(VX_ZONE_ERROR, "awb_h3a_res is NULL");
        return -1;
    }
    if(!(aewb_prev))
    {
        VX_PRINT(VX_ZONE_ERROR, "aewb_prev is NULL");
        return -1;
    }
    if(!aewb_result)
    {
        VX_PRINT(VX_ZONE_ERROR, "aewb_result is NULL");
        return -1;
    }

    //tiae_exp_t exp1 = h->prev_ae;
    tiae_exp_t exp2;
    aewb_result->ae_converged = TI_AE_do(h, awb_h3a_res, h3a_data_x, h3a_data_y, NULL, 256, 256, 256, &exp2);

    //float adj = (float)exp2.aperture_size / exp1.aperture_size
    //            * exp2.exposure_time  / exp1.exposure_time
    //            * exp2.analog_gain    / exp1.analog_gain
    //            * exp2.digital_gain   / exp1.digital_gain;
    //printf(">>> AE debug kernel: adj = %4f, locked = %d, lock_cnt = %d\n", adj, h->locked, h->lock_cnt);
    //printf(">>> AE debug kernel: T1 = %d, T2 = %d\n", exp1.exposure_time, exp2.exposure_time);

    aewb_result->analog_gain   = exp2.analog_gain;
    aewb_result->digital_gain  = exp2.digital_gain;
    aewb_result->exposure_time = exp2.exposure_time;
    aewb_result->ae_valid = 1;

    return VX_SUCCESS;
}
#endif //x86_64

// Gang TODO: this only works with j7 h3a c model; MUST change to work with j7 H/W
static uint8_t * decode_h3a_header(uint8_t * h3a_buf, uint16_t *n_row, uint16_t *n_col, uint16_t * n_pix)
{
    // the header is added by H3A c model
    uint16_t * header = (uint16_t*) h3a_buf;
    *n_row = header[2];
    *n_col = header[3];
    uint16_t winh = (header[1] + header[5] - 1) / header[5];
    uint16_t winv = (header[0] + header[4] - 1) / header[4];
    *n_pix = winh * winv;
    VX_PRINT(VX_ZONE_INFO, "H3A config: winH=%d, winW=%d, winVC=%d, winHC=%d, incV=%d, incH=%d\n",
            header[0], header[1], header[2], header[3], header[4], header[5]);

    return &h3a_buf[12];
}

static uint8_t * decode_h3a_header_dcc(uint8_t * h3a_buf, iss_ipipe_h3a_aewb * h3a_dcc_cfg,
                                                            uint16_t *n_row, uint16_t *n_col, uint16_t * n_pix)
{
    uint16_t winh = (h3a_dcc_cfg->h_size + h3a_dcc_cfg->h_skip - 1)/h3a_dcc_cfg->h_skip;
    uint16_t winv = (h3a_dcc_cfg->v_size + h3a_dcc_cfg->v_skip - 1)/h3a_dcc_cfg->v_skip;
    *n_row = h3a_dcc_cfg->v_count;
    *n_col = h3a_dcc_cfg->h_count;
    *n_pix = winh * winv;
    VX_PRINT(VX_ZONE_INFO, "decode_h3a_header_dcc : num_rows = %d\n", *n_row);
    VX_PRINT(VX_ZONE_INFO, "decode_h3a_header_dcc : num_cols = %d\n", *n_col);
    VX_PRINT(VX_ZONE_INFO, "decode_h3a_header_dcc : num_pix = %d\n", *n_pix);

    return h3a_buf;
}

#ifndef x86_64

/*
    2A node does not know sensor name, I2C address or any other parameters
    It recieves channel ID from OpenVX graph and computes exp, gain params
    The results are send to sensor driver f/w along with channel ID
    It is the responsibility of sensor driver f/w to apply these results to
    the correct sensor

    This needs to generic even if 2A node and sensor driver are executing on
    different cores and Operating Systems

    2A node will send a Control Command to sensor driver f/w. Payload format is

    ----ISS_SENSORS_MAX_NAME bytes sensor name prefilled to "dummy". Ignored by sensor driver
    ----32-bit channel ID
    ----32-bit Command ID = SETEXP_PARAMS
    ----aePrms
*/
static int32_t sendExpGainToSensor(tivx_aewb_config_t * aewb_config, IssSensor_ExposureParams *pAePrms)
{
    int32_t status = -1;
    uint8_t  cmdPrm[CMD_PARAM_SIZE];
    uint8_t * ptr8 = (uint8_t * )cmdPrm;
    uint32_t chId = (uint32_t)aewb_config->channel_id;
    char dummy_name[ISS_SENSORS_MAX_NAME] = "dummy";
    IMAGE_SENSOR_CTRLCMD ctrlCmd = IMAGE_SENSOR_CTRLCMD_SETEXPGAIN;

    memcpy(ptr8, dummy_name, ISS_SENSORS_MAX_NAME);
    ptr8 += ISS_SENSORS_MAX_NAME;

    memcpy(ptr8, &chId, sizeof(uint32_t));
    ptr8 += sizeof(uint32_t);

    memcpy(ptr8, &ctrlCmd, sizeof(IMAGE_SENSOR_CTRLCMD));
    ptr8 += sizeof(IMAGE_SENSOR_CTRLCMD);

    memcpy(ptr8, pAePrms, sizeof(IssSensor_ExposureParams));
    VX_PRINT(VX_ZONE_INFO, "Sending command to sensor \n");
    status = appRemoteServiceRun(
        APP_IPC_CPU_MCU2_0,
        IMAGE_SENSOR_REMOTE_SERVICE_NAME,
        IM_SENSOR_CMD_CTL,
        (void*)cmdPrm,
        CMD_PARAM_SIZE,
        0
    );
    return status;
}

/*
    2A node will send a Control Command to sensor driver f/w. Payload format is

    ----ISS_SENSORS_MAX_NAME bytes sensor name prefilled to "dummy". Ignored by sensor driver
    ----32-bit channel ID
    ----32-bit Command ID = GETEXPPRG
    ----Sensor driver will copy IssAeDynamicParams in the command buffer at the location after command ID

    Must be careful that worst case size of IssAeDynamicParams does not exceed the available bytes in command buffer
*/
static int32_t getExpPrgFromSensor(uint32_t channel_id, IssAeDynamicParams * p_AEdynPrms)
{
    int32_t status = -1;
    uint8_t  cmdPrm[CMD_PARAM_SIZE];
    uint8_t * ptr8 = (uint8_t * )cmdPrm;
    char dummy_name[ISS_SENSORS_MAX_NAME] = "dummy";
    IMAGE_SENSOR_CTRLCMD ctrlCmd = IMAGE_SENSOR_CTRLCMD_GETEXPPRG;

    memcpy(ptr8, dummy_name, ISS_SENSORS_MAX_NAME);
    ptr8 += ISS_SENSORS_MAX_NAME;

    memcpy(ptr8, &channel_id, sizeof(uint32_t));
    ptr8 += sizeof(uint32_t);

    memcpy(ptr8, &ctrlCmd, sizeof(IMAGE_SENSOR_CTRLCMD));
    ptr8 += sizeof(IMAGE_SENSOR_CTRLCMD);

    VX_PRINT(VX_ZONE_INFO, "Sending control command to sensor \n");
    status = appRemoteServiceRun(
        APP_IPC_CPU_MCU2_0,
        IMAGE_SENSOR_REMOTE_SERVICE_NAME,
        IM_SENSOR_CMD_CTL,
        (void*)cmdPrm,
        CMD_PARAM_SIZE,
        0
    );
    memcpy(p_AEdynPrms, ptr8, sizeof(IssAeDynamicParams));
    return status;
}

/*
    2A node will send a Control Command to sensor driver f/w. Payload format is

    ----ISS_SENSORS_MAX_NAME bytes sensor name prefilled to "dummy". Ignored by sensor driver
    ----32-bit channel ID
    ----32-bit Command ID = GETWBPRG
    ----Sensor driver will copy IssAwbDynamicParams in the command buffer at the location after command ID

*/
static int32_t getWbPrgFromSensor(uint32_t channel_id, IssAwbDynamicParams * p_AWBdynPrms)
{
    int32_t status = -1;
    uint8_t  cmdPrm[CMD_PARAM_SIZE];
    uint8_t * ptr8 = (uint8_t * )cmdPrm;
    char dummy_name[ISS_SENSORS_MAX_NAME] = "dummy";
    IMAGE_SENSOR_CTRLCMD ctrlCmd = IMAGE_SENSOR_CTRLCMD_GETWBCFG;

    memcpy(ptr8, dummy_name, ISS_SENSORS_MAX_NAME);
    ptr8 += ISS_SENSORS_MAX_NAME;

    memcpy(ptr8, &channel_id, sizeof(uint32_t));
    ptr8 += sizeof(uint32_t);

    memcpy(ptr8, &ctrlCmd, sizeof(IMAGE_SENSOR_CTRLCMD));
    ptr8 += sizeof(IMAGE_SENSOR_CTRLCMD);

    status = appRemoteServiceRun(
        APP_IPC_CPU_MCU2_0,
        IMAGE_SENSOR_REMOTE_SERVICE_NAME,
        IM_SENSOR_CMD_CTL,
        (void*)cmdPrm,
        CMD_PARAM_SIZE,
        0
    );
    memcpy(p_AWBdynPrms, ptr8, sizeof(IssAwbDynamicParams));
    return status;
}

/*
    2A node does not know sensor name, I2C address or any other parameters
    It recieves channel ID from OpenVX graph and computes exp, gain params
    The results are send to sensor driver f/w along with channel ID
    It is the responsibility of sensor driver f/w to apply these results to
    the correct sensor

    This needs to generic even if 2A node and sensor driver are executing on
    different cores and Operating Systems

    2A node will send a Control Command to sensor driver f/w. Payload format is

    ----ISS_SENSORS_MAX_NAME bytes sensor name prefilled to "dummy". Ignored by sensor driver
    ----32-bit channel ID
    ----32-bit Command ID = IMAGE_SENSOR_CTRLCMD_SETWBGAIN
    ----aePrms
*/

static int32_t sendWbGainToSensor(tivx_aewb_config_t * aewb_config, IssSensor_WhiteBalanceParams *pAwbPrms)
{
    int32_t status = -1;
    uint8_t  cmdPrm[CMD_PARAM_SIZE];
    uint8_t * ptr8 = (uint8_t * )cmdPrm;
    uint32_t chId = (uint32_t)aewb_config->channel_id;
    char dummy_name[ISS_SENSORS_MAX_NAME] = "dummy";
    IMAGE_SENSOR_CTRLCMD ctrlCmd = IMAGE_SENSOR_CTRLCMD_SETWBGAIN;

    memcpy(ptr8, dummy_name, ISS_SENSORS_MAX_NAME);
    ptr8 += ISS_SENSORS_MAX_NAME;

    memcpy(ptr8, &chId, sizeof(uint32_t));
    ptr8 += sizeof(uint32_t);

    memcpy(ptr8, &ctrlCmd, sizeof(IMAGE_SENSOR_CTRLCMD));
    ptr8 += sizeof(IMAGE_SENSOR_CTRLCMD);

    memcpy(ptr8, pAwbPrms, sizeof(IssSensor_WhiteBalanceParams));

    status = appRemoteServiceRun(
        APP_IPC_CPU_MCU2_0,
        IMAGE_SENSOR_REMOTE_SERVICE_NAME,
        IM_SENSOR_CMD_CTL,
        (void*)cmdPrm,
        CMD_PARAM_SIZE,
        0
    );
    return status;
}

/*

    2A node will send a Control Command to VISS node. Payload format is

    ----32-bit channel ID
    ----32-bit Command ID = VISS_CMD_SET_2A_PARAMS
    ----awbPrms
*/
static uint8_t  vissCmdPrm[VISS_CMD_PARAM_SIZE];
static int32_t sendAewbToViss(uint32_t channel_id, tivx_ae_awb_params_t *pAewbPrms)
{
    int32_t status = -1;
    uint8_t * ptr8 = (uint8_t * )vissCmdPrm;
    uint32_t chId = channel_id;

    memcpy(ptr8, &chId, sizeof(uint32_t));
    ptr8 += sizeof(uint32_t);

    memcpy(ptr8, pAewbPrms, sizeof(tivx_ae_awb_params_t));

    status = appRemoteServiceRun(
        APP_IPC_CPU_MCU2_0,
        VISS_SERVER_REMOTE_SERVICE_NAME,
        VISS_CMD_SET_2A_PARAMS,
        (void*)vissCmdPrm,
        VISS_CMD_PARAM_SIZE,
        0
    );

    if(0!=status)
    {
        printf("VISS_CMD_SET_2A_PARAMS returned 0x%x \n", status);
    }
    return status;
}

#endif //x86_64

static vx_status VX_CALLBACK tivxAewbProcess(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivx_obj_desc_user_data_object_t *configuration_desc;
    tivx_obj_desc_distribution_t *histogram_desc;
    tivx_obj_desc_user_data_object_t *h3a_aew_af_desc;
    tivx_obj_desc_user_data_object_t *ae_awb_result_desc;
    tivxAEWBParams *nodePrms = NULL;
#ifndef x86_64
    vx_bool test_mode = vx_false_e;
    tivx_h3a_data_t *pHh3aData;
    uint32_t viss_channel_id = 0;
#endif

    uint32_t node_param_size;
    uint8_t skipAWB = 0;
    uint8_t skipAE = 0;

#ifndef x86_64
    uint8_t rIndex = 0xFF;
    uint8_t grIndex = 0xFF;
    uint8_t gbIndex = 0xFF;
    uint8_t bIndex  = 0xFF;
#endif

    status = tivxGetTargetKernelInstanceContext(kernel, (void **)&nodePrms, &node_param_size);

    if (NULL == nodePrms->p_awb_params)
    {
        VX_PRINT(VX_ZONE_ERROR, "tivxAewbProcess : Failed to get kernel instance \n");
        status = VX_FAILURE;
    }

    if(VX_SUCCESS == status)
    {
        if ( (num_params != TIVX_KERNEL_AEWB_MAX_PARAMS)
            || (NULL == obj_desc[TIVX_KERNEL_AEWB_CONFIGURATION_IDX])
            || (NULL == obj_desc[TIVX_KERNEL_AEWB_H3A_AEW_AF_IDX])
            || (NULL == obj_desc[TIVX_KERNEL_AEWB_AE_AWB_RESULT_IDX])
        )
        {
            status = VX_FAILURE;
            VX_PRINT(VX_ZONE_ERROR, "tivxAewbProcess : Incorrect params : num_params =%d\n", num_params);
        }
    }

    if(VX_SUCCESS == status)
    {
        configuration_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_AEWB_CONFIGURATION_IDX];
        histogram_desc = (tivx_obj_desc_distribution_t *)obj_desc[TIVX_KERNEL_AEWB_HISTOGRAM_IDX];
        h3a_aew_af_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_AEWB_H3A_AEW_AF_IDX];
        ae_awb_result_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_AEWB_AE_AWB_RESULT_IDX];

        /*
           In RAW frame inject mode, VISS node indicates that H3A data is invalid.
           In this scenario, image sensor is not connected. Therefore no physical
           communication should be attempted with the image sensor
        */
        if(tivxFlagIsBitSet(obj_desc[TIVX_KERNEL_AEWB_H3A_AEW_AF_IDX]->flags, TIVX_REF_FLAG_IS_INVALID) != 0U)
        {
            VX_PRINT(VX_ZONE_INFO, "tivxAewbProcess : H3A output maybe invalid.\n");
#ifndef x86_64
            test_mode = vx_true_e;
#endif
        }

        {
            void *configuration_target_ptr;
            void *histogram_target_ptr;
            tivx_h3a_data_t *h3a_aew_af_target_ptr;
            tivx_ae_awb_params_t *ae_awb_result_prev_target_ptr;
            tivx_ae_awb_params_t *ae_awb_result_target_ptr;
            tivx_aewb_config_t * aewb_config;

            configuration_target_ptr = tivxMemShared2TargetPtr(&configuration_desc->mem_ptr);
            tivxMemBufferMap(configuration_target_ptr,
               configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
               VX_READ_ONLY);

            aewb_config = (tivx_aewb_config_t *)configuration_target_ptr;
            ae_awb_result_prev_target_ptr = &(nodePrms->ae_awb_result_prev);

#ifndef x86_64
            rIndex  = (uint8_t)nodePrms->p_awb_params->sen_awb_calc_data->red_index;
            grIndex = (uint8_t)nodePrms->p_awb_params->sen_awb_calc_data->green1_index;
            gbIndex = (uint8_t)nodePrms->p_awb_params->sen_awb_calc_data->green2_index;
            bIndex  = (uint8_t)nodePrms->p_awb_params->sen_awb_calc_data->blue_index;

            tivxMutexLock(itt_aewb_lock[aewb_config->channel_id]);
            nodePrms->p_awb_params->mode = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.awbMode;
            nodePrms->p_ae_params->mode = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.aeMode;

            if (vx_true_e == test_mode)
            {
                nodePrms->p_ae_params->mode = ALGORITHMS_ISS_AE_DISABLED;
                nodePrms->sensor_pre_wb_gain = 0;
            }

            if((-1==nodePrms->sensor_pre_wb_gain) && (ALGORITHMS_ISS_AWB_AUTO == nodePrms->p_awb_params->mode))
            {
                IssAwbDynamicParams awb_dynPrms;
                status = getWbPrgFromSensor(aewb_config->channel_id, &awb_dynPrms);
                nodePrms->sensor_pre_wb_gain = awb_dynPrms.sensor_pre_gain;
            }
#else
            nodePrms->p_awb_params->mode = aewb_config->awb_mode;
            nodePrms->p_ae_params->mode = aewb_config->ae_mode;
#endif

            ae_awb_result_target_ptr = tivxMemShared2TargetPtr(&ae_awb_result_desc->mem_ptr);
            tivxMemBufferMap(ae_awb_result_target_ptr,
               ae_awb_result_desc->mem_size, VX_MEMORY_TYPE_HOST,
               VX_WRITE_ONLY);

            if(ALGORITHMS_ISS_AWB_AUTO == nodePrms->p_awb_params->mode)
            {
                if(0 != ((nodePrms->frame_count + aewb_config->channel_id) % (aewb_config->awb_num_skip_frames+1)))
                {
                    skipAWB = 1;
                    ae_awb_result_target_ptr->awb_valid = 0;

                    ae_awb_result_target_ptr->wb_gains[0]       = ae_awb_result_prev_target_ptr->wb_gains[0];
                    ae_awb_result_target_ptr->wb_gains[1]       = ae_awb_result_prev_target_ptr->wb_gains[1];
                    ae_awb_result_target_ptr->wb_gains[2]       = ae_awb_result_prev_target_ptr->wb_gains[2];
                    ae_awb_result_target_ptr->wb_gains[3]       = ae_awb_result_prev_target_ptr->wb_gains[3];
                    ae_awb_result_target_ptr->wb_offsets[0]     = ae_awb_result_prev_target_ptr->wb_offsets[0];
                    ae_awb_result_target_ptr->wb_offsets[1]     = ae_awb_result_prev_target_ptr->wb_offsets[1];
                    ae_awb_result_target_ptr->wb_offsets[2]     = ae_awb_result_prev_target_ptr->wb_offsets[2];
                    ae_awb_result_target_ptr->wb_offsets[3]     = ae_awb_result_prev_target_ptr->wb_offsets[3];
                    ae_awb_result_target_ptr->color_temperature = ae_awb_result_prev_target_ptr->color_temperature;
                }
            }
            else if(ALGORITHMS_ISS_AWB_MANUAL == nodePrms->p_awb_params->mode)
            {
#ifndef x86_64
                ae_awb_result_target_ptr->wb_gains[rIndex] = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.rGain;
                ae_awb_result_target_ptr->wb_gains[grIndex] = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.gGain;
                ae_awb_result_target_ptr->wb_gains[gbIndex] = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.gGain;
                ae_awb_result_target_ptr->wb_gains[bIndex] = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.bGain;
                ae_awb_result_target_ptr->color_temperature = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.colorTemp;
                ae_awb_result_target_ptr->awb_valid = 1;
#endif
            }

            if(ALGORITHMS_ISS_AE_AUTO == nodePrms->p_ae_params->mode)
            {
                if(0 != ((nodePrms->frame_count + aewb_config->channel_id) % (aewb_config->ae_num_skip_frames+1)))
                {
                    skipAE = 1;
                    ae_awb_result_target_ptr->ae_valid = 0;
                    ae_awb_result_target_ptr->analog_gain       = ae_awb_result_prev_target_ptr->analog_gain;
                    ae_awb_result_target_ptr->digital_gain      = ae_awb_result_prev_target_ptr->digital_gain;
                    ae_awb_result_target_ptr->exposure_time     = ae_awb_result_prev_target_ptr->exposure_time;
                    ae_awb_result_target_ptr->ae_converged      = ae_awb_result_prev_target_ptr->ae_converged;
                }
            }
            else if(ALGORITHMS_ISS_AE_MANUAL == nodePrms->p_ae_params->mode)
            {
#ifndef x86_64
                int max_n = nodePrms->p_ae_params->exposure_program.num_ranges - 1;
                int32_t max_analog_gain = nodePrms->p_ae_params->exposure_program.analog_gain_range[max_n].max;
                int32_t min_analog_gain = nodePrms->p_ae_params->exposure_program.analog_gain_range[0].min;
                int32_t max_exp_time = nodePrms->p_ae_params->exposure_program.exposure_time_range[max_n].max;
                int32_t min_exp_time = nodePrms->p_ae_params->exposure_program.exposure_time_range[0].min;
                int32_t max_digital_gain = nodePrms->p_ae_params->exposure_program.digital_gain_range[max_n].max;
                int32_t min_digital_gain = nodePrms->p_ae_params->exposure_program.digital_gain_range[0].min;

                if(aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.analogGain > max_analog_gain)
                {
                    aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.analogGain = max_analog_gain;
                }else if(aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.analogGain < min_analog_gain)
                {
                    aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.analogGain = min_analog_gain;
                }

                if(aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.expTime > max_exp_time)
                {
                    aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.expTime = max_exp_time;
                }else if(aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.expTime < min_exp_time)
                {
                    aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.expTime = min_exp_time;
                }

                if(aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.digitalGain > max_digital_gain)
                {
                    aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.digitalGain = max_digital_gain;
                }else if(aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.digitalGain < min_digital_gain)
                {
                    aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.digitalGain = min_digital_gain;
                }

                ae_awb_result_target_ptr->analog_gain   = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.analogGain;
                ae_awb_result_target_ptr->exposure_time = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.expTime;
                ae_awb_result_target_ptr->digital_gain = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.digitalGain;
#endif
            }

            nodePrms->frame_count++;

            if((1==skipAE) && (1==skipAWB))
            {
                /*AE and AWB are both skipped. No need for H3A parsing or any other processing */
                tivxMemBufferUnmap(ae_awb_result_target_ptr,
                   ae_awb_result_desc->mem_size, VX_MEMORY_TYPE_HOST,
                    VX_WRITE_ONLY);

#ifndef x86_64
                tivxMutexUnlock(itt_aewb_lock[aewb_config->channel_id]);
#endif
                VX_PRINT(VX_ZONE_INFO, "AE and AWB are skipped. tivxAewbProcess returning SUCCESS\n");
                return VX_SUCCESS;
            }

            if( histogram_desc != NULL)
            {
                histogram_target_ptr = tivxMemShared2TargetPtr(&histogram_desc->mem_ptr);
                tivxMemBufferMap(histogram_target_ptr,
                   histogram_desc->mem_size, VX_MEMORY_TYPE_HOST,
                   VX_READ_ONLY);
            }

            /* kernel processing function start */
            if( h3a_aew_af_desc != NULL)
            {
                h3a_aew_af_target_ptr = tivxMemShared2TargetPtr(&h3a_aew_af_desc->mem_ptr);
                tivxMemBufferMap(h3a_aew_af_target_ptr, h3a_aew_af_desc->mem_size, VX_MEMORY_TYPE_HOST, VX_READ_ONLY);

#ifndef x86_64
                pHh3aData = (tivx_h3a_data_t*)h3a_aew_af_target_ptr;
                viss_channel_id = pHh3aData->channel_id;
#endif

                if (NULL == nodePrms->awb_h3a_res)
                {
                    VX_PRINT(VX_ZONE_ERROR, "tivxAewbProcess: Failed to allocate memory for awb_h3a_res\n");
#ifndef x86_64
                    tivxMutexUnlock(itt_aewb_lock[aewb_config->channel_id]);
#endif
                    return VX_ERROR_NO_MEMORY;
                }

    /*
    H3A config should be read from DCC
    Header sent by VISS node is read but AWB params are overwritten by DCC values
    */
                uint8_t *h3a_buf = NULL;
                if(NULL == nodePrms->dcc_output_params)
                {
                    h3a_buf = decode_h3a_header((uint8_t *)h3a_aew_af_target_ptr->data,
                                             &nodePrms->p_awb_params->ti_awb_data_in.frame_data.h3a_data_x,
                                             &nodePrms->p_awb_params->ti_awb_data_in.frame_data.h3a_data_y,
                                             &nodePrms->p_awb_params->ti_awb_data_in.frame_data.pix_in_pax);
                }
                else
                {
                    h3a_buf = decode_h3a_header_dcc((uint8_t *)h3a_aew_af_target_ptr->data,
                                             &nodePrms->dcc_output_params->ipipeH3A_AEWBCfg,
                                             &nodePrms->p_awb_params->ti_awb_data_in.frame_data.h3a_data_x,
                                             &nodePrms->p_awb_params->ti_awb_data_in.frame_data.h3a_data_y,
                                             &nodePrms->p_awb_params->ti_awb_data_in.frame_data.pix_in_pax);
                }

                AWB_TI_parse_H3a_buf(h3a_buf, nodePrms->p_awb_params, nodePrms->awb_h3a_res, nodePrms->p_h3a_merge);

                ae_awb_result_target_ptr->h3a_source_data = 0;

#ifndef x86_64
    /*AutoExposure is not supported for PC emulation mode*/

                if(1==skipAE)
                {
                    ae_awb_result_target_ptr->ae_valid = 0;
                }
                else
                {
                    if(ALGORITHMS_ISS_AE_AUTO == nodePrms->p_ae_params->mode)
                    {
                        if(0 == nodePrms->p_ae_params->exposure_program.num_ranges)
                        {
                            /*AE Exposure program not initialized. get it from sensor driver and re-initialize AE*/
                            IssAeDynamicParams ae_dynPrms;
                            tiae_exp_prog_t ae_exp_prg;
                            status = getExpPrgFromSensor(aewb_config->channel_id, &ae_dynPrms);
                            if(0 != status)
                            {
                                map_sensor_ae_dynparams(&ae_dynPrms, &ae_exp_prg);
                                TI_AE_init(nodePrms->p_ae_params, &ae_exp_prg);
                            }
                            else
                            {
                                ae_awb_result_target_ptr->ae_valid = 0;
                                nodePrms->p_ae_params->exposure_program.num_ranges = 0;
                                tivxMutexUnlock(itt_aewb_lock[aewb_config->channel_id]);
                                return VX_SUCCESS;
                            }
                        }
                        status = AE_TI_process(nodePrms->awb_h3a_res,
                                               nodePrms->p_awb_params->ti_awb_data_in.frame_data.h3a_data_x,
                                               nodePrms->p_awb_params->ti_awb_data_in.frame_data.h3a_data_y,
                                               nodePrms->p_ae_params,
                                               ae_awb_result_prev_target_ptr,
                                               ae_awb_result_target_ptr);
                        if(VX_SUCCESS != status)
                        {
                            VX_PRINT(VX_ZONE_ERROR, "AE_TI_process returned error 0x%x \n", status);
                            ae_awb_result_target_ptr->ae_valid = 0;
                        }
                    }

                    if(ALGORITHMS_ISS_AE_DISABLED != nodePrms->p_ae_params->mode)
                    {
                        IssSensor_ExposureParams aePrms;
                        if(1 == ae_awb_result_target_ptr->ae_valid)
                        {
                            aePrms.chId = 0;
                            aePrms.analogGain[ISS_SENSOR_EXPOSURE_LONG] = ae_awb_result_target_ptr->analog_gain;
                            aePrms.analogGain[ISS_SENSOR_EXPOSURE_SHORT] = ae_awb_result_target_ptr->analog_gain;
                            aePrms.analogGain[ISS_SENSOR_EXPOSURE_VSHORT] = ae_awb_result_target_ptr->analog_gain;
                            aePrms.exposureTime[ISS_SENSOR_EXPOSURE_LONG] = ae_awb_result_target_ptr->exposure_time;
                            aePrms.exposureTime[ISS_SENSOR_EXPOSURE_SHORT] = ae_awb_result_target_ptr->exposure_time;
                            aePrms.exposureTime[ISS_SENSOR_EXPOSURE_VSHORT] = ae_awb_result_target_ptr->exposure_time;
                            aePrms.expRatio = 1U;/*Do not care*/
                            status = sendExpGainToSensor(aewb_config, &aePrms);
                        	if(0 != status)
                            {
                            	VX_PRINT(VX_ZONE_ERROR, "Failed to send AE results to image sensor \n");
                            }
                        }
                    }
                }
#endif /*x86_64*/

                uint8_t * scratch_memory = tivxMemAlloc(sizeof(uint8_t) * AWB_SCRATCH_MEM_SIZE, TIVX_MEM_EXTERNAL);
                if (NULL == scratch_memory)
                {
                    VX_PRINT(VX_ZONE_ERROR, "Failed to allocate AWB scratch buffer \n");
#ifndef x86_64
                    tivxMutexUnlock(itt_aewb_lock[aewb_config->channel_id]);
#endif
                    return VX_ERROR_NO_MEMORY;
                }
                else
                {
                    VX_PRINT(VX_ZONE_INFO, "AWB_ScratchMemory size = %d\n", AWB_SCRATCH_MEM_SIZE);
                    memset(scratch_memory, 0xAB, AWB_SCRATCH_MEM_SIZE);
                }

                if(1==skipAWB)
                {
                    ae_awb_result_target_ptr->awb_valid = 0;
                }
                else
                {
                    if(ALGORITHMS_ISS_AWB_AUTO == nodePrms->p_awb_params->mode)
                    {
                        status = AWB_TI_process(nodePrms->p_h3a_merge, nodePrms->p_awb_params, ae_awb_result_prev_target_ptr, ae_awb_result_target_ptr, scratch_memory, nodePrms->sensor_pre_wb_gain);

                        if(VX_SUCCESS != status)
                        {
                            VX_PRINT(VX_ZONE_ERROR, "AWB_TI_process returned error 0x%x \n", status);
                            ae_awb_result_target_ptr->awb_valid = 0;
                        }

                        if (NULL != nodePrms->dcc_output_params)
                        {
                            if (100 == nodePrms->dcc_output_params->awbCalbData.apply_rgb_adjust)
                            {
                                double power = nodePrms->dcc_output_params->awbCalbData.R_adjust / 100.0;
                                double g0 = ae_awb_result_target_ptr->wb_gains[0];
                                double g1 = ae_awb_result_target_ptr->wb_gains[1];
                                double g2 = ae_awb_result_target_ptr->wb_gains[2];
                                double g3 = ae_awb_result_target_ptr->wb_gains[3];
                                ae_awb_result_target_ptr->wb_gains[0] = pow(g0 / 512.0, power) * 512 + 0.5;
                                ae_awb_result_target_ptr->wb_gains[1] = pow(g1 / 512.0, power) * 512 + 0.5;
                                ae_awb_result_target_ptr->wb_gains[2] = pow(g2 / 512.0, power) * 512 + 0.5;
                                ae_awb_result_target_ptr->wb_gains[3] = pow(g3 / 512.0, power) * 512 + 0.5;
                            }
                        }

                        VX_PRINT(VX_ZONE_INFO, "AWB Gain = (%d, %d, %d, %d) \n",
                                ae_awb_result_target_ptr->wb_gains[0],
                                ae_awb_result_target_ptr->wb_gains[1],
                                ae_awb_result_target_ptr->wb_gains[2],
                                ae_awb_result_target_ptr->wb_gains[3]);
                    }
                }
                tivxMemFree(scratch_memory, sizeof(uint8_t) * AWB_SCRATCH_MEM_SIZE, TIVX_MEM_EXTERNAL);
                tivxMemBufferUnmap(h3a_aew_af_target_ptr, h3a_aew_af_desc->mem_size, VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
            }

            /* kernel processing function complete */

            tivxMemBufferUnmap(configuration_target_ptr,
               configuration_desc->mem_size, VX_MEMORY_TYPE_HOST,
                VX_READ_ONLY);

            if( histogram_desc != NULL)
            {
                tivxMemBufferUnmap(histogram_target_ptr,
                   histogram_desc->mem_size, VX_MEMORY_TYPE_HOST,
                    VX_READ_ONLY);
            }


#ifndef x86_64
            if(1 == ae_awb_result_target_ptr->ae_valid)
            {
                aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.digitalGain = ae_awb_result_target_ptr->digital_gain;
                aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.analogGain = ae_awb_result_target_ptr->analog_gain;
                aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.expTime = ae_awb_result_target_ptr->exposure_time;
            }
            if(1 == ae_awb_result_target_ptr->awb_valid)
            {
                aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.rGain = ae_awb_result_target_ptr->wb_gains[rIndex];
                aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.gGain =
                    (ae_awb_result_target_ptr->wb_gains[grIndex] + ae_awb_result_target_ptr->wb_gains[gbIndex])/2;
                aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.bGain = ae_awb_result_target_ptr->wb_gains[bIndex];
                aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.colorTemp = ae_awb_result_target_ptr->color_temperature;
                aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.awb_valid = ae_awb_result_target_ptr->awb_valid;
                aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.awb_converged = ae_awb_result_target_ptr->awb_converged;
            }

            tivxMutexUnlock(itt_aewb_lock[aewb_config->channel_id]);

            if(1 == nodePrms->sensor_pre_wb_gain)
            {
                IssSensor_WhiteBalanceParams awbPrms;
                awbPrms.chId = aewb_config->channel_id;
                awbPrms.rGain[ISS_SENSOR_EXPOSURE_LONG] = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.rGain;
                awbPrms.gGain[ISS_SENSOR_EXPOSURE_LONG] = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.gGain;
                awbPrms.bGain[ISS_SENSOR_EXPOSURE_LONG] = aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.bGain;

                status = sendWbGainToSensor(aewb_config, &awbPrms);
            	if(0 != status)
                {
                	VX_PRINT(VX_ZONE_ERROR, "Failed to send AWB results to image sensor \n");
                }
                ae_awb_result_target_ptr->wb_gains[0] = 512;
                ae_awb_result_target_ptr->wb_gains[1] = 512;
                ae_awb_result_target_ptr->wb_gains[2] = 512;
                ae_awb_result_target_ptr->wb_gains[3] = 512;
            }

            status = sendAewbToViss(viss_channel_id, ae_awb_result_target_ptr);
        	if(0 != status)
            {
            	VX_PRINT(VX_ZONE_ERROR, "Failed to send AWB results to VISS node \n");
            }

#endif //x86_64

            memcpy(ae_awb_result_prev_target_ptr, ae_awb_result_target_ptr, sizeof(tivx_ae_awb_params_t));

            tivxMemBufferUnmap(ae_awb_result_target_ptr,
               ae_awb_result_desc->mem_size, VX_MEMORY_TYPE_HOST,
                VX_WRITE_ONLY);
        }

        VX_PRINT(VX_ZONE_INFO, "tivxAewbProcess returning %d\n", status);
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "tivxAewbProcess returning %d\n", status);
    }
    return status;
}

#ifndef x86_64
static void map_sensor_ae_dynparams(IssAeDynamicParams * p_ae_dynPrms, tiae_exp_prog_t * p_ae_exp_prg)
{
   uint8_t count;
   uint8_t num_dyn_params = p_ae_dynPrms->numAeDynParams;

   p_ae_exp_prg->num_ranges = num_dyn_params;
   p_ae_exp_prg->target_brightness = p_ae_dynPrms->targetBrightness;
   p_ae_exp_prg->target_brightness_range.min = p_ae_dynPrms->targetBrightnessRange.min;
   p_ae_exp_prg->target_brightness_range.max = p_ae_dynPrms->targetBrightnessRange.max;
   p_ae_exp_prg->exposure_time_step_size = p_ae_dynPrms->exposureTimeStepSize;
   p_ae_exp_prg->enableBLC = p_ae_dynPrms->enableBlc;

   for(count = 0; count < num_dyn_params; count++)
   {
       p_ae_exp_prg->exposure_time_range[count].min = p_ae_dynPrms->exposureTimeRange[count].min;
       p_ae_exp_prg->exposure_time_range[count].max = p_ae_dynPrms->exposureTimeRange[count].max;

       p_ae_exp_prg->analog_gain_range[count].min = p_ae_dynPrms->analogGainRange[count].min;
       p_ae_exp_prg->analog_gain_range[count].max = p_ae_dynPrms->analogGainRange[count].max;

       p_ae_exp_prg->digital_gain_range[count].min = p_ae_dynPrms->digitalGainRange[count].min;
       p_ae_exp_prg->digital_gain_range[count].max = p_ae_dynPrms->digitalGainRange[count].max;

       p_ae_exp_prg->aperture_size_range[count].min = 1;
       p_ae_exp_prg->aperture_size_range[count].max = 1;
   }
}
#endif //x86_64

static vx_status VX_CALLBACK tivxAewbCreate(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    vx_status awbInitStatus;
    awb_calc_data_t * awb_calib;
    tivx_obj_desc_user_data_object_t *dcc_desc;
    vx_status dcc_status = VX_FAILURE;
    tivxAEWBParams *nodePrms = NULL;

    tivx_aewb_config_t *aewb_config;
    tivx_obj_desc_user_data_object_t *configuration_desc;
    void *configuration_target_ptr;

#ifndef x86_64
    IssAwbDynamicParams awb_dynPrms;
#endif

    dcc_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_AEWB_AE_AWB_DCC_IDX];

    nodePrms = tivxMemAlloc(sizeof(tivxAEWBParams), TIVX_MEM_EXTERNAL);
    if(NULL == nodePrms)
    {
        VX_PRINT(VX_ZONE_ERROR, "tivxMemAlloc returned NULL \n");
        return VX_ERROR_NO_MEMORY;
    }

    nodePrms->dcc_input_params = (dcc_parser_input_params_t *)tivxMemAlloc(sizeof(dcc_parser_input_params_t), TIVX_MEM_EXTERNAL);
    nodePrms->dcc_output_params = (dcc_parser_output_params_t *)tivxMemAlloc(sizeof(dcc_parser_output_params_t), TIVX_MEM_EXTERNAL);

    if((!nodePrms->dcc_input_params) || (!nodePrms->dcc_output_params))
    {
        VX_PRINT(VX_ZONE_ERROR, "tivxMemAlloc returned NULL \n");
        return VX_ERROR_NO_MEMORY;
    }

    configuration_desc = (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_AEWB_CONFIGURATION_IDX];

    configuration_target_ptr = tivxMemShared2TargetPtr(&configuration_desc->mem_ptr);

    tivxMemBufferMap(configuration_target_ptr, configuration_desc->mem_size, VX_MEMORY_TYPE_HOST, VX_READ_ONLY);

    aewb_config = (tivx_aewb_config_t *)configuration_target_ptr;
    nodePrms->dcc_id = aewb_config->sensor_dcc_id;
    VX_PRINT(VX_ZONE_INFO, "tivxAewbCreate : DCC ID = %d \n", aewb_config->sensor_dcc_id);
    VX_PRINT(VX_ZONE_INFO, "tivxAewbCreate : Image Format = %d \n", aewb_config->sensor_img_format);
    VX_PRINT(VX_ZONE_INFO, "tivxAewbCreate : Phase = %d \n", aewb_config->sensor_img_phase);
    VX_PRINT(VX_ZONE_INFO, "tivxAewbCreate : awb_mode = %d \n", aewb_config->awb_mode);
    VX_PRINT(VX_ZONE_INFO, "tivxAewbCreate : ae_mode = %d \n", aewb_config->ae_mode);
    VX_PRINT(VX_ZONE_INFO, "tivxAewbCreate : awb_num_skip_frames = %d \n", aewb_config->awb_num_skip_frames);
    VX_PRINT(VX_ZONE_INFO, "tivxAewbCreate : ae_num_skip_frames = %d \n", aewb_config->ae_num_skip_frames);
    nodePrms->frame_count = 0;

#ifndef x86_64
    status = getWbPrgFromSensor(aewb_config->channel_id, &awb_dynPrms);
	if(0==status)
    {
    	nodePrms->sensor_pre_wb_gain = awb_dynPrms.sensor_pre_gain;
    }
	else
    {
    	nodePrms->sensor_pre_wb_gain = 0;
    }

    aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.aeMode = aewb_config->ae_mode;
    aewbCtrlPrms[aewb_config->channel_id].aewb2APrms.awbMode = aewb_config->awb_mode;

#else
    nodePrms->sensor_pre_wb_gain = 0;
#endif //x86_64


    if(NULL != dcc_desc)
    {
        nodePrms->dcc_input_params->dcc_buf_size = dcc_desc->mem_size;
        VX_PRINT(VX_ZONE_INFO, "tivxAewbCreate : DCC Numbytes = %d \n", nodePrms->dcc_input_params->dcc_buf_size);
        nodePrms->dcc_input_params->dcc_buf = tivxMemShared2TargetPtr(&dcc_desc->mem_ptr);
        if(NULL != nodePrms->dcc_input_params->dcc_buf)
        {
            tivxMemBufferMap(nodePrms->dcc_input_params->dcc_buf, dcc_desc->mem_size, VX_MEMORY_TYPE_HOST, VX_READ_ONLY);

            nodePrms->dcc_input_params->analog_gain = 1000;
            nodePrms->dcc_input_params->cameraId = aewb_config->sensor_dcc_id;
            nodePrms->dcc_input_params->color_temparature = 5000;
            nodePrms->dcc_input_params->exposure_time = 33333;

            VX_PRINT(VX_ZONE_INFO, "Before dcc_update : awb_calb_data.radius = 0x%x \n", nodePrms->dcc_output_params->awbCalbData.radius);

            dcc_status = Dcc_Create(nodePrms->dcc_output_params, NULL);
            dcc_status |= dcc_update(nodePrms->dcc_input_params, nodePrms->dcc_output_params);

            VX_PRINT(VX_ZONE_INFO, "After dcc_update : awb_calb_data.radius = 0x%x \n", nodePrms->dcc_output_params->awbCalbData.radius);
            tivxMemBufferUnmap(nodePrms->dcc_input_params->dcc_buf, dcc_desc->mem_size, VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "DCC buffer is NULL \n");
        }
    }
    else
    {
        VX_PRINT(VX_ZONE_WARNING, "No DCC buffer passed. Using defaults \n");
        return VX_ERROR_INVALID_PARAMETERS;
    }

    nodePrms->p_awb_params = tivxMemAlloc(sizeof(awbprm_t), TIVX_MEM_EXTERNAL);
    if(NULL == nodePrms->p_awb_params)
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to allocate AWB Param\n");
        return VX_ERROR_NO_MEMORY;
    }

    nodePrms->p_ae_params = tivxMemAlloc(sizeof(tiae_prm_t), TIVX_MEM_EXTERNAL);
    if(NULL == nodePrms->p_ae_params)
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to allocate AE Param\n");
        return VX_ERROR_NO_MEMORY;
    }

    nodePrms->p_awb_params->sen_awb_calc_data = tivxMemAlloc(sizeof(awb_calc_data_t), TIVX_MEM_EXTERNAL);
    if(NULL == nodePrms->p_awb_params->sen_awb_calc_data)
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to allocate AWB sensor calibration\n");
        return VX_ERROR_NO_MEMORY;
    }

    nodePrms->p_h3a_merge = tivxMemAlloc(H3A_MAX_WINH*H3A_MAX_WINV*sizeof(h3a_aewb_paxel_data_t), TIVX_MEM_EXTERNAL);
    if(NULL == nodePrms->p_h3a_merge)
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to allocate AWB H3A merge data\n");
        return VX_ERROR_NO_MEMORY;
    }
    memset(nodePrms->p_h3a_merge, 0x0, H3A_MAX_WINH*H3A_MAX_WINV*sizeof(h3a_aewb_paxel_data_t));

    if (VX_SUCCESS == dcc_status)
    {
        awb_calib = &(nodePrms->dcc_output_params->awbCalbData);
        VX_PRINT(VX_ZONE_INFO, "DCC succeeded \n");
    }
    else
    {
        awb_calib = &ts1_5_awb_calc_data;
        VX_PRINT(VX_ZONE_INFO, "DCC failed. Using default AWB calibration \n");
    }

    TI_AE_init(nodePrms->p_ae_params, NULL);

    awbInitStatus = AWB_TI_create(nodePrms->p_awb_params, awb_calib);
    if(VX_SUCCESS == awbInitStatus)
    {
        nodePrms->awb_h3a_res = tivxMemAlloc(H3A_MAX_WINH*H3A_MAX_WINV*sizeof(h3a_aewb_paxel_data_t), TIVX_MEM_EXTERNAL);
        VX_PRINT(VX_ZONE_INFO, "AWB Initialization successful \n");
        status = VX_SUCCESS;
    }
    else
    {
        status = VX_ERROR_INVALID_PARAMETERS;
        printf("AWB Initialization returned %d \n", awbInitStatus);
    }

	if(VX_SUCCESS == status)
    {
    	if(NULL != nodePrms->awb_h3a_res)
        {
        	tivxSetTargetKernelInstanceContext(kernel, nodePrms, sizeof(tivxAEWBParams));
        	status = VX_SUCCESS;
        	VX_PRINT(VX_ZONE_INFO, "AWB Initialization successful \n");
        }
    	else
        {
        	status = VX_ERROR_NO_MEMORY;
        	VX_PRINT(VX_ZONE_ERROR, "Failed to allocate H3A buffer \n");
        }
    }
    tivxMemBufferUnmap(configuration_target_ptr, configuration_desc->mem_size,
        VX_MEMORY_TYPE_HOST, VX_READ_ONLY);

    return status;
}

static vx_status VX_CALLBACK tivxAewbDelete(
       tivx_target_kernel_instance kernel,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivxAEWBParams* nodePrms = NULL;
    uint32_t size;
    status = tivxGetTargetKernelInstanceContext(kernel, (void **)&nodePrms, &size);

    if(VX_SUCCESS == status)
    {
    if(NULL != nodePrms->dcc_input_params)
    {
        tivxMemFree(nodePrms->dcc_input_params, sizeof(dcc_parser_input_params_t), TIVX_MEM_EXTERNAL);
    }
    if(NULL != nodePrms->dcc_output_params)
    {
        tivxMemFree(nodePrms->dcc_output_params, sizeof(dcc_parser_output_params_t), TIVX_MEM_EXTERNAL);
    }
    if(NULL != nodePrms->p_h3a_merge)
    {
        tivxMemFree(nodePrms->p_h3a_merge, H3A_MAX_WINH*H3A_MAX_WINV*sizeof(h3a_aewb_paxel_data_t), TIVX_MEM_EXTERNAL);
    }
    if(NULL != nodePrms->p_ae_params)
    {
        tivxMemFree(nodePrms->p_ae_params, sizeof(tiae_prm_t), TIVX_MEM_EXTERNAL);
    }
    if(NULL != nodePrms->p_awb_params)
    {
        if(NULL != nodePrms->p_awb_params->sen_awb_calc_data)
        {
            tivxMemFree(nodePrms->p_awb_params->sen_awb_calc_data, sizeof(awb_calc_data_t), TIVX_MEM_EXTERNAL);
        }
        tivxMemFree(nodePrms->p_awb_params, size, TIVX_MEM_EXTERNAL);
    }
        if(NULL != nodePrms->awb_h3a_res)
        {
            tivxMemFree(nodePrms->awb_h3a_res, H3A_MAX_WINH*H3A_MAX_WINV*sizeof(h3a_aewb_paxel_data_t), TIVX_MEM_EXTERNAL);
        }
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "tivxAewbDelete : Error in getting kernel instance \n");
    }
    return status;
}

static vx_status VX_CALLBACK tivxAewbControl(
       tivx_target_kernel_instance kernel,
       uint32_t node_cmd_id,
       tivx_obj_desc_t *obj_desc[],
       uint16_t num_params,
       void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    tivxAEWBParams* nodePrms = NULL;
    uint32_t size;
    tivx_obj_desc_user_data_object_t * dcc_udo;
    status = tivxGetTargetKernelInstanceContext(kernel, (void **)&nodePrms, &size);

    if(ISS_CMD_SET_DCC_PARAMS == node_cmd_id)
    {
        if(NULL != obj_desc)
        {
            void *target_ptr;
            dcc_parser_input_params_t dcc_live_input_params;
            dcc_udo = (tivx_obj_desc_user_data_object_t *)(obj_desc[0U]);
            target_ptr = tivxMemShared2TargetPtr(&dcc_udo->mem_ptr);
            tivxMemBufferMap(target_ptr, dcc_udo->mem_size, (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY);

            memcpy(&dcc_live_input_params, nodePrms->dcc_input_params, sizeof(dcc_parser_input_params_t));
            dcc_live_input_params.dcc_buf = (uint8_t *)target_ptr;
            dcc_live_input_params.dcc_buf_size = dcc_udo->mem_size;
            status |= dcc_update(&dcc_live_input_params, nodePrms->dcc_output_params);

            tivxMemBufferUnmap(target_ptr, dcc_udo->mem_size, (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY);
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "obj_desc is NULL \n");
        }
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Unknown node_cmd_id=0x%x \n", node_cmd_id);
    }

    return status;
}

void tivxAddTargetKernelAewb(void)
{
    vx_status status = VX_FAILURE;
    char target_name[TIVX_TARGET_MAX_NAME];
    vx_enum self_cpu;

    self_cpu = tivxGetSelfCpuId();

    if ( self_cpu == TIVX_CPU_ID_IPU1_0 )
    {
        strncpy(target_name, TIVX_TARGET_IPU1_0, TIVX_TARGET_MAX_NAME);
        status = VX_SUCCESS;
    }
    else if ( self_cpu == TIVX_CPU_ID_IPU1_1 )
    {
        strncpy(target_name, TIVX_TARGET_IPU1_1, TIVX_TARGET_MAX_NAME);
        status = VX_SUCCESS;
    }
    else
    {
        status = VX_FAILURE;
    }

    if (status == VX_SUCCESS)
    {
        vx_aewb_target_kernel = tivxAddTargetKernelByName(
                            TIVX_KERNEL_AEWB_NAME,
                            target_name,
                            tivxAewbProcess,
                            tivxAewbCreate,
                            tivxAewbDelete,
                            tivxAewbControl,
                            NULL);
    }
}

void tivxRemoveTargetKernelAewb(void)
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_aewb_target_kernel);
    if (status == VX_SUCCESS)
    {
        vx_aewb_target_kernel = NULL;
    }
}

static void parse_h3a_out(uint8_t h3a_buf[], int32_t n_col, int32_t n_row,
                          int32_t id_r, int32_t id_gr, int32_t id_gb, int32_t id_b,
                          h3a_aewb_paxel_data_t *h3a_data)
{
    uint8_t * cur_addr = h3a_buf;
    int n_win = 0;
    int j, i;

    for (j = 0; j < n_row; j++)
    {
        for (i = 0; i < n_col; i++)
        {
            uint16_t * pAewbWinData = (uint16_t *)cur_addr;

            uint32_t sum_gr = pAewbWinData[id_gr];
            uint32_t sum_gb = pAewbWinData[id_gb];
            uint16_t sum_g  = (uint16_t)((sum_gr + sum_gb + 1) >> 1);

            h3a_data[j * n_col + i].green = sum_g;
            h3a_data[j * n_col + i].red   = pAewbWinData[id_r];
            h3a_data[j * n_col + i].blue  = pAewbWinData[id_b];

            cur_addr += sizeof(uint16_t) * 8;
            n_win++;

            if (n_win % 8 == 0)
            {
                cur_addr += sizeof(uint16_t) * 8;
            }
        }

        if ((cur_addr - h3a_buf) % 32 == 16)
        {
            cur_addr += 16;
        }
    }
}

static void h3a_merge(
    h3a_aewb_paxel_data_t *p_in,
    int32_t sz_h,
    int32_t sz_v,
    int32_t pix_in_pax,
    int32_t T_low,
    int32_t T_high,
    h3a_aewb_paxel_data_t *p_out)
{
    int32_t th1 = pix_in_pax * T_low;
    int32_t th2 = pix_in_pax * T_high;
    int k;
    for (k = 0; k < sz_v * sz_h; k++)
    {
        if (p_in[k].green >= th1 && p_in[k].green <= th2)
        {
            p_out[k] = p_in[k];
        }
        else
        {
            p_out[k].red   = (p_out[k].red   * 7) >> 3;
            p_out[k].green = (p_out[k].green * 7) >> 3;
            p_out[k].blue  = (p_out[k].blue  * 7) >> 3;
        }
    }
}

static void h3a_normalize(h3a_aewb_paxel_data_t p_h3a[],
                          int32_t sz_h, int32_t sz_v, int32_t pix_in_pax)
{
    int k;
    for (k = 0; k < sz_v * sz_h; k++)
    {
        int32_t r = p_h3a[k].red;
        int32_t g = p_h3a[k].green;
        int32_t b = p_h3a[k].blue;
        p_h3a[k].red   = (uint16_t)((r + (pix_in_pax >> 1)) / pix_in_pax);
        p_h3a[k].green = (uint16_t)((g + (pix_in_pax >> 1)) / pix_in_pax);
        p_h3a[k].blue  = (uint16_t)((b + (pix_in_pax >> 1)) / pix_in_pax);
    }
}

static vx_status AWB_TI_parse_H3a_buf(uint8_t * pH3a_out, awbprm_t * p_awb_params,
        h3a_aewb_paxel_data_t * awb_h3a_res, h3a_aewb_paxel_data_t * p_h3a_merge)
{
    int32_t h3a_n_col = p_awb_params->ti_awb_data_in.frame_data.h3a_data_x;
    int32_t h3a_n_row = p_awb_params->ti_awb_data_in.frame_data.h3a_data_y;
    int32_t h3a_n_pix = p_awb_params->ti_awb_data_in.frame_data.pix_in_pax;
    int32_t rIndex    = p_awb_params->sen_awb_calc_data->red_index;
    int32_t grIndex   = p_awb_params->sen_awb_calc_data->green1_index;
    int32_t gbIndex   = p_awb_params->sen_awb_calc_data->green2_index;
    int32_t bIndex    = p_awb_params->sen_awb_calc_data->blue_index;
    VX_PRINT(VX_ZONE_INFO, "H3A color: r=%d, gr=%d, gb=%d, b=%d \n", rIndex, grIndex, gbIndex, bIndex);

    parse_h3a_out(pH3a_out, h3a_n_col, h3a_n_row, rIndex, grIndex, gbIndex, bIndex, awb_h3a_res);

    // "0, 1023" below: no merge for 10-bit H3A without H3A switching
    h3a_merge(awb_h3a_res, h3a_n_col, h3a_n_row, h3a_n_pix, 0, 1023, p_h3a_merge);

    h3a_normalize(awb_h3a_res, h3a_n_col, h3a_n_row, h3a_n_pix);

    return VX_SUCCESS;
}

static vx_status AWB_TI_create(awbprm_t *p_awb_params, awb_calc_data_t* p_calib)
{
    vx_status status = VX_SUCCESS;
    TI_AWB_ERROR awbInitStatus;

    if((NULL == p_calib) || (NULL == p_awb_params))
    {
        printf("AWB_TI_create Error : Invalid parameters \n");
        status = VX_ERROR_INVALID_PARAMETERS;
    }
    else
    {
        p_awb_params->init_done = 0;
        p_awb_params->mode = AWB_WB_MODE_AUTO;
        p_awb_params->AWB_ScratchMemory = NULL;
        p_awb_params->manl_tmpr = 0;
        p_awb_params->sb_total_exp = 999999;

        *p_awb_params->sen_awb_calc_data = *p_calib;

        awbInitStatus = TI_AWB_init(p_awb_params);
        if(TI_AWB_ERROR_OK == awbInitStatus)
        {
            status = VX_SUCCESS;
        }
        else
        {
            printf("ERROR : AWB Initialization returned %d \n", awbInitStatus);
            status = VX_ERROR_INVALID_PARAMETERS;
        }
    }

    return status;
}

static vx_status AWB_TI_process(
        h3a_aewb_paxel_data_t * awb_h3a_res,
        awbprm_t              * p_awb_params,
        tivx_ae_awb_params_t  * aewb_prev,
        tivx_ae_awb_params_t  * aewb_result,
        uint8_t               * scratch_mem,
        uint8_t                 sensor_pre_gain)
{
    p_awb_params->ti_awb_data_in.frame_data.h3a_res = awb_h3a_res;
    p_awb_params->AWB_ScratchMemory = scratch_mem;
    p_awb_params->ti_awb_data_in.is_face.faces = NULL;      // Not supported
    vx_status status = VX_FAILURE;

    uint8_t rIndex  = (uint8_t)p_awb_params->sen_awb_calc_data->red_index;
    uint8_t grIndex = (uint8_t)p_awb_params->sen_awb_calc_data->green1_index;
    uint8_t gbIndex = (uint8_t)p_awb_params->sen_awb_calc_data->green2_index;
    uint8_t bIndex  = (uint8_t)p_awb_params->sen_awb_calc_data->blue_index;

    if (1==sensor_pre_gain)
    {
        uint16_t paxelIndex = 0;
        h3a_aewb_paxel_data_t h3a_paxel_data;
        awb_frame_data_t    * pFrame_data = &(p_awb_params->ti_awb_data_in.frame_data);
        int n_pax = pFrame_data->h3a_data_x * pFrame_data->h3a_data_y;
        uint32_t rGain_prev, bGain_prev, gGain_prev;

        rGain_prev = aewb_prev->wb_gains[rIndex];
        gGain_prev = (aewb_prev->wb_gains[grIndex]+aewb_prev->wb_gains[gbIndex])/2;
        bGain_prev = aewb_prev->wb_gains[bIndex];
        for (paxelIndex = 0; paxelIndex < n_pax; paxelIndex++)
        {
            h3a_paxel_data = pFrame_data->h3a_res[paxelIndex];
            h3a_paxel_data.red   /= rGain_prev;
            h3a_paxel_data.green /= gGain_prev;
            h3a_paxel_data.blue  /= bGain_prev;
        }
    }

    awb_data_out_t  awb_data_out;
    TI_AWB_ERROR awbStatus;
    awbStatus = TI_AWB_do(p_awb_params, &awb_data_out);
    if(TI_AWB_ERROR_OK == awbStatus)
    {
        TI_AWB_stab(p_awb_params, &awb_data_out);

        aewb_result->wb_gains[rIndex] = awb_data_out.gain_R << 1;
        aewb_result->wb_gains[grIndex] = awb_data_out.gain_Gr << 1;
        aewb_result->wb_gains[gbIndex] = awb_data_out.gain_Gb << 1;
        aewb_result->wb_gains[bIndex] = awb_data_out.gain_B << 1;

        aewb_result->wb_offsets[rIndex] = 0;
        aewb_result->wb_offsets[grIndex] = 0;
        aewb_result->wb_offsets[gbIndex] = 0;
        aewb_result->wb_offsets[bIndex] = 0;

        aewb_result->color_temperature = awb_data_out.color_temperature_estim;
        aewb_result->awb_valid = 1;
        status = VX_SUCCESS;
    }
    else
    {
        aewb_result->awb_valid = 0;
        VX_PRINT(VX_ZONE_ERROR, "TI_AWB_do Returned error %d \n", awbStatus);
        status = VX_FAILURE;
    }

    p_awb_params->AWB_ScratchMemory = NULL;
    return status;
}

