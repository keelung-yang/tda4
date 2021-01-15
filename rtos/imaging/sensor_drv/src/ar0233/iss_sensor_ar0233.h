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
#include <iss_sensors.h>
#include <iss_sensor_priv.h>
#include <iss_sensor_if.h>
#include <iss_sensor_serdes.h>
/**< ISS AEWB plugin is included here to get the default AEWB configuration
     from each sensor */

#define AR0233_OUT_WIDTH   (1920)
#define AR0233_OUT_HEIGHT  (1280)

#include "ar0233_linear_config.h"

#define AR0233_LINE_LEN_PCK (0x073A)

#include "ar0233_wdr_config.h"
#include "ar0233_gain_table.h"

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */

#define ISS_SENSOR_AR0233_DEFAULT_EXP_RATIO     (128U)

#define ISS_SENSOR_AR0233_FEATURES      (ISS_SENSOR_FEATURE_MANUAL_EXPOSURE|   \
                                         ISS_SENSOR_FEATURE_MANUAL_GAIN|       \
                                         ISS_SENSOR_FEATURE_LINEAR_MODE|       \
                                         ISS_SENSOR_FEATURE_COMB_COMP_WDR_MODE|       \
                                         ISS_SENSOR_FEATURE_DCC_SUPPORTED)

#define AR0233_CHIP_ID_REG_ADDR               (0x3000)
#define AR0233_CHIP_ID_REG_VAL                (0x956)
#define SENSOR_AR0233_EXP_T1                  (0x3012U)
#define SENSOR_AR0233_MIN_EXP_ROWS            (0x1)
#define SENSOR_AR0233_MAX_EXP_ROWS            (0x140)
#define SENSOR_AR0233_ANALOG_COARSE_GAIN      (0x3366U)
#define SENSOR_AR0233_ANALOG_FINE_GAIN        (0x336AU)
#define SENSOR_AR0233_DIGITAL_GAIN            (0x3308U)

#define SENSOR_AR0233_ANALOG_GAIN_T1_MASK   (0x000FU)
#define SENSOR_AR0233_ANALOG_GAIN_T2_MASK   (0x00F0U)
#define SENSOR_AR0233_ANALOG_GAIN_T3_MASK   (0x0F00U)
#define SENSOR_AR0233_ANALOG_GAIN_T4_MASK   (0xF000U)

#define SENSOR_AR0233_ANALOG_GAIN_T1T2T3_MASK   (0x0FFFU)
#define SENSOR_AR0233_ANALOG_GAIN_T1T2T3T4_MASK (0xFFFFU)

#define AR0233_LINE_LEN_TIME_NS             (26040U)

#define AR0233_GREEN1_GAIN_REG             (0x3056U)
#define AR0233_BLUE_GAIN_REG               (0x3058U)
#define AR0233_RED_GAIN_REG                (0x305AU)
#define AR0233_GREEN2_GAIN_REG             (0x305CU)

/*******************************************************************************
 *  Data structure's
 *******************************************************************************
 */

struct {

    uint32_t                    maxCoarseIntgTime;
    /**< Max Coarse integration time in milliseconds supported by sensor */
    uint32_t                     lineIntgTime;
    /**< Line Integration time in microseconds */
    uint32_t                     pixIntgTime;
    /**< Pixel Integration time in microseconds  */
} gImx390DeviceObj;

/*******************************************************************************
 *  Local Functions Declarations
 *******************************************************************************
 */

static int32_t AR0233_Probe(uint32_t chId, void *pSensorHdl);
static int32_t AR0233_Config(uint32_t chId, void *pSensorHdl, uint32_t sensor_features_requested);
static int32_t AR0233_StreamOn(uint32_t chId, void *pSensorHdl);
static int32_t AR0233_StreamOff(uint32_t chId, void *pSensorHdl);
static int32_t AR0233_PowerOn(uint32_t chId, void *pSensorHdl);
static int32_t AR0233_PowerOff(uint32_t chId, void *pSensorHdl);
static int32_t AR0233_GetExpParams(uint32_t chId, void *pSensorHdl, IssSensor_ExposureParams *pExpPrms);
static int32_t AR0233_SetAeParams(void *pSensorHdl, uint32_t chId, IssSensor_ExposureParams *pExpPrms);
static int32_t AR0233_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms);
static void AR0233_InitAewbConfig(uint32_t chId, void *pSensorHdl);
static void AR0233_GetIspConfig (uint32_t chId, void *pSensorHdl);
static void AR0233_deinit (uint32_t chId, void *pSensorHdl);
static int32_t AR0233_ReadWriteReg (uint32_t chId, void *pSensorHdl, uint32_t readWriteFlag, I2cParams *pReg);
static int32_t AR0233_GetExpPrgFxn(uint32_t chId, void *pSensorHdl, IssAeDynamicParams *p_ae_dynPrms);
static int32_t AR0233_GetWBPrgFxn(uint32_t chId, void *pSensorHdl, IssAwbDynamicParams *p_awb_dynPrms);
static int32_t AR0233_SetAwbParams(void *pSensorHdl, uint32_t chId, IssSensor_WhiteBalanceParams *pWbPrms);

#ifdef _AR0233_DEBUG_
int32_t AR0233_Debug(uint32_t chId, void *pSensorHdl);
#endif

static int32_t AR0233_WriteReg(uint8_t    i2cInstId,
                             uint8_t      i2cAddr,
                             uint16_t     regAddr,
                             uint16_t     regVal,
                             uint32_t     numRegs);

static int32_t AR0233_ReadReg(uint8_t     i2cInstId,
                            uint8_t         i2cAddr,
                            uint16_t        regAddr,
                            uint16_t         *regVal,
                            uint32_t        numRegs);

