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

#define AR0820_OUT_WIDTH   (3840)
#define AR0820_OUT_HEIGHT  (2160)

#include "ar0820_linear_config.h"
#include "ar0820_wdr_config.h"

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */

#define ISS_SENSOR_AR0820_FEATURES      (ISS_SENSOR_FEATURE_MANUAL_EXPOSURE|   \
                                         ISS_SENSOR_FEATURE_MANUAL_GAIN|       \
                                         ISS_SENSOR_FEATURE_LINEAR_MODE|       \
                                         ISS_SENSOR_FEATURE_COMB_COMP_WDR_MODE|       \
                                         ISS_SENSOR_FEATURE_DCC_SUPPORTED)

#define AR0820_CHIP_ID_REG_ADDR             (0x3000)
#define AR0820_CHIP_ID_REG_VAL              (0x557)
#define SENSOR_AR0820_EXP_COARSE            (0x3012U)
#define SENSOR_AR0820_CLK_PIX_FREQ_MHz      (156)
#define SENSOR_AR0820_LINE_PCK              (0x440)
#define SENSOR_AR0820_ROWTIME_NS            ((SENSOR_AR0820_LINE_PCK*1000)/SENSOR_AR0820_CLK_PIX_FREQ_MHz)
#define SENSOR_AR0820_MAX_EXP_ROWS          (AR0820_OUT_HEIGHT-1)
#define SENSOR_AR0820_COARSE_ANALOG_GAIN    (0x3366U)
#define SENSOR_AR0820_FINE_ANALOG_GAIN      (0x336AU)
#define SENSOR_AR0820_ANALOG_GAIN_T1_MASK   (0x000FU)
#define SENSOR_AR0820_ANALOG_GAIN_T2_MASK   (0x00F0U)
#define SENSOR_AR0820_ANALOG_GAIN_T3_MASK   (0x0F00U)
#define SENSOR_AR0820_ANALOG_GAIN_T4_MASK   (0xF000U)
#define AR0820_GROUP_HOLD_REG_ADDR          (0x3022)

#define SENSOR_AR0820_ANALOG_GAIN_T1T2T3_MASK (0x0FFFU)

#define SENSOR_AR0820_EXP_T1            (0x2020U)
#define SENSOR_AR0820_EXP_T2            (0x2022U)
#define SENSOR_AR0820_EXP_T3            (0x2024U)


/*AR0820 Timing parameters for AutoExposure Control fro WDR mode
Sensitive to sensor configuration
*/

/*Fine Integration Time Reg 0x3014*/
#define FIT (0x0)
/*Middle Integration Time Reg 0x32F6*/
#define MIT (0x1)
/*Timing parameter*/
#define LLP (1480U)
/*Pixel clock in Hz*/
#define PCLK (125000U)
/*Number of exposures*/
#define NUM_EXP (3U)
/*Number of ADCs*/
#define NUM_ADC (4U)
/*T1:T2 Exp Ratio*/
#define T1_T2_EXP_RATIO (11U)
/*T1:T3 Exp Ratio*/
#define T1_T3_EXP_RATIO (92U)
/*Number of rows in frame Reg 0x300A*/
#define FRAME_LEN_LINES (2296U)


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

static int32_t AR0820_Probe(uint32_t chId, void *pSensorHdl);
static int32_t AR0820_Config(uint32_t chId, void *pSensorHdl, uint32_t sensor_features_requested);
static int32_t AR0820_StreamOn(uint32_t chId, void *pSensorHdl);
static int32_t AR0820_StreamOff(uint32_t chId, void *pSensorHdl);
static int32_t AR0820_PowerOn(uint32_t chId, void *pSensorHdl);
static int32_t AR0820_PowerOff(uint32_t chId, void *pSensorHdl);
static int32_t AR0820_GetExpParams(uint32_t chId, void *pSensorHdl, IssSensor_ExposureParams *pExpPrms);
static int32_t AR0820_SetAeParams(void *pSensorHdl, uint32_t chId, IssSensor_ExposureParams *pExpPrms);
static int32_t AR0820_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms);
static void AR0820_InitAewbConfig(uint32_t chId, void *pSensorHdl);
static void AR0820_GetIspConfig (uint32_t chId, void *pSensorHdl);
static void AR0820_deinit (uint32_t chId, void *pSensorHdl);
static int32_t AR0820_ReadWriteReg (uint32_t chId, void *pSensorHdl, uint32_t readWriteFlag, I2cParams *pReg);
static int32_t AR0820_GetExpPrgFxn(uint32_t chId, void *pSensorHdl, IssAeDynamicParams *p_ae_dynPrms);

#ifdef _AR0820_DEBUG_
int32_t AR0820_Debug(uint32_t chId, void *pSensorHdl);
#endif

static int32_t AR0820_WriteReg(uint8_t    i2cInstId,
                             uint8_t      i2cAddr,
                             uint16_t     regAddr,
                             uint16_t     regVal,
                             uint32_t     numRegs);

static int32_t AR0820_ReadReg(uint8_t     i2cInstId,
                            uint8_t         i2cAddr,
                            uint16_t        regAddr,
                            uint16_t         *regVal,
                            uint32_t        numRegs);

