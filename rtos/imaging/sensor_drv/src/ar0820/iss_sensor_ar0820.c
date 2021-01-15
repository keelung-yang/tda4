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
#include "iss_sensor_ar0820.h"
#include "ar0820_serdes_config.h"

#define AR0820_MAX_COARSE_AGAIN (8U)
#define AR0820_MAX_FINE_AGAIN (2U)

static IssSensor_CreateParams  ar0820CreatePrms = {
    SENSOR_ONSEMI_AR0820_UB953_LI,     /*sensor name*/
    0x6,                             /*i2cInstId*/
    {SENSOR_0_I2C_ALIAS, 0, 0, 0, 0, 0, 0, 0},   /*i2cAddrSensor*/
    {SER_0_I2C_ALIAS, 0, 0, 0, 0, 0, 0, 0},      /*i2cAddrSer*/
    /*IssSensor_Info*/
    {
        {
            AR0820_OUT_WIDTH,               /*width*/
            AR0820_OUT_HEIGHT,              /*height*/
            1,                              /*num_exposures*/
            vx_false_e,                     /*line_interleaved*/
            {
                {TIVX_RAW_IMAGE_16_BIT, 11},    /*dataFormat and MSB [0]*/
            },
            0,                              /*meta_height_before*/
            0,                              /*meta_height_after*/
        },
        ISS_SENSOR_AR0820_FEATURES,     /*features*/
        ALGORITHMS_ISS_AEWB_MODE_AEWB,  /*aewbMode*/
        30,                             /*fps*/
        4,                              /*numDataLanes*/
        {1, 2, 3, 4},                   /*dataLanesMap*/
        {0, 0, 0, 0},                   /*dataLanesPolarity*/
        800,                            /*CSI Clock*/
    },
    1,                                  /*numChan*/
    820,                                /*dccId*/
};

static IssSensorFxns           ar0820SensorFxns = {
    AR0820_Probe,
    AR0820_Config,
    AR0820_StreamOn,
    AR0820_StreamOff,
    AR0820_PowerOn,
    AR0820_PowerOff,
    AR0820_GetExpParams,
    AR0820_SetAeParams,
    AR0820_GetDccParams,
    AR0820_InitAewbConfig,
    AR0820_GetIspConfig,
    AR0820_ReadWriteReg,
    AR0820_GetExpPrgFxn,
    AR0820_deinit,
    NULL,
    NULL
};

static IssSensorIntfParams     ar0820SensorIntfPrms = {
    0,                                /*isMultiChannel*/
    4,                              /*numCSI2Lanes*/
    1,                              /*inCsi2VirtualChanNum*/
    1,                /* isCplxCfgValid */
     {
        {0, 1}, /* Clock Lane */
        {0, 2}, /* data1Lane */
        {0, 3}, /* data2Lane */
        {0, 4}, /* data3Lane*/
        {0, 5}, /* data4Lane */
    },
    800,                 /* csi2PhyClk */ 
    0,             /*sensorBroadcast*/
    0,             /*enableFsin*/
};

IssSensorConfig     ar0820SensorRegConfigLinear = {
    ub960DesCfg_AR0820,     /*desCfgPreScript*/
    ub953SerCfg_AR0820,     /*serCfgPreScript*/
    ar0820LinearConfig,        /*sensorCfgPreScript*/
    ub960AR0820DesCSI2Enable,        /*desCfgPostScript*/
    NULL,                    /*serCfgPostScript*/
    NULL,                    /*sensorCfgPostScript*/
};

IssSensorConfig     ar0820SensorRegConfigWdr = {
    ub960DesCfg_AR0820,     /*desCfgPreScript*/
    ub953SerCfg_AR0820,     /*serCfgPreScript*/
    ar0820WdrConfig,        /*sensorCfgPreScript*/
    ub960AR0820DesCSI2Enable,        /*desCfgPostScript*/
    NULL,                    /*serCfgPostScript*/
    NULL,                    /*sensorCfgPostScript*/
};

IssSensors_Handle ar0820SensorHandle = {
    1,                                 /*isUsed*/
    &ar0820CreatePrms,                /*CreatePrms*/
    &ar0820SensorFxns,                /*SensorFxns*/
    &ar0820SensorIntfPrms,            /*SensorIntfPrms*/
};

/*
 * \brief DCC Parameters of AR0820
 */
IssCapture_CmplxIoLaneCfg           ar0820Csi2CmplxIoLaneCfg;

extern IssSensors_Handle * gIssSensorTable[ISS_SENSORS_MAX_SUPPORTED_SENSOR];

int32_t IssSensor_AR0820_Init()
{
    int32_t status;
    status = IssSensor_Register(&ar0820SensorHandle);
    if(0 != status)
    {
        printf("IssSensor_AR0820_Init failed \n");
    }

    return status;
}

/*******************************************************************************
 *  Local Functions Definition
 *******************************************************************************
 */

static int32_t AR0820_Probe(uint32_t chId, void *pSensorHdl)
{
    int32_t status = -1;
    uint32_t i2cInstId;
    uint8_t sensorI2cAddr;
    uint16_t chipIdRegAddr = AR0820_CHIP_ID_REG_ADDR;
    uint16_t chipIdRegValueRead = 0xABCD;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    uint8_t count=0;
    uint8_t max_retries = 5;

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    i2cInstId = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];

    /*Read chip ID to detect if the sensor can be detected*/

    while( (chipIdRegValueRead != AR0820_CHIP_ID_REG_VAL) && (count < max_retries))
    {
        status = AR0820_ReadReg(i2cInstId, sensorI2cAddr, chipIdRegAddr, &chipIdRegValueRead, 1U);
        if(status == 0)
        {
            if(chipIdRegValueRead == AR0820_CHIP_ID_REG_VAL)
            {
                printf("AR0820_Probe SUCCESS : Read expected value 0x%x at chip ID register 0x%x \n", AR0820_CHIP_ID_REG_VAL, chipIdRegAddr);
            }
            else
            {
                status = -1;
                printf("AR0820_Probe : 0x%x read at chip ID register 0x%x. Expected 0x%x \n", chipIdRegValueRead, chipIdRegAddr, AR0820_CHIP_ID_REG_VAL);
                printf("AR0820 Probe Failed \n");
                appLogWaitMsecs(100);
            }
        }
        else
        {
            printf("AR0820_Probe Error : Failed reading CHIP_ID register 0x%x \n", chipIdRegAddr);
        }
        count++;
    }

    if(0 == status)
    {
        chipIdRegAddr = 0x31FE;/*Sensor Revision*/
        status = AR0820_ReadReg(i2cInstId, sensorI2cAddr, chipIdRegAddr, &chipIdRegValueRead, 1U);
        if(status == 0)
        {
            printf("AR0820 Sensor Revision ID = 0x%x \n", chipIdRegValueRead);
        }
        else
        {
            printf("AR0820_Probe Error : Failed reading Rev ID register 0x%x \n", chipIdRegAddr);
        }
    }
    return (status);
}

/*Flag indicating operating mode 0:Linear, 1:WDR*/
static uint8_t ar0820_op_mode = 0;

static int32_t AR0820_Config(uint32_t chId, void *pSensorHdl, uint32_t sensor_features_requested)
{
    int32_t status = 0;
    uint32_t regCnt, i2cInstId;
    uint8_t sensorI2cAddr;
    uint16_t regAddr;
    uint16_t regValue;
    uint16_t delayMilliSec;
    uint16_t sensor_cfg_script_len = 0;
    I2cParams *sensorCfg = NULL;
    I2cParams *deserCfg = NULL;
    I2cParams *serCfg = NULL;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    int8_t ub960InstanceId = getUB960InstIdFromChId(chId);

    if(ub960InstanceId < 0)
    {
        printf("Invalid ub960InstanceId \n");
        return 0xFF;
    }

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    if(sensor_features_requested != (sensor_features_requested & ISS_SENSOR_AR0820_FEATURES))
    {
        printf("AR0820_Config : Error. feature set 0x%x is not supported \n", sensor_features_requested);
        return -1;
    }
    i2cInstId = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];

    if(sensor_features_requested & ISS_SENSOR_FEATURE_COMB_COMP_WDR_MODE)
    {
        deserCfg = ar0820SensorRegConfigWdr.desCfgPreScript;
        serCfg = ar0820SensorRegConfigWdr.serCfgPreScript;
        sensorCfg = ar0820SensorRegConfigWdr.sensorCfgPreScript;
        sensor_cfg_script_len = AR0820_WDR_CONFIG_SIZE;
        ar0820_op_mode = 1U;
    }else
    {
        deserCfg = ar0820SensorRegConfigLinear.desCfgPreScript;
        serCfg = ar0820SensorRegConfigLinear.serCfgPreScript;
        sensorCfg = ar0820SensorRegConfigLinear.sensorCfgPreScript;
        sensor_cfg_script_len = AR0820_LINEAR_CONFIG_SIZE;
        ar0820_op_mode = 0U;
    }

    if(NULL != deserCfg)
    {
        status = ub960_cfgScript(deserCfg, ub960InstanceId);
    }
    if(0 == status)
    {
        appLogWaitMsecs(50);

    /*The code assumes that I2C instance is the same for sensor and serializer*/
        if(NULL != serCfg)
        {
            status = ub953_cfgScript(i2cInstId, pCreatePrms->i2cAddrSer[chId], serCfg);
        }
        if(0 == status)
        {
            appLogWaitMsecs(50);
    
#ifndef _TEST_PATTERN_ENABLE_
            if(NULL != sensorCfg)
            {
                printf(" Configuring AR0820 imager .. Please wait till it finishes \n");
                for(regCnt=0; regCnt<sensor_cfg_script_len; regCnt++)
                {
                    regAddr  = sensorCfg[regCnt].nRegAddr;
                    regValue = sensorCfg[regCnt].nRegValue;
                    delayMilliSec = sensorCfg[regCnt].nDelay;
    
                    status = AR0820_WriteReg(i2cInstId, sensorI2cAddr, regAddr, regValue, 1u);
                    if (0 != status)
                    {
                        printf(" \n \n AR0820: Sensor Reg Write Failed for regAddr 0x%x \n \n", regAddr);
                    }
    
                    if (delayMilliSec > 0)
                    {
                        appLogWaitMsecs(delayMilliSec);
                    }
                }
                /*Wait 10ms after the init is done*/
                appLogWaitMsecs(10);
                printf(" AR0820 config done\n");
            }
            else
            {
                printf(" AR0820 config script is NULL \n");
            }
#endif

        }

    }

    return (status);
}

static int32_t AR0820_StreamOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    int8_t ub960InstanceId = getUB960InstIdFromChId(chId);

    if(ub960InstanceId < 0)
    {
        printf("Invalid ub960InstanceId \n");
        return 0xFF;
    }

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    /*Start Streaming from sensor*/
    status = AR0820_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], 0x301A, 0x005C, 1u);
    if(status == 0 )
    {
        appLogWaitMsecs(10);
        status = ub960_cfgScript(ub960AR0820DesCSI2Enable, ub960InstanceId);
    }
    return (status);
}

static int32_t AR0820_StreamOff(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    int8_t ub960InstanceId = getUB960InstIdFromChId(chId);

    if(ub960InstanceId < 0)
    {
        printf("Invalid ub960InstanceId \n");
        return 0xFF;
    }

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    /*Stop Streaming from sensor*/
    status = AR0820_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], 0x301A, 0x0058, 1u);
    if(status == 0)
    {
        appLogWaitMsecs(10);
        status = ub960_cfgScript(ub960AR0820DesCSI2Disable, ub960InstanceId);
    }
    return (status);
}

static int32_t AR0820_PowerOn(uint32_t chId, void *pSensorHdl)
{
    return 0;
}

static int32_t AR0820_PowerOff(uint32_t chId, void *pSensorHdl)
{
    return (0);
}

static uint16_t exp_time_to_rows(uint32_t integration_time)
{
    uint16_t num_rows = (((integration_time - FIT)*PCLK) - (MIT*LLP))/(LLP*NUM_EXP*NUM_ADC);
    if (num_rows > FRAME_LEN_LINES)
    {
      num_rows = FRAME_LEN_LINES;
    }

    return num_rows;
}

static uint16_t expTimePrev[ISS_SENSORS_MAX_CHANNEL];
static uint16_t analogGainPrev[ISS_SENSORS_MAX_CHANNEL];
static int32_t AR0820_SetAeParams(void *pSensorHdl, uint32_t chId, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;
    uint16_t regAddr;
    uint16_t regValue;
    uint32_t analogGain = 0;
    uint8_t coarseGain = 0;
    uint8_t tmp8 = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    if (expTimePrev[chId] != pExpPrms->exposureTime[ISS_SENSOR_EXPOSURE_LONG]) 
    {
        expTimePrev[chId] = pExpPrms->exposureTime[ISS_SENSOR_EXPOSURE_LONG];
        regAddr = AR0820_GROUP_HOLD_REG_ADDR;
        regValue = 0x1;
        status = AR0820_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, regValue, 1u);
        if(status < 0)
        {
            printf("AR0820_SetAeParams : Failed writing 0x%x to register 0x%x\n", regValue, regAddr);
        }

        if(0 == ar0820_op_mode)
        {
            /*Linear Mode*/
            regAddr = SENSOR_AR0820_EXP_T1;
            regValue = exp_time_to_rows(pExpPrms->exposureTime[ISS_SENSOR_EXPOSURE_LONG]);
            status = AR0820_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, regValue, 1u);
            if(status < 0)
            {
                printf("AR0820_SetAeParams : Failed writing 0x%x to register 0x%x\n", regValue, regAddr);
            }
        }
        else
        {
            /*WDR mode*/
            uint32_t t1_exp_time = pExpPrms->exposureTime[ISS_SENSOR_EXPOSURE_LONG];
            uint32_t t2_exp_time = t1_exp_time/T1_T2_EXP_RATIO;
            uint32_t t3_exp_time = t1_exp_time/T1_T3_EXP_RATIO;

            regAddr = SENSOR_AR0820_EXP_T1;
            regValue = exp_time_to_rows(t1_exp_time);
            status = AR0820_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, regValue, 1u);
            if(status < 0)
            {
                printf("AR0820_SetAeParams : Failed writing 0x%x to register 0x%x\n", regValue, regAddr);
            }

            regAddr = SENSOR_AR0820_EXP_T2;
            regValue = exp_time_to_rows(t2_exp_time);
            status = AR0820_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, regValue, 1u);
            if(status < 0)
            {
                printf("AR0820_SetAeParams : Failed writing 0x%x to register 0x%x\n", regValue, regAddr);
            }

            regAddr = SENSOR_AR0820_EXP_T3;
            regValue = exp_time_to_rows(t3_exp_time);
            status = AR0820_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, regValue, 1u);
            if(status < 0)
            {
               printf("AR0820_SetAeParams : Failed writing 0x%x to register 0x%x\n", regValue, regAddr);
            }
        }
    }

    if (analogGainPrev[chId] != pExpPrms->analogGain[ISS_SENSOR_EXPOSURE_LONG]) 
    {
        analogGainPrev[chId] = pExpPrms->analogGain[ISS_SENSOR_EXPOSURE_LONG];
        analogGain = pExpPrms->analogGain[ISS_SENSOR_EXPOSURE_LONG];

        /*Calculate coarse gain*/
        if(analogGain < 2048U)
        {
            coarseGain = 1U;
            tmp8 = 0;
        }else if(analogGain < 4096U)
        {
            coarseGain = 2U;
            tmp8 = 1;
        }else if(analogGain < 8192U)
        {
            coarseGain = 4U;
            tmp8 = 2U;
        }else
        {
            coarseGain = 8U;
            tmp8 = 3U;
        }

        regAddr = SENSOR_AR0820_COARSE_ANALOG_GAIN;
        regValue = (tmp8<<12U) | (tmp8<<8U) | (tmp8<<4U) | (tmp8);
        status = AR0820_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, regValue, 1u);
        if(status < 0)
        {
           printf("AR0820_SetAeParams : Failed writing 0x%x to register 0x%x\n", regValue, regAddr);
        }

        /*Calculate fine gain*/
        regAddr = SENSOR_AR0820_FINE_ANALOG_GAIN;
        tmp8 = (16*((analogGain/coarseGain)-1024))/1024;
        regValue = (tmp8<<12U) | (tmp8<<8U) | (tmp8<<4U) | (tmp8);
        status = AR0820_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, regValue, 1u);
        if(status < 0)
        {
           printf("AR0820_SetAeParams : Failed writing 0x%x to register 0x%x\n", regValue, regAddr);
        }

        regAddr = AR0820_GROUP_HOLD_REG_ADDR;
        regValue = 0x0;
        status = AR0820_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, regValue, 1u);
        if(status < 0)
        {
            printf("AR0820_SetAeParams : Failed writing 0x%x to register 0x%x\n", regValue, regAddr);
        }
    }

    return (status);
}

static int32_t AR0820_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms)
{
    int32_t status = 0;
    return (status);
}

static int32_t AR0820_GetExpParams(uint32_t chId, void *pSensorHdl, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;

    assert(NULL != pExpPrms);
    pExpPrms->expRatio = 256;

    return (status);
}

static void AR0820_InitAewbConfig(uint32_t chId, void *pSensorHdl)
{
    return;
}

static void AR0820_GetIspConfig (uint32_t chId, void *pSensorHdl)
{
    return;
}

static void AR0820_deinit (uint32_t chId, void *pSensorHdl)
{
    return;
}

static int32_t AR0820_ReadWriteReg (uint32_t chId, void *pSensorHdl, uint32_t readWriteFlag, I2cParams *pReg)
{
    int32_t status = 0;

    uint16_t regValue = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;

    assert(NULL != pReg);

    if (1u == readWriteFlag)
    {
        /*write*/
        regValue = pReg->nRegValue;
        status = AR0820_WriteReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, regValue, 1u);
    }
    else
    {
        /*read*/
        status = AR0820_ReadReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, &regValue, 1u);

        if (0 == status)
        {
            pReg->nRegValue = regValue;
        }
    }
    return (status);
}

static int32_t AR0820_ReadReg(uint8_t     i2cInstId,
                            uint8_t         i2cAddr,
                            uint16_t        regAddr,
                            uint16_t         *regVal,
                            uint32_t        numRegs)
{
    int32_t  status = -1;
    I2C_Handle sensorI2cHandle = NULL;
    static uint8_t sensorI2cByteOrder = BOARD_I2C_REG_ADDR_MSB_FIRST;
    uint8_t   readReg8_High = 0xAB;
    uint8_t   readReg8_Low = 0xCD;
    uint32_t count;

    getIssSensorI2cInfo(&sensorI2cByteOrder, &sensorI2cHandle);
    if(NULL == sensorI2cHandle)
    {
        printf("Sensor I2C Handle is NULL \n");
        return -1;
    }
    for(count = 0;count<numRegs;count++)
    {
        /*Read a 16-bit value as two 8-byte values*/
        status = Board_i2c16BitRegRd(sensorI2cHandle, i2cAddr, regAddr, &readReg8_High, 1U, sensorI2cByteOrder, SENSOR_I2C_TIMEOUT);
        if(status == 0)
        {
            status = Board_i2c16BitRegRd(sensorI2cHandle, i2cAddr, regAddr+1, &readReg8_Low, 1U, sensorI2cByteOrder, SENSOR_I2C_TIMEOUT);
        }
    }
    *regVal = ((readReg8_High << 8) & 0xFF00) | readReg8_Low;

    return (status);
}

static int32_t AR0820_WriteReg(uint8_t    i2cInstId,
                             uint8_t      i2cAddr,
                             uint16_t     regAddr,
                             uint16_t     regVal,
                             uint32_t     numRegs)
{
    int32_t  status = 0;
    int16_t  ret = 0;
    I2C_Handle sensorI2cHandle = NULL;
    static uint8_t sensorI2cByteOrder = 255u;
    getIssSensorI2cInfo(&sensorI2cByteOrder, &sensorI2cHandle);
    uint8_t   rawRegVal[4];
    I2C_Transaction transaction;

    I2C_transactionInit(&transaction);

    if(NULL == sensorI2cHandle)
    {
        printf("Sensor I2C Handle is NULL \n");
        return -1;
    }

    transaction.slaveAddress = i2cAddr;
    transaction.writeBuf     = rawRegVal;
    transaction.writeCount   = 4;
    transaction.readBuf      = NULL;
    transaction.readCount    = 0;

    /*Assume numRegs = 1 */
    {
          /* Convert Registers address and value into 8bit array */
          rawRegVal[0U] = (uint8_t) ((regAddr >> 8U) & (uint8_t) 0xFF);
          rawRegVal[1U] = (uint8_t) ((regAddr >> 0U) & (uint8_t) 0xFF);
          rawRegVal[2U] = (uint8_t) ((regVal >> 8U) & (uint8_t) 0xFF);
          rawRegVal[3U] = (uint8_t) ((regVal >> 0U) & (uint8_t) 0xFF);
          ret = I2C_transfer(sensorI2cHandle, &transaction); 
          if(ret != 1u)
          {
                appLogPrintf("Error writing to register 0x%x \n ", regAddr);
                status = -1;
          }
    }

    return (status);
}

static int32_t AR0820_GetExpPrgFxn(uint32_t chId, void *pSensorHdl, IssAeDynamicParams *p_ae_dynPrms)
{
    int32_t  status = -1;
    uint8_t count = 0;
    /*Max 40ms Exposure time since sensor is configured at 25 fps*/
    p_ae_dynPrms->targetBrightnessRange.min = 30;
    p_ae_dynPrms->targetBrightnessRange.max = 45;
    p_ae_dynPrms->targetBrightness = 35;
    p_ae_dynPrms->threshold = 1;
    p_ae_dynPrms->exposureTimeStepSize = 1;
    p_ae_dynPrms->enableBlc = 0;

    p_ae_dynPrms->exposureTimeRange[count].min = 100;
    p_ae_dynPrms->exposureTimeRange[count].max = 40000;
    p_ae_dynPrms->analogGainRange[count].min = 1024;
    p_ae_dynPrms->analogGainRange[count].max = 1024;
    p_ae_dynPrms->digitalGainRange[count].min = 256;
    p_ae_dynPrms->digitalGainRange[count].max = 256;
    count++;

    p_ae_dynPrms->exposureTimeRange[count].min = 40000;
    p_ae_dynPrms->exposureTimeRange[count].max = 40000;
    p_ae_dynPrms->analogGainRange[count].min = 1024;
    p_ae_dynPrms->analogGainRange[count].max = 15872;
    p_ae_dynPrms->digitalGainRange[count].min = 256;
    p_ae_dynPrms->digitalGainRange[count].max = 256;
    count++;

    p_ae_dynPrms->numAeDynParams = count;
    return (status);
}

#ifdef _AR0820_DEBUG_
typedef struct 
{
    uint32_t regAddr;
    char regDesc[32];
}sensorDebug;

int32_t AR0820_Debug(uint32_t chId, void *pSensorHdl)
{
    int32_t status = -1;
    uint32_t i2cInstId;
    uint8_t sensorI2cAddr;
    uint16_t regAddr;
    uint16_t regValueRead = 0xABCD;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    uint16_t count=0;
    uint16_t  numDbgReg = 0;

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    i2cInstId = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];

    const sensorDebug ar0820RegReadList[] = {
            {0x2000, "FRAME_COUNT2"},
            {0x2002, "FRAME_COUNT"},
            {0x2008, "FRAME_STATUS"},
        };
    numDbgReg = sizeof(ar0820RegReadList)/sizeof(ar0820RegReadList[0]);

    for(count = 0; count < numDbgReg; count++ )
    {
        regAddr = ar0820RegReadList[count].regAddr;
        AR0820_ReadReg(i2cInstId, sensorI2cAddr, regAddr, &regValueRead, 1u);
        printf("AR0820 Debug Reg %d : %s, Addr 0x%x, Value 0x%x \n", count, ar0820RegReadList[count].regDesc, regAddr, regValueRead);
    }

    return (status);
}
#endif //_AR0820_DEBUG_
