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
#include "iss_sensor_ar0233.h"
#include "ar0233_serdes_config.h"

static IssSensor_CreateParams  ar0233CreatePrms = {
    SENSOR_ONSEMI_AR0233_UB953_MARS,     /*sensor name*/
    0x6,                             /*i2cInstId*/
    {SENSOR_0_I2C_ALIAS, 0, 0, 0, 0, 0, 0, 0},   /*i2cAddrSensor*/
    {SER_0_I2C_ALIAS, 0, 0, 0, 0, 0, 0, 0},      /*i2cAddrSer*/
    /*IssSensor_Info*/
    {
        {
            AR0233_OUT_WIDTH,               /*width*/
            AR0233_OUT_HEIGHT,              /*height*/
            1,                              /*num_exposures*/
            vx_false_e,                     /*line_interleaved*/
            {
                {TIVX_RAW_IMAGE_16_BIT, 11},    /*dataFormat and MSB [0]*/
            },
            0,                              /*meta_height_before*/
            0,                              /*meta_height_after*/
        },
        ISS_SENSOR_AR0233_FEATURES,     /*features*/
        ALGORITHMS_ISS_AEWB_MODE_AEWB,  /*aewbMode*/
        30,                             /*fps*/
        4,                              /*numDataLanes*/
        {1, 2, 3, 4},                   /*dataLanesMap*/
        {0, 0, 0, 0},                   /*dataLanesPolarity*/
        800,                            /*CSI Clock*/
    },
    1,                                  /*numChan*/
    233,                                /*dccId*/
};

static IssSensorFxns           ar0233SensorFxns = {
    AR0233_Probe,
    AR0233_Config,
    AR0233_StreamOn,
    AR0233_StreamOff,
    AR0233_PowerOn,
    AR0233_PowerOff,
    AR0233_GetExpParams,
    AR0233_SetAeParams,
    AR0233_GetDccParams,
    AR0233_InitAewbConfig,
    AR0233_GetIspConfig,
    AR0233_ReadWriteReg,
    AR0233_GetExpPrgFxn,
    AR0233_deinit,
    AR0233_GetWBPrgFxn,
    AR0233_SetAwbParams
};

static IssSensorIntfParams     ar0233SensorIntfPrms = {
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

IssSensorConfig     ar0233SensorRegConfigLinear = {
    ub960DesCfg_AR0220,     /*desCfgPreScript*/
    ub953SerCfg_AR0220,     /*serCfgPreScript*/
    ar0220LinearConfig_0,        /*sensorCfgPreScript*/
    ub960AR0233DesCSI2Enable,        /*desCfgPostScript*/
    NULL,                    /*serCfgPostScript*/
    NULL,                    /*sensorCfgPostScript*/
};

IssSensorConfig     ar0233SensorRegConfigWdr = {
    ub960DesCfg_AR0233,     /*desCfgPreScript*/
    ub953SerCfg_AR0233,     /*serCfgPreScript*/
    ar0233WdrConfig,        /*sensorCfgPreScript*/
    ub960AR0233DesCSI2Enable,        /*desCfgPostScript*/
    NULL,                    /*serCfgPostScript*/
    NULL,                    /*sensorCfgPostScript*/
};

IssSensors_Handle ar0233SensorHandle = {
    1,                                 /*isUsed*/
    &ar0233CreatePrms,                /*CreatePrms*/
    &ar0233SensorFxns,                /*SensorFxns*/
    &ar0233SensorIntfPrms,            /*SensorIntfPrms*/
};

/*
 * \brief DCC Parameters of AR0233
 */
IssCapture_CmplxIoLaneCfg           ar0233Csi2CmplxIoLaneCfg;

extern IssSensors_Handle * gIssSensorTable[ISS_SENSORS_MAX_SUPPORTED_SENSOR];


int32_t IssSensor_AR0233_Init()
{
    int32_t status;
    status = IssSensor_Register(&ar0233SensorHandle);
    if(0 != status)
    {
        appLogPrintf("IssSensor_AR0233_Init failed \n");
    }

    return status;

}

/*******************************************************************************
 *  Local Functions Definition
 *******************************************************************************
 */

static int32_t AR0233_Probe(uint32_t chId, void *pSensorHdl)
{
    int32_t status = -1;
    uint32_t i2cInstId;
    uint8_t sensorI2cAddr;
    uint16_t chipIdRegAddr = AR0233_CHIP_ID_REG_ADDR;
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

    while( (chipIdRegValueRead != AR0233_CHIP_ID_REG_VAL) && (count < max_retries))
    {
        status = AR0233_ReadReg(i2cInstId, sensorI2cAddr, chipIdRegAddr, &chipIdRegValueRead, 1U);
        if(status == 0 )
        {
            if(chipIdRegValueRead == AR0233_CHIP_ID_REG_VAL)
            {
                status = 0;
                appLogPrintf("AR0233_Probe SUCCESS : Read expected value 0x%x at chip ID register 0x%x \n", AR0233_CHIP_ID_REG_VAL, chipIdRegAddr);
            }
            else
            {
                status = -1;
                appLogPrintf("AR0233_Probe : 0x%x read at chip ID register 0x%x. Expected 0x%x \n", chipIdRegValueRead, chipIdRegAddr, AR0233_CHIP_ID_REG_VAL);
                appLogPrintf("AR0233 Probe Failed \n");
                appLogWaitMsecs(100);
            }
        }
        else
        {
            appLogPrintf("AR0233 Probe : Failed to read CHIP_ID register 0x%x \n", chipIdRegAddr);
        }
        count++;
    }
    return (status);
}

static int32_t AR0233_Config(uint32_t chId, void *pSensorHdl, uint32_t sensor_features_requested)
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
        appLogPrintf("Invalid ub960InstanceId \n");
        return -1;
    }

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    if(sensor_features_requested != (sensor_features_requested & ISS_SENSOR_AR0233_FEATURES))
    {
        appLogPrintf("AR0233_Config : Error. feature set 0x%x is not supported \n", sensor_features_requested);
        return -1;
    }
    i2cInstId = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];

    if(sensor_features_requested & ISS_SENSOR_FEATURE_COMB_COMP_WDR_MODE)
    {
        appLogPrintf("AR0233_Config --:-- ISS_SENSOR_FEATURE_COMB_COMP_WDR_MODE");
        deserCfg = ar0233SensorRegConfigWdr.desCfgPreScript;
        serCfg = ar0233SensorRegConfigWdr.serCfgPreScript;
        sensorCfg = ar0233SensorRegConfigWdr.sensorCfgPreScript;
        sensor_cfg_script_len = AR0233_WDR_CONFIG_SIZE;
    }else
    {
        appLogPrintf("AR0233_Config --:-- ISS_SENSOR_FEATURE_LINEAR_MODE");
        deserCfg = ar0233SensorRegConfigLinear.desCfgPreScript;
        serCfg = ar0233SensorRegConfigLinear.serCfgPreScript;
        sensorCfg = ar0233SensorRegConfigLinear.sensorCfgPreScript;
        sensor_cfg_script_len = AR0220_LINEAR_CONFIG_SIZE_0;
    }

    if(NULL != deserCfg)
    {
        appLogPrintf("AR0233_Config --:-- Ininialize des");
        status |= ub960_cfgScript(deserCfg, ub960InstanceId);
    }
    if(0 == status)
    {
        appLogWaitMsecs(50);
    
    /*The code assumes that I2C instance is the same for sensor and serializer*/
        if(NULL != serCfg)
        {
            appLogPrintf("AR0233_Config --:-- Ininialize ser");
            status = ub953_cfgScript(i2cInstId, pCreatePrms->i2cAddrSer[chId], serCfg);
        }
        if(0 == status)
        {
            appLogWaitMsecs(50);

#ifndef _TEST_PATTERN_ENABLE_
            if(NULL != sensorCfg)
            {
                appLogPrintf(" Configuring AR0233 imager .. Please wait till it finishes \n");
                for(regCnt=0; regCnt<sensor_cfg_script_len; regCnt++)
                {
                    regAddr  = sensorCfg[regCnt].nRegAddr;
                    regValue = sensorCfg[regCnt].nRegValue;
                    delayMilliSec = sensorCfg[regCnt].nDelay;
        
                    status = AR0233_WriteReg(i2cInstId, sensorI2cAddr, regAddr, regValue, 1u);
                    if (0 != status)
                    {
                        appLogPrintf(" \n \n AR0233: Sensor Reg Write Failed for regAddr 0x%x \n \n", regAddr);
                    }
        
                    if (delayMilliSec > 0)
                    {
                        appLogWaitMsecs(delayMilliSec);
                    }
                }
                /*Wait 10ms after the init is done*/
                appLogWaitMsecs(10);
                appLogPrintf(" AR0233 config done\n");
            }
            else
            {
                appLogPrintf(" AR0233 config script is NULL \n");
            }
#endif

        }

    }

    return (status);
}

static int32_t AR0233_StreamOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    int8_t ub960InstanceId = getUB960InstIdFromChId(chId);

    if(ub960InstanceId < 0)
    {
        appLogPrintf("Invalid ub960InstanceId \n");
        return -1;
    }

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    /*Start Streaming from sensor*/
    status |= AR0233_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], 0x301A, 0x005C, 1u);
    appLogWaitMsecs(10);
    status |= ub960_cfgScript(ub960AR0233DesCSI2Enable, ub960InstanceId);
    return (status);
}

static int32_t AR0233_StreamOff(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    int8_t ub960InstanceId = getUB960InstIdFromChId(chId);

    if(ub960InstanceId < 0)
    {
        appLogPrintf("Invalid ub960InstanceId \n");
        return -1;
    }

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    /*Stop Streaming from sensor*/
    status |= AR0233_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], 0x301A, 0x0058, 1u);
    appLogWaitMsecs(10);
    status |= ub960_cfgScript(ub960AR0233DesCSI2Disable, ub960InstanceId);
    return (status);
}

static int32_t AR0233_PowerOn(uint32_t chId, void *pSensorHdl)
{
    return 0;
}

static int32_t AR0233_PowerOff(uint32_t chId, void *pSensorHdl)
{
    return (0);
}

static uint16_t expRegValueOld[ISS_SENSORS_MAX_CHANNEL];
static uint16_t coarseGainRegValueOld[ISS_SENSORS_MAX_CHANNEL];
static uint16_t fineGainRegValueOld[ISS_SENSORS_MAX_CHANNEL];

static int32_t AR0233_SetAeParams(void *pSensorHdl, uint32_t chId, IssSensor_ExposureParams *pExpPrms)
{
    uint16_t regAddr;
    uint16_t extRegValue;
    uint16_t coarseGainRegValue = 0x3331;
    uint16_t fineGainRegValue = 0xCCC0;
    int32_t status = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    uint32_t lineIntgTimeInNs = AR0233_LINE_LEN_TIME_NS;
    uint32_t maxCoarseIntgRows = SENSOR_AR0233_MAX_EXP_ROWS;
    uint32_t minCoarseIntgRows = SENSOR_AR0233_MIN_EXP_ROWS;
    uint16_t cnt;
    uint16_t analog_coarse_gain_t1 = 0U;
    uint16_t analog_fine_gain_t1 = 0;

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    regAddr = SENSOR_AR0233_EXP_T1;
    extRegValue = (uint16_t) ((pExpPrms->exposureTime[ISS_SENSOR_EXPOSURE_LONG] *1000)/ lineIntgTimeInNs);

    if (extRegValue > maxCoarseIntgRows)
    {
      extRegValue = maxCoarseIntgRows;
    }else if(extRegValue < minCoarseIntgRows)
    {
      extRegValue = minCoarseIntgRows;
    }

    if (extRegValue == expRegValueOld[chId])
    {
        status = 0;
    }
    else
    {
        status |= AR0233_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, extRegValue, 1u);
        expRegValueOld[chId] = extRegValue;
    }

    for (cnt = 0; cnt < ISS_SENSORS_AR0233_ANALOG_GAIN_TBL_SIZE; cnt ++)
    {
        if (pExpPrms->analogGain[ISS_SENSOR_EXPOSURE_LONG] <= ar0233GainsTable[cnt][0])
        {
            /*Apply same analog gain to T1 exposure*/
            analog_coarse_gain_t1 = ar0233GainsTable[cnt][1];
            analog_fine_gain_t1 = ar0233GainsTable[cnt][2];

            break;
        }
    }

    coarseGainRegValue = (uint16_t)( analog_coarse_gain_t1 & SENSOR_AR0233_ANALOG_GAIN_T1T2T3T4_MASK);
    fineGainRegValue = (uint16_t)( analog_fine_gain_t1 & SENSOR_AR0233_ANALOG_GAIN_T1T2T3T4_MASK);

    regAddr = SENSOR_AR0233_ANALOG_COARSE_GAIN;
    if (coarseGainRegValue == coarseGainRegValueOld[chId])
    {
        status = 0;
    }
    else
    {
        status |= AR0233_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, coarseGainRegValue, 1u);
        coarseGainRegValueOld[chId] = coarseGainRegValue;
    }

    regAddr = SENSOR_AR0233_ANALOG_FINE_GAIN;
    if (fineGainRegValue == fineGainRegValueOld[chId])
    {
        status = 0;
    }
    else
    {
        status |= AR0233_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, fineGainRegValue, 1u);
        fineGainRegValueOld[chId] = fineGainRegValue;
    }

    return (status);
}

static uint16_t redGain_prev[ISS_SENSORS_MAX_CHANNEL];
static uint16_t greenGain_prev[ISS_SENSORS_MAX_CHANNEL];
static uint16_t blueGain_prev[ISS_SENSORS_MAX_CHANNEL];

static int32_t AR0233_SetAwbParams(void *pSensorHdl, uint32_t chId, IssSensor_WhiteBalanceParams *pWbPrms)
{
    int32_t status = 0;
    uint16_t regAddr;
    uint16_t regValue;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);
    assert(NULL != pWbPrms);

    if(redGain_prev[chId] != pWbPrms->rGain[0])
    {
        redGain_prev[chId] = pWbPrms->rGain[0];
        regAddr = AR0233_RED_GAIN_REG;
        regValue = (pWbPrms->rGain[0]>>2);/*Sensor gain is Q8, ISP gain is Q10*/
        status |= AR0233_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, regValue, 1u);
    }

    if(greenGain_prev[chId] != pWbPrms->gGain[0])
    {
        greenGain_prev[chId] = pWbPrms->gGain[0];
        regAddr = AR0233_GREEN1_GAIN_REG;
        regValue = (pWbPrms->gGain[0]>>2);/*Sensor gain is Q8, ISP gain is Q10*/
        status |= AR0233_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, regValue, 1u);
        regAddr = AR0233_GREEN2_GAIN_REG;
        status |= AR0233_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, regValue, 1u);
    }

    if(blueGain_prev[chId] != pWbPrms->bGain[0])
    {
        blueGain_prev[chId] = pWbPrms->bGain[0];
        regAddr = AR0233_BLUE_GAIN_REG;
        regValue = (pWbPrms->bGain[0]>>2);/*Sensor gain is Q8, ISP gain is Q10*/
        status |= AR0233_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], regAddr, regValue, 1u);
    }

    return (status);
}

static int32_t AR0233_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms)
{
    int32_t status = 0;
    return (status);
}

static int32_t AR0233_GetExpParams(uint32_t chId, void *pSensorHdl, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;

    assert(NULL != pExpPrms);
    pExpPrms->expRatio = ISS_SENSOR_AR0233_DEFAULT_EXP_RATIO;

    return (status);
}

static void AR0233_InitAewbConfig(uint32_t chId, void *pSensorHdl)
{
    return;
}

static void AR0233_GetIspConfig (uint32_t chId, void *pSensorHdl)
{
    return;
}

static void AR0233_deinit (uint32_t chId, void *pSensorHdl)
{
    return;
}

static int32_t AR0233_ReadWriteReg (uint32_t chId, void *pSensorHdl, uint32_t readWriteFlag, I2cParams *pReg)
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
        status = AR0233_WriteReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, regValue, 1u);
    }
    else
    {
        /*read*/
        status = AR0233_ReadReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, &regValue, 1u);

        if (0 == status)
        {
            pReg->nRegValue = regValue;
        }
    }
    return (status);
}

static int32_t AR0233_ReadReg(uint8_t     i2cInstId,
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
        appLogPrintf("AR0233_ReadReg : Sensor I2C Handle is NULL \n");
        return -1;
    }
    for(count = 0;count<numRegs;count++)
    {
        /*Read a 16-bit value as two 8-byte values*/
        status = Board_i2c16BitRegRd(sensorI2cHandle, i2cAddr, regAddr, &readReg8_High, 1U, sensorI2cByteOrder, SENSOR_I2C_TIMEOUT);
        if(status == 0 )
        {
            status = Board_i2c16BitRegRd(sensorI2cHandle, i2cAddr, regAddr+1, &readReg8_Low, 1U, sensorI2cByteOrder, SENSOR_I2C_TIMEOUT);
        }
    }
    *regVal = ((readReg8_High << 8) & 0xFF00) | readReg8_Low;

    return (status);
}

static int32_t AR0233_WriteReg(uint8_t    i2cInstId,
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
        appLogPrintf("AR0233_WriteReg : Sensor I2C Handle is NULL \n");
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

static int32_t AR0233_GetExpPrgFxn(uint32_t chId, void *pSensorHdl, IssAeDynamicParams *p_ae_dynPrms)
{
    int32_t  status = -1;
    uint8_t count = 0;

    p_ae_dynPrms->targetBrightnessRange.min = 40;
    p_ae_dynPrms->targetBrightnessRange.max = 50;
    p_ae_dynPrms->targetBrightness = 45;
    p_ae_dynPrms->threshold = 5;
    p_ae_dynPrms->exposureTimeStepSize = 1;
    p_ae_dynPrms->enableBlc = 0;

    /*Limit max exposure time to 8.3ms to minimize motion artifacts*/
    p_ae_dynPrms->exposureTimeRange[count].min = 100;
    p_ae_dynPrms->exposureTimeRange[count].max = 8333;
    p_ae_dynPrms->analogGainRange[count].min = 1024;
    p_ae_dynPrms->analogGainRange[count].max = 1024;
    p_ae_dynPrms->digitalGainRange[count].min = 256;
    p_ae_dynPrms->digitalGainRange[count].max = 256;
    count++;

    p_ae_dynPrms->exposureTimeRange[count].min = 8333;
    p_ae_dynPrms->exposureTimeRange[count].max = 8333;
    p_ae_dynPrms->analogGainRange[count].min = 1024;
    p_ae_dynPrms->analogGainRange[count].max = 65536;
    p_ae_dynPrms->digitalGainRange[count].min = 256;
    p_ae_dynPrms->digitalGainRange[count].max = 256;
    count++;
    p_ae_dynPrms->numAeDynParams = count;
    return (status);
}

static int32_t AR0233_GetWBPrgFxn(uint32_t chId, void *pSensorHdl, IssAwbDynamicParams *p_awb_dynPrms)
{
    int32_t  status = 0;

    p_awb_dynPrms->redGainRange.min = 512;
    p_awb_dynPrms->redGainRange.max = 2048;

    p_awb_dynPrms->greenGainRange.min = 512;
    p_awb_dynPrms->greenGainRange.max = 2048;

    p_awb_dynPrms->blueGainRange.min = 512;
    p_awb_dynPrms->blueGainRange.max = 2048;

    p_awb_dynPrms->sensor_pre_gain = 1;

    return (status);
}


#ifdef _AR0233_DEBUG_
typedef struct 
{
    uint32_t regAddr;
    char regDesc[32];
}sensorDebug;

int32_t AR0233_Debug(uint32_t chId, void *pSensorHdl)
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

    const sensorDebug ar0233RegReadList[] = {
            {0x2000, "FRAME_COUNT2"},
            {0x2002, "FRAME_COUNT"},
            {0x2008, "FRAME_STATUS"},
        };
    numDbgReg = sizeof(ar0233RegReadList)/sizeof(ar0233RegReadList[0]);

    for(count = 0; count < numDbgReg; count++ )
    {
        regAddr = ar0233RegReadList[count].regAddr;
        AR0233_ReadReg(i2cInstId, sensorI2cAddr, regAddr, &regValueRead, 1u);
        appLogPrintf("AR0233 Debug Reg %d : %s, Addr 0x%x, Value 0x%x \n", count, ar0233RegReadList[count].regDesc, regAddr, regValueRead);
    }

    return (status);
}
#endif //_AR0233_DEBUG_
