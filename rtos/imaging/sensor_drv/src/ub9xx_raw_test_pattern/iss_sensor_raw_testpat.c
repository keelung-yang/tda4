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
#include "iss_sensor_raw_testpat.h"
#include "raw_testpat_serdes_config.h"

static IssSensor_CreateParams  raw_testpat_CreatePrms = {
    UB9XX_RAW_TESTPAT,     /*sensor name*/
    0x6,                             /*i2cInstId*/
    {SENSOR_0_I2C_ALIAS, 0, 0, 0, 0, 0, 0, 0},   /*i2cAddrSensor*/
    {SER_0_I2C_ALIAS, 0, 0, 0, 0, 0, 0, 0},      /*i2cAddrSer*/
    /*IssSensor_Info*/
    {
        {
            RAWTESTPAT_OUT_WIDTH,               /*width*/
            RAWTESTPAT_OUT_HEIGHT,              /*height*/
            1,                              /*num_exposures*/
            vx_false_e,                     /*line_interleaved*/
            {
                {TIVX_RAW_IMAGE_16_BIT, 11},    /*dataFormat and MSB [0]*/
            },
            0,                              /*meta_height_before*/
            0,                              /*meta_height_after*/
        },
        ISS_SENSOR_RAWTESTPAT_FEATURES,     /*features*/
        ALGORITHMS_ISS_AEWB_MODE_NONE,  /*aewbMode*/
        30,                             /*fps*/
        4,                              /*numDataLanes*/
        {1, 2, 3, 4},                   /*dataLanesMap*/
        {0, 0, 0, 0},                   /*dataLanesPolarity*/
        800,                            /*CSI Clock*/
    },
    1,                                  /*numChan*/
    9702,                                /*dccId*/
};

static IssSensorFxns           raw_testpat_SensorFxns = {
    rawtestpat_Probe,
    rawtestpat_Config,
    rawtestpat_StreamOn,
    rawtestpat_StreamOff,
    rawtestpat_PowerOn,
    rawtestpat_PowerOff,
    rawtestpat_GetExpParams,
    rawtestpat_SetAeParams,
    rawtestpat_GetDccParams,
    rawtestpat_InitAewbConfig,
    rawtestpat_GetIspConfig,
    rawtestpat_ReadWriteReg,
    rawtestpat_GetExpPrgFxn,
    rawtestpat_deinit,
    NULL,
    NULL
};

static IssSensorIntfParams     raw_testpat_SensorIntfPrms = {
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

IssSensorConfig     raw_testpat_SensorConfig = {
    ub9702DesCfg_RAWTESTPAT,     /*desCfgPreScript*/
    NULL,      /*serCfgPreScript*/
    NULL,      /*sensorCfgPreScript*/
    ub9702RAWTESTPATDesCSI2Enable,        /*desCfgPostScript*/
    NULL,                    /*serCfgPostScript*/
    NULL,                    /*sensorCfgPostScript*/
};

IssSensors_Handle raw_testpat_SensorHandle = {
    1,                                 /*isUsed*/
    &raw_testpat_CreatePrms,                /*CreatePrms*/
    &raw_testpat_SensorFxns,                /*SensorFxns*/
    &raw_testpat_SensorIntfPrms,            /*SensorIntfPrms*/
};

/*
 * \brief DCC Parameters of RAWTESTPAT
 */
IssCapture_CmplxIoLaneCfg           raw_testpat_Csi2CmplxIoLaneCfg;

extern IssSensors_Handle * gIssSensorTable[ISS_SENSORS_MAX_SUPPORTED_SENSOR];

int32_t IssSensor_rawtestpat_Init()
{
    int32_t status;
    status = IssSensor_Register(&raw_testpat_SensorHandle);
    if(0 != status)
    {
        printf("IssSensor_rawtestpat_Init failed \n");
    }

    return status;
}

/*******************************************************************************
 *  Local Functions Definition
 *******************************************************************************
 */

static int32_t rawtestpat_Probe(uint32_t chId, void *pSensorHdl)
{
    int32_t status = -1;
    uint8_t  domain;
    uint8_t  ub97x_I2cAddr;
    uint8_t  ub97x_I2cInstId;
    uint8_t  ub97x_devId_regAddr = 0x0;
    uint8_t  ub97x_devId_regVal = 0xAB;

    I2C_Handle sensorI2cHandle = NULL;
    static uint8_t sensorI2cByteOrder = BOARD_I2C_REG_ADDR_MSB_FIRST;

    getIssSensorI2cInfo(&sensorI2cByteOrder, &sensorI2cHandle);
    if(NULL == sensorI2cHandle)
    {
        printf("Sensor I2C Handle is NULL \n");
        return -1;
    }

    /*
        Driver implementation for Fusion2 board with dual UB97x
        Cameras 0-3 are mapped to UB97_0
        Cameras 4-7 are mapped to UB97_1
        Other boards or deserializers may need a change 
    */

    if(chId < 4)
    {
        Board_fpdUb9702GetI2CAddr(BOARD_FPD_9702_CSI2_DES_HUB1,
                              &domain,
                              &ub97x_I2cInstId,
                              &ub97x_I2cAddr);
        status = 0;
    }
    else
    {
        printf("Invalid channel ID %d \n", chId);
        ub97x_I2cAddr = 0xFF;
        status = -1;
    }

    if(0 == status)
    {
        status = Board_i2c8BitRegRd(sensorI2cHandle, ub97x_I2cAddr, ub97x_devId_regAddr, &ub97x_devId_regVal, 1U, SENSOR_I2C_TIMEOUT);
        if(0 != status)
        {
            printf("rawtestpat_Probe : Error reading from register 0x%x on slave device 0x%x \n", ub97x_devId_regAddr, ub97x_I2cAddr);
        }
        else
        {
            printf("rawtestpat_Probe : Read 0x%x from register 0x%x on slave device 0x%x \n", ub97x_devId_regVal, ub97x_devId_regAddr, ub97x_I2cAddr);
        }
    }

    return (status);
}

static int32_t rawtestpat_Config(uint32_t chId, void *pSensorHdl, uint32_t sensor_features_requested)
{
    int32_t status = 0;
    I2cParams *deserCfg = NULL;
    uint8_t  ub97x_I2cAddr;
    uint8_t  ub97xI2CInstanceId;
    uint8_t  domain;
    int8_t ub97xInstanceId = getUB960InstIdFromChId(chId);

    if(ub97xInstanceId < 0)
    {
        printf("Invalid ub97xInstanceId \n");
        return 0xFF;
    }

    if(chId < 4)
    {
        Board_fpdUb9702GetI2CAddr(BOARD_FPD_9702_CSI2_DES_HUB1,
                              &domain,
                              &ub97xI2CInstanceId,
                              &ub97x_I2cAddr);
        status = 0;
    }
    else
    {
        printf("Invalid channel ID %d \n", chId);
        ub97x_I2cAddr = 0xFF;
        status = -1;
    }

    if(0 == status)
    {
        if(sensor_features_requested != (sensor_features_requested & ISS_SENSOR_RAWTESTPAT_FEATURES))
        {
            printf("rawtestpat_Config : Error. feature set 0x%x is not supported \n", sensor_features_requested);
            return -1;
        }
        deserCfg = raw_testpat_SensorConfig.desCfgPreScript;

        if(NULL != deserCfg)
        {
            status = ub960_cfgScript(deserCfg, ub97xInstanceId);
        }
    }

    printf(" rawtestpat_Config : status = 0x%x \n", status);
    return (status);
}

static int32_t rawtestpat_StreamOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    int8_t ub97xInstanceId = getUB960InstIdFromChId(chId);

    if(ub97xInstanceId < 0)
    {
        printf("Invalid ub97xInstanceId \n");
        return 0xFF;
    }

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    /*Start Streaming from sensor*/
    status |= rawtestpat_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], 0x301A, 0x005C, 1u);
    appLogWaitMsecs(10);
    status |= ub960_cfgScript(ub9702RAWTESTPATDesCSI2Enable, ub97xInstanceId);
    return (status);
}

static int32_t rawtestpat_StreamOff(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    int8_t ub97xInstanceId = getUB960InstIdFromChId(chId);

    if(ub97xInstanceId < 0)
    {
        printf("Invalid ub97xInstanceId \n");
        return 0xFF;
    }

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    /*Stop Streaming from sensor*/
    status |= rawtestpat_WriteReg(pCreatePrms->i2cInstId, pCreatePrms->i2cAddrSensor[chId], 0x301A, 0x0058, 1u);
    appLogWaitMsecs(10);
    status |= ub960_cfgScript(ub9702RAWTESTPATDesCSI2Disable, ub97xInstanceId);
    return (status);
}

static int32_t rawtestpat_PowerOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status;
    status = initFusion2_UB97x();
    if(status != 0)
    {
        printf("rawtestpat_PowerOn : initFusion2_UB97x returned 0x%x \n", status);
    }
    else
    {
        status = rawtestpat_Probe(chId, pSensorHdl);
        if(status != 0)
        {
            printf("rawtestpat_PowerOn : probe function returned 0x%x \n", status);
        }
    }
    return status;
}

static int32_t rawtestpat_PowerOff(uint32_t chId, void *pSensorHdl)
{
    int32_t status;
    status = deInitFusion2_UB97x();
    if(status != 0)
    {
        printf("rawtestpat_PowerOn : deInitFusion2_UB97x returned 0x%x \n", status);
    }
    return status;
}

static int32_t rawtestpat_SetAeParams(void *pSensorHdl, uint32_t chId, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;
    return (status);
}

static int32_t rawtestpat_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms)
{
    int32_t status = 0;
    return (status);
}

static int32_t rawtestpat_GetExpParams(uint32_t chId, void *pSensorHdl, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;

    assert(NULL != pExpPrms);
    pExpPrms->expRatio = 256;

    return (status);
}

static void rawtestpat_InitAewbConfig(uint32_t chId, void *pSensorHdl)
{
    return;
}

static void rawtestpat_GetIspConfig (uint32_t chId, void *pSensorHdl)
{
    return;
}

static void rawtestpat_deinit (uint32_t chId, void *pSensorHdl)
{
    return;
}

static int32_t rawtestpat_ReadWriteReg (uint32_t chId, void *pSensorHdl, uint32_t readWriteFlag, I2cParams *pReg)
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
        status = rawtestpat_WriteReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, regValue, 1u);
    }
    else
    {
        /*read*/
        status = rawtestpat_ReadReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, &regValue, 1u);

        if (0 == status)
        {
            pReg->nRegValue = regValue;
        }
    }
    return (status);
}

static int32_t rawtestpat_ReadReg(uint8_t     i2cInstId,
                            uint8_t         i2cAddr,
                            uint16_t        regAddr,
                            uint16_t         *regVal,
                            uint32_t        numRegs)
{
    int32_t  status = 0;
    return (status);
}

static int32_t rawtestpat_WriteReg(uint8_t    i2cInstId,
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

static int32_t rawtestpat_GetExpPrgFxn(uint32_t chId, void *pSensorHdl, IssAeDynamicParams *p_ae_dynPrms)
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
    p_ae_dynPrms->analogGainRange[count].max = 8192;
    p_ae_dynPrms->digitalGainRange[count].min = 256;
    p_ae_dynPrms->digitalGainRange[count].max = 256;
    count++;

    p_ae_dynPrms->numAeDynParams = count;
    return (status);
}

