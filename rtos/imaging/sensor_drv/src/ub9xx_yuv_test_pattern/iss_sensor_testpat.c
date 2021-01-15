#include "iss_sensor_testpat.h"
#include "ub9xx_testpat_serdes_config.h"

static IssSensor_CreateParams  testpatCreatePrms = {
    UB96X_TESTPATTERN_UYVY,     /*sensor name*/
    0x6,                             /*i2cInstId*/
    {0, 0, 0, 0, 0, 0, 0, 0},   /*i2cAddrSensor*/
    {0, 0, 0, 0, 0, 0, 0, 0},      /*i2cAddrSer*/
    /*IssSensor_Info*/
    {
        {
            TESTPAT_OUT_WIDTH,               /*width*/
            TESTPAT_OUT_HEIGHT,              /*height*/
            1,                              /*num_exposures*/
            vx_false_e,                     /*line_interleaved*/
            {
                {VX_DF_IMAGE_UYVY, 7},    /*dataFormat and MSB [0]*/
            },
            0,                              /*meta_height_before*/
            0,                              /*meta_height_after*/
        },
        ISS_SENSOR_TESTPAT_FEATURES,     /*features*/
        ALGORITHMS_ISS_AEWB_MODE_NONE,  /*aewbMode*/
        30,                             /*fps*/
        4,                              /*numDataLanes*/
        {1, 2, 3, 4},                   /*dataLanesMap*/
        {0, 0, 0, 0},                   /*dataLanesPolarity*/
        800,                            /*CSI Clock*/
    },
    1,                                  /*numChan*/
    960,                                /*dccId*/
};

static IssSensorFxns           testpatSensorFxns = {
    testpat_Probe,
    testpat_Config,
    testpat_StreamOn,
    testpat_StreamOff,
    testpat_PowerOn,
    testpat_PowerOff,
    NULL,
    NULL,
    testpat_GetDccParams,
    testpat_InitAewbConfig,
    NULL,
    NULL,
    NULL,
    testpat_deinit,
    NULL,
    NULL
};

static IssSensorIntfParams     testpatSensorIntfPrms = {
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

IssSensorConfig     testpatSensorRegConfig = {
    ub9xxDesCfg_testpat,     /*desCfgPreScript*/
    NULL,      /*serCfgPreScript*/
    NULL,      /*sensorCfgPreScript*/
    ub9xxtestpatDesCSI2Enable,        /*desCfgPostScript*/
    NULL,                    /*serCfgPostScript*/
    NULL,                    /*sensorCfgPostScript*/
};


IssSensors_Handle testpatSensorHandle = {
    1,                                 /*isUsed*/
    &testpatCreatePrms,                /*CreatePrms*/
    &testpatSensorFxns,                /*SensorFxns*/
    &testpatSensorIntfPrms,            /*SensorIntfPrms*/
};

/*
 * \brief DCC Parameters of testpat
 */
IssCapture_CmplxIoLaneCfg           testpatCsi2CmplxIoLaneCfg;

extern IssSensors_Handle * gIssSensorTable[ISS_SENSORS_MAX_SUPPORTED_SENSOR];

int32_t IssSensor_testpat_Init()
{
    int32_t status;
    status = IssSensor_Register(&testpatSensorHandle);
    if(0 != status)
    {
        printf("IssSensor_testpat_Init failed \n");
    }

    return status;
}

/*******************************************************************************
 *  Local Functions Definition
 *******************************************************************************
 */

static int32_t testpat_Probe(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    return (status);
}

static int32_t testpat_Config(uint32_t chId, void *pSensorHdl, uint32_t sensor_features_requested)
{
    int32_t status = 0;
    I2cParams *deserCfg = NULL;
    int8_t ub9xxInstanceId = getUB960InstIdFromChId(chId);

    deserCfg = testpatSensorRegConfig.desCfgPreScript;

    if(NULL != deserCfg)
    {
        status |= ub960_cfgScript(deserCfg, ub9xxInstanceId);
    }

    return (status);
}

static int32_t testpat_StreamOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    int8_t ub9xxInstanceId = getUB960InstIdFromChId(chId);

    if(ub9xxInstanceId < 0)
    {
        printf("Invalid ub9xxInstanceId \n");
        return 0xFF;
    }

    /*Start Streaming*/
    status |= ub960_cfgScript(ub9xxtestpatDesCSI2Enable, ub9xxInstanceId);
    return (status);
}

static int32_t testpat_StreamOff(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    int8_t ub9xxInstanceId = getUB960InstIdFromChId(chId);

    if(ub9xxInstanceId < 0)
    {
        printf("Invalid ub9xxInstanceId \n");
        return 0xFF;
    }

    /*Stop Streaming*/
    status |= ub960_cfgScript(ub9xxtestpatDesCSI2Disable, ub9xxInstanceId);
    return (status);
}

static int32_t testpat_PowerOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    status = testpat_Probe(chId, pSensorHdl);
    if(status != 0)
    {
        printf("testpat_PowerOn : probe function returned 0x%x \n", status);
    }
    return status;
}

static int32_t testpat_PowerOff(uint32_t chId, void *pSensorHdl)
{
    int32_t status;
    status = deInitFusion2_UB97x();
    if(status != 0)
    {
        printf("testpat_PowerOn : deInitFusion2_UB97x returned 0x%x \n", status);
    }
    return status;
}

static int32_t testpat_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms)
{
    int32_t status = 0;
    return (status);
}

static void testpat_InitAewbConfig(uint32_t chId, void *pSensorHdl)
{
    return;
}


static void testpat_deinit (uint32_t chId, void *pSensorHdl)
{
    return;
}


