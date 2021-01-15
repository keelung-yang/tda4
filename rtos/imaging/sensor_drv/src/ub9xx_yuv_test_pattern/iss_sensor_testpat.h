#include <iss_sensors.h>
#include <iss_sensor_priv.h>
#include <iss_sensor_if.h>
#include <iss_sensor_serdes.h>

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */

#define ISS_SENSOR_TESTPAT_FEATURES      (ISS_SENSOR_FEATURE_DCC_SUPPORTED)
/*******************************************************************************
 *  Local Functions Declarations
 *******************************************************************************
 */

static int32_t testpat_Probe(uint32_t chId, void *pSensorHdl);
static int32_t testpat_Config(uint32_t chId, void *pSensorHdl, uint32_t sensor_features_requested);
static int32_t testpat_StreamOn(uint32_t chId, void *pSensorHdl);
static int32_t testpat_StreamOff(uint32_t chId, void *pSensorHdl);
static int32_t testpat_PowerOn(uint32_t chId, void *pSensorHdl);
static int32_t testpat_PowerOff(uint32_t chId, void *pSensorHdl);
static int32_t testpat_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms);
static void testpat_InitAewbConfig(uint32_t chId, void *pSensorHdl);
static void testpat_deinit (uint32_t chId, void *pSensorHdl);

