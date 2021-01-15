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
#include <iss_sensor_if.h>
#include <app_remote_service.h>
#include <app_ipc.h>
#include <itt_srvr_remote.h>

AlgItt_IssAewb2AControlParams aewbCtrlPrms[ISS_SENSORS_MAX_SUPPORTED_SENSOR];
tivx_mutex                 itt_aewb_lock[ISS_SENSORS_MAX_SUPPORTED_SENSOR];
int32_t AewbServer_RemoteServiceHandler(char *service_name, uint32_t cmd,
    void *prm, uint32_t prm_size, uint32_t flags)
{
    int32_t status = 0;
    uint8_t * cmd_param = (uint8_t * )prm;
    uint32_t chId;

    if(NULL == cmd_param)
    {
        printf("AewbServer_RemoteServiceHandler : cmd_param = NULL \n");
        return -1;
    }
    chId = cmd_param[0];

    switch(cmd)
    {
        case AEWB_CMD_GET_2A_PARAMS:
            memcpy(cmd_param, &aewbCtrlPrms[chId], sizeof(AlgItt_IssAewb2AControlParams));
            break;

        case AEWB_CMD_SET_2A_PARAMS:
            cmd_param++;
            status |= tivxMutexLock(itt_aewb_lock[chId]);
            memcpy(&aewbCtrlPrms[chId], cmd_param, sizeof(AlgItt_IssAewb2AControlParams));
            status |= tivxMutexUnlock(itt_aewb_lock[chId]);
            break;

        default:
            printf("AewbServer_RemoteServiceHandler : Invalid command %d\n", cmd);
    }


    return status;
}

int32_t IttRemoteServer_Init()
{
    int32_t status = 0;
    int32_t i = 0;
    status = appRemoteServiceRegister(
        AEWB_SERVER_REMOTE_SERVICE_NAME,
        AewbServer_RemoteServiceHandler
    );

    if(status!=0)
    {
        printf(" AEWB_SERVER_REMOTE_SERVICE_NAME: ERROR: Unable to register remote service AEWB handler\n");
    }

    for(i=0;i<ISS_SENSORS_MAX_SUPPORTED_SENSOR;i++)
    {
        status = tivxMutexCreate(&(itt_aewb_lock[i]));
        if(status!=0)
        {
            printf(" AEWB_SERVER_REMOTE_SERVICE_NAME: ERROR: Unable to create mutex\n");
        }
    }

    return status;
}

int32_t IttRemoteServer_DeInit()
{
    int32_t status = 0;
    int32_t i = 0;
    status = appRemoteServiceUnRegister(AEWB_SERVER_REMOTE_SERVICE_NAME);

    if(status!=0)
    {
        printf(" AEWB_SERVER_REMOTE_SERVICE_NAME: ERROR: Unable to unregister remote service AEWB handler\n");
    }

    for(i=0;i<ISS_SENSORS_MAX_SUPPORTED_SENSOR;i++)
    {
        status = tivxMutexDelete(&(itt_aewb_lock[i]));
        if(status!=0)
        {
            printf(" AEWB_SERVER_REMOTE_SERVICE_NAME: ERROR: Unable to delete mutex\n");
        }
    }

    return status;
}

