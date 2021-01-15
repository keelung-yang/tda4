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
#ifndef _IMX390_SERDES_H_
#define _IMX390_SERDES_H_

/**
7-bit Alias addresses for sensor and serializer
Physical addresses must be programmed in UB96x config
SoC will communicate with the devices using alias adresses 
*/

//#define CAM_MODULE_VER 0 //D3_IMX390_CM
#define CAM_MODULE_VER 1 //D3_IMX390_RCM

//#define FUSION_BOARD_VER 0 //RevB and older
#define FUSION_BOARD_VER 1 //RevC

#if (CAM_MODULE_VER == 0)
#define IMX390_I2C_ADDR 0x42
#elif (CAM_MODULE_VER == 1)
#define IMX390_I2C_ADDR 0x34
#else
Unsuppprted version
#endif

#define IMX390_D3_SER_CFG_SIZE    (10U)

I2cParams ub953SerCfg_D3IMX390[IMX390_D3_SER_CFG_SIZE] = {
    {0x01, 0x01, 0x20},
    {0x02, 0x72, 0x10},

#if (FUSION_BOARD_VER == 0)
    {0x06, 0x21, 0x1F},
#elif (FUSION_BOARD_VER == 1)
    {0x06, 0x41, 0x1F},
#else
Unsuppprted version
#endif

#if (CAM_MODULE_VER == 0)
    {0x07, 0x28, 0x1F},
    {0x0D, 0x01, 0x10},
#elif (CAM_MODULE_VER == 1)
    {0x07, 	0x25, 0x1F},
    {0x0D, 0x03, 0x10},
#else
Unsuppprted version
#endif
    {0x0E, 0xF0, 0x10},
    {0xB0, 0x04, 0x10},
    {0xB1, 0x08, 0x10},
    {0xB2, 0x07, 0x10},
    {0xFFFF, 0x00, 0x0} //End of script
};

#define IMX390_D3_DES_CFG_SIZE    (59U)
I2cParams ub960DesCfg_D3IMX390_0[IMX390_D3_DES_CFG_SIZE] = {
    {0x01, 0x02, 0x20},
    {0x1f, 0x00, 0x00},

    {0x0D, 0x90, 0x1}, /*I/O to 3V3 - Options not valid with datashee*/
    {0x0C, 0x0F, 0x1}, /*Enable All ports*/

    /*Select Channel 0*/                                               
    {0x4C, 0x01, 0x10},
    {0x58, 0x5E, 0x1}, /*Enable Back channel, set to 50Mbs*/
    {0x72, 0x00, 0x1}, /*VC map*/

    /*Select Channel 1*/
    {0x4C, 0x12, 0x10},
    {0x58, 0x5E, 0x1},/*Enable Back channel, set to 50Mbs*/

    /*Select Channel 2*/
    {0x4C, 0x24, 0x10},
    {0x58, 0x5E, 0x1},/*Enable Back channel, set to 50Mbs*/
   
    /*Select Channel 3*/
    {0x4C, 0x38, 0x10},
    {0x58, 0x5E, 0x1},/*Enable Back channel, set to 50Mbs*/
  
    {0x20, 0x00, 0x1}, /*Forwarding and using CSIport 0 */

    /*Sets GPIOS*/     
    {0x10, 0x83, 0x1},
    {0x11, 0xA3, 0x1},
    {0x12, 0xC3, 0x1},
    {0x13, 0xE3, 0x1},

    {0x4C, 0x01, 0x10}, /* 0x01 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_0_I2C_ALIAS << 1U), 0x1},
    {0x5E, IMX390_I2C_ADDR, 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_0_I2C_ALIAS << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0x00,0x0}, /*VC Map - All to 0 */

    {0x4C, 0x12, 0x10}, /* 0x12 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_1_I2C_ALIAS << 1U), 0x1},
    {0x5E, IMX390_I2C_ADDR, 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_1_I2C_ALIAS << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0x55,0x0}, /*VC Map - All to 1 */

    {0x4C, 0x24, 0x10}, /* 0x24 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_2_I2C_ALIAS<< 1U), 0x1},
    {0x5E, IMX390_I2C_ADDR, 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_2_I2C_ALIAS << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0xaa,0x0}, /*VC Map - All to 2 */

    {0x4C, 0x38, 0x10}, /* 0x38 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_3_I2C_ALIAS << 1U), 0x1},
    {0x5E, IMX390_I2C_ADDR, 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_3_I2C_ALIAS<< 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0xFF,0x0}, /*VC Map - All to 3 */
    {0xFFFF, 0x00, 0x0} //End of script
};

I2cParams ub960DesCfg_D3IMX390_1[IMX390_D3_DES_CFG_SIZE] = {
    {0x01, 0x02, 0x20},
    {0x1f, 0x00, 0x00},

    {0x0D, 0x90, 0x1}, /*I/O to 3V3 - Options not valid with datashee*/
    {0x0C, 0x0F, 0x1}, /*Enable All ports*/

    /*Select Channel 0*/                                               
    {0x4C, 0x01, 0x10},
    {0x58, 0x5E, 0x1}, /*Enable Back channel, set to 50Mbs*/
    {0x72, 0x00, 0x1}, /*VC map*/

    /*Select Channel 1*/
    {0x4C, 0x12, 0x10},
    {0x58, 0x5E, 0x1},/*Enable Back channel, set to 50Mbs*/

    /*Select Channel 2*/
    {0x4C, 0x24, 0x10},
    {0x58, 0x5E, 0x1},/*Enable Back channel, set to 50Mbs*/
   
    /*Select Channel 3*/
    {0x4C, 0x38, 0x10},
    {0x58, 0x5E, 0x1},/*Enable Back channel, set to 50Mbs*/
  
    {0x20, 0x00, 0x1}, /*Forwarding and using CSIport 0 */

    /*Sets GPIOS*/     
    {0x10, 0x83, 0x1},
    {0x11, 0xA3, 0x1},
    {0x12, 0xC3, 0x1},
    {0x13, 0xE3, 0x1},

    {0x4C, 0x01, 0x10}, /* 0x01 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_4_I2C_ALIAS << 1U), 0x1},
    {0x5E, IMX390_I2C_ADDR, 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_4_I2C_ALIAS << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0x00,0x0}, /*VC Map - All to 0 */

    {0x4C, 0x12, 0x10}, /* 0x12 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_5_I2C_ALIAS << 1U), 0x1},
    {0x5E, IMX390_I2C_ADDR, 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_5_I2C_ALIAS << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0x55,0x0}, /*VC Map - All to 1 */

    {0x4C, 0x24, 0x10}, /* 0x24 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_6_I2C_ALIAS<< 1U), 0x1},
    {0x5E, IMX390_I2C_ADDR, 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_6_I2C_ALIAS << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0xaa,0x0}, /*VC Map - All to 2 */

    {0x4C, 0x38, 0x10}, /* 0x38 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_7_I2C_ALIAS << 1U), 0x1},
    {0x5E, IMX390_I2C_ADDR, 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_7_I2C_ALIAS<< 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0xFF,0x0}, /*VC Map - All to 3 */
    {0xFFFF, 0x00, 0x0} //End of script
};


#endif /* _IMX390_SERDES_H_ */


