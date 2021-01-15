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
#ifndef _AR0820_SERDES_H_
#define _AR0820_SERDES_H_

/**
7-bit Alias addresses for sensor and serializer
Physical addresses must be programmed in UB96x config
SoC will communicate with the devices using alias adresses 
*/

#define AR0820_I2C_ADDR        (0x10U)

#define AR0820_SER_CFG_SIZE    (4U)
I2cParams ub953SerCfg_AR0820[AR0820_SER_CFG_SIZE] = {
    {0x1, 0x2, 1000},
    {0x0E, 0xF0, 0x1},
    {0x0D, 0x0F, 0x1},
    {0xFFFF, 0x00, 0x0} /*End of script */
};

#define AR0820_DES_CFG_SIZE    (16U)
I2cParams ub960DesCfg_AR0820[AR0820_DES_CFG_SIZE] = {
    {0x1, 0x2, 1000},
    {0xB3, 0x0, 0x1},
    {0x4C, 0x01, 0x1},
    {0x58, 0x5e, 0x1},
    {0x5C, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5D, (SENSOR_0_I2C_ALIAS << 1), 0x10},
    {0x65, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5E, (AR0820_I2C_ADDR << 1), 0x10},
    {0x66, (SENSOR_0_I2C_ALIAS << 1), 0x10},
    {0x32, 0x01, 0x1},
    {0x33, 0x02, 0x1},
    {0x20, 0x20, 0x1},
    {0x0E, 0x7F, 0x1},
    {0x6E, 0x10, 0x1},
    {0x6F, 0x32, 0x1},
    {0xFFFF, 0x00, 0x0} /*End of script */
};

I2cParams ub960AR0820DesCSI2Enable[2u] = {
    {0x33, 0x03, 0x10},
    {0xFFFF, 0x00, 0x0} //End of script
};

I2cParams ub960AR0820DesCSI2Disable[2u] = {
    {0x33, 0x02, 0x10},
    {0xFFFF, 0x00, 0x0} /*End of script */
};

#endif /* _AR0820_SERDES_H_ */


