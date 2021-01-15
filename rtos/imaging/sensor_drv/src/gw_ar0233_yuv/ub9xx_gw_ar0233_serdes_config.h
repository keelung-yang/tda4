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
#ifndef _UB96XGW_AR0233_SERDES_H_
#define _UB96XGW_AR0233_SERDES_H_

#define GW_AR0233_OUT_WIDTH           (1820U)
#define GW_AR0233_OUT_HEIGHT          (940U)

#define GW_AR0233_DES_CFG_SIZE    (10U)
I2cParams ub9xxDesCfg_GW_AR0233[GW_AR0233_DES_CFG_SIZE] = {
    {0xB3, 0x00, 0x1},  // BIST Control Register
    {0x4C, 0x01, 0x1},  // FPD3_PORT_SEL Register
    {0x58, 0x5E, 0x1},  // BCC_CONFIG Register
    {0x32, 0x01, 0x1},  // CSI_PORT_SEL Register
    {0x33, 0x02, 0x1},  // CSI_CTL Register
    {0x20, 0x20, 0x1},  // FWD_CTL1 Register
    {0x0F, 0x7F, 0x1},  // GPIO_INPUT_CTL Register
    {0x6E, 0x10, 0x1},  // BC_GPIO_CTL0 Register
    {0x6F, 0x32, 0x1},  // BC_GPIO_CTL1
    {0xFFFF, 0x00, 0x0} /*End of script */
};

#define GW_AR0233_SER_CFG_SIZE    (5U)
I2cParams ub9xxSerCfg_GW_AR0233[GW_AR0233_SER_CFG_SIZE] = {
    {0x0E, 0xF0, 0xF0}, // Vertical Back Porch
    {0x0D, 0xF0, 0x60}, // Line Period
    {0x0D, 0xB0, 0x60},
    {0x0D, 0xB4, 0x60},
    {0xFFFF, 0x00, 0x0} /*End of script */
};

#define GW_AR0233_SENSOR_CFG_SIZE    (5U)
I2cParams ub9xxSensorCfg_GW_AR0233[GW_AR0233_SENSOR_CFG_SIZE] = {
    {0x301A, 0x0018, 500},  // RESET_REGISTER
    {0x3070, 0x0000, 1},    //  1: Solid color test pattern,
                            //  2: Full color bar test pattern,
                            //  3: Fade to grey color bar test pattern,
                            //  256: Walking 1 test pattern (12 bit)
    {0x3072, 0x0123, 1},    // R
    {0x3074, 0x0456, 1},    // G(GR row)
    {0x3076, 0x0abc, 1},    // B
    {0x3078, 0x0def, 1},    // G(GB row)

    {0xFFFF, 0x00, 0x0} /*End of script */
};

I2cParams ub9xxGW_AR0233DesCSI2Enable[10u] = {
    {0x33, 0x03, 0x1},
    {0xFFFF, 0x00, 0x0} //End of script
};

I2cParams ub9xxGW_AR0233DesCSI2Disable[2u] = {
    {0x33, 0x02, 0x10},
    {0xFFFF, 0x00, 0x0} /*End of script */
};

#endif /* _UB96XGW_AR0233_SERDES_H_ */

