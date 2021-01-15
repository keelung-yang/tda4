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
#ifndef _RAWTESTPAT_SERDES_H_
#define _RAWTESTPAT_SERDES_H_

/**
7-bit Alias addresses for sensor and serializer
Physical addresses must be programmed in UB9xx config
SoC will communicate with the devices using alias adresses 
*/
#define RAWTESTPAT_OUT_WIDTH           (3840)
#define RAWTESTPAT_OUT_HEIGHT          (2160)
#define RAWTESTPAT_I2C_ADDR_8BIT    (0x60U)
#define UB971_I2C_ADDR_8BIT     (0x30U)

#define RAWTESTPAT_DES_CFG_SIZE    (39U)

/*RAW12*/
#define RAWTESTPAT_OUT_BYTES_PER_LINE ((RAWTESTPAT_OUT_WIDTH * 3 )>>1)

#define RAWTESTPAT_BPLN_HIGH ((RAWTESTPAT_OUT_BYTES_PER_LINE & 0xFF00)>>8)
#define RAWTESTPAT_BPLN_LOW  (RAWTESTPAT_OUT_BYTES_PER_LINE & 0x00FF)

#define RAWTESTPAT_OUT_HEIGHT_HIGH ((RAWTESTPAT_OUT_HEIGHT & 0xFF00)>>8)
#define RAWTESTPAT_OUT_HEIGHT_LOW  (RAWTESTPAT_OUT_HEIGHT & 0x00FF)

/*Same configuration works for UB960 (Fusion1) and UB9702 (Fusion2)*/
I2cParams ub9702DesCfg_RAWTESTPAT[RAWTESTPAT_DES_CFG_SIZE] = {
    {0x32, 0x01, 0x50},
    {0x1F, 0x10, 0x1},
    {0xC9, 0x32, 0x1},
    {0xB0, 0x1C, 0x1},
    {0xB1, 0x92, 0x1},
    {0xB2, 0x40, 0x1},
    {0xB0, 0x01, 0x1},
    {0xB1, 0x01, 0x1},
    {0xB2, 0x01, 0x1},
    {0xB1, 0x02, 0x1},
    {0xB2, 0xF3, 0x1},
    {0xB1, 0x03, 0x1},
    {0xB2, 0x2C, 0x1},
    {0xB1, 0x04, 0x1},
    {0xB2, RAWTESTPAT_BPLN_HIGH, 0x1},
    {0xB1, 0x05, 0x1},
    {0xB2, RAWTESTPAT_BPLN_LOW, 0x1},
    {0xB1, 0x06, 0x1},
    {0xB2, 0x02, 0x1},
    {0xB1, 0x07, 0x1},
    {0xB2, 0x80, 0x1},/*D0*/
    {0xB1, 0x08, 0x1},
    {0xB2, RAWTESTPAT_OUT_HEIGHT_HIGH, 0x1},
    {0xB1, 0x09, 0x1},
    {0xB2, RAWTESTPAT_OUT_HEIGHT_LOW, 0x1},
    {0xB1, 0x0A, 0x1},
    {0xB2, 0x08, 0x1},
    {0xB1, 0x0B, 0x1},
    {0xB2, 0x80, 0x1},
    {0xB1, 0x0C, 0x1},
    {0xB2, 0x04, 0x1},
    {0xB1, 0x0D, 0x1},
    {0xB2, 0x7D, 0x1},
    {0xB1, 0x0E, 0x1},
    {0xB2, 0x07, 0x1},
    {0xB1, 0x0F, 0x1},
    {0xB2, 0x08, 0x1},
    {0x33, 0x02, 0x1},
    {0xFFFF, 0x00, 0x0} /*End of script */
};

I2cParams ub9702RAWTESTPATDesCSI2Enable[10u] = {
    {0x33, 0x03, 0x1},
    {0xFFFF, 0x00, 0x0} //End of script
};

I2cParams ub9702RAWTESTPATDesCSI2Disable[2u] = {
    {0x33, 0x02, 0x10},
    {0xFFFF, 0x00, 0x0} /*End of script */
};

#endif /* _RAWTESTPAT_SERDES_H_ */


