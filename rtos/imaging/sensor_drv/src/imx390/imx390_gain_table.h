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
#ifndef IMX390_GAIN_TABLE_H_
#define IMX390_GAIN_TABLE_H_

#define ISS_IMX390_GAIN_TBL_SIZE                (71U)
/*
 * \brief Gain Table for IMX390
 */
static const uint16_t gIMX390GainsTable[ISS_IMX390_GAIN_TBL_SIZE][2U] =
{
    {1024, 0x20},
    {1121, 0x23},
    {1223, 0x26},
    {1325, 0x28},
    {1427, 0x2A},
    {1529, 0x2C},
    {1631, 0x2E},
    {1733, 0x30},
    {1835, 0x32},
    {1937, 0x33},
    {2039, 0x35},
    {2141, 0x36},
    {2243, 0x37},
    {2345, 0x39},
    {2447, 0x3A},
    {2549, 0x3B},
    {2651, 0x3C},
    {2753, 0x3D},
    {2855, 0x3E},
    {2957, 0x3F},
    {3059, 0x40},
    {3160, 0x41},
    {3262, 0x42},
    {3364, 0x43},
    {3466, 0x44},
    {3568, 0x45},
    {3670, 0x46},
    {3772, 0x46},
    {3874, 0x47},
    {3976, 0x48},
    {4078, 0x49},
    {4180, 0x49},
    {4282, 0x4A},
    {4384, 0x4B},
    {4486, 0x4B},
    {4588, 0x4C},
    {4690, 0x4D},
    {4792, 0x4D},
    {4894, 0x4E},
    {4996, 0x4F},
    {5098, 0x4F},
    {5200, 0x50},
    {5301, 0x50},
    {5403, 0x51},
    {5505, 0x51},
    {5607, 0x52},
    {5709, 0x52},
    {5811, 0x53},
    {5913, 0x53},
    {6015, 0x54},
    {6117, 0x54},
    {6219, 0x55},
    {6321, 0x55},
    {6423, 0x56},
    {6525, 0x56},
    {6627, 0x57},
    {6729, 0x57},
    {6831, 0x58},
    {6933, 0x58},
    {7035, 0x58},
    {7137, 0x59},
    {7239, 0x59},
    {7341, 0x5A},
    {7442, 0x5A},
    {7544, 0x5A},
    {7646, 0x5B},
    {7748, 0x5B},
    {7850, 0x5C},
    {7952, 0x5C},
    {8054, 0x5C},
    {8192, 0x5D}
};

#endif /* IMX390_GAIN_TABLE_H_ */
