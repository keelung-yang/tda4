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
#ifndef AR0233_GAIN_TABLE_H_
#define AR0233_GAIN_TABLE_H_

#define ISS_SENSORS_AR0233_ANALOG_GAIN_TBL_SIZE           (52U)
static uint32_t ar0233GainsTable[ISS_SENSORS_AR0233_ANALOG_GAIN_TBL_SIZE][3] =
{                                                                              
     {1024, 0x3331, 0xCCC0},
     {1092, 0x3331, 0xCCC2},
     {1170, 0x3331, 0xCCC4},
     {1260, 0x3331, 0xCCC6},
     {1365, 0x3331, 0xCCC8},
     {1489, 0x3331, 0xCCCA},
     {1638, 0x3331, 0xCCCC},
     {1820, 0x3331, 0xCCCE},
     {2048, 0x3332, 0xCCC0},
     {2185, 0x3332, 0xCCC3},
     {2341, 0x3332, 0xCCC5},
     {2521, 0x3332, 0xCCC7},
     {2731, 0x3332, 0xCCC9},
     {2979, 0x3332, 0xCCCB},
     {3277, 0x3332, 0xCCCD},
     {3641, 0x3332, 0xCCCF},
     {4096, 0x3333, 0xCCC0},
     {4369, 0x3333, 0xCCC2},
     {4681, 0x3333, 0xCCC4},
     {5041, 0x3333, 0xCCC6},
     {5461, 0x3333, 0xCCC8},
     {5958, 0x3333, 0xCCCB},
     {6554, 0x3333, 0xCCCD},
     {7282, 0x3333, 0xCCCF},
     {8192, 0x3334, 0xCCC0},
     {8738, 0x3334, 0xCCC2},
     {9362, 0x3334, 0xCCC4},
     {10082, 0x3334, 0xCCC6},
     {10923, 0x3334, 0xCCC9},
     {11916, 0x3334, 0xCCCB},
     {13107, 0x3334, 0xCCCC},
     {14564, 0x3334, 0xCCCF},
     {16384, 0x3335, 0xCCC0},
     {17476, 0x3335, 0xCCC2},
     {18725, 0x3335, 0xCCC5},
     {20165, 0x3335, 0xCCC7},
     {21845, 0x3335, 0xCCC8},
     {23831, 0x3335, 0xCCCA},
     {26214, 0x3335, 0xCCCC},
     {29127, 0x3335, 0xCCCE},
     {32768, 0x3336, 0xCCC0},
     {34953, 0x3336, 0xCCC3},
     {37449, 0x3336, 0xCCC4},
     {40330, 0x3336, 0xCCC7},
     {43691, 0x3336, 0xCCC9},
     {47663, 0x3336, 0xCCCB},
     {52429, 0x3336, 0xCCCD},
     {58254, 0x3336, 0xCCCE},
     {65536, 0x3337, 0xCCC0}
};

#endif /* AR0233_GAIN_TABLE_H_ */
