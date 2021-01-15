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
#ifndef AR0820_WDR_CONFIG_H_
#define AR0820_WDR_CONFIG_H_

#define X_ADDR_START (0)
#define Y_ADDR_START (0)
#define X_ADDR_END (X_ADDR_START + AR0820_OUT_WIDTH - 1)
#define Y_ADDR_END (Y_ADDR_START + AR0820_OUT_HEIGHT - 1)

/* AR0820 recommended setting */

#define AR0820_WDR_CONFIG_SIZE (302)
static I2cParams ar0820WdrConfig[AR0820_WDR_CONFIG_SIZE] = 
{
     {0x301A, 0x0059, 0x01},  /* RESET_REGISTER */
     {0x301A, 0x0058, 0x0100},  /* RESET_REGISTER */
     {0x301A, 0x0058, 0x01},  /* RESET_REGISTER */
     {0x2512, 0x8000, 0x01},  /* SEQ_CTRL_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFF07, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xFFFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3001, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3010, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3006, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3020, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3008, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xB031, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xA824, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x003C, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x001F, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xB2F9, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x006F, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0078, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x005C, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x106F, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xC013, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x006E, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0079, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x007B, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xC806, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x106E, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0017, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0013, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x004B, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0002, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x90F2, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x90FF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xD034, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1032, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0000, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0033, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x00D1, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x092E, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1333, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x123D, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x045B, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x11BB, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x133A, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1013, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1017, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1015, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1099, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x14DB, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x00DD, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3088, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3084, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x2003, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x11F9, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x02DA, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xD80C, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x2006, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x017A, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x01F0, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x14F0, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x008B, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x10F8, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x118B, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x00ED, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x00E4, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0072, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x203B, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x8828, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x2003, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1064, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0063, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1072, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x003E, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xC00A, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x05CD, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x006E, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x100E, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0019, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0015, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x16EE, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0071, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x10BE, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1063, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1671, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1095, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1019, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3088, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3084, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x2003, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x018B, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x110B, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x117B, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x00E4, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0072, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x20C4, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1064, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x107A, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1072, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3041, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xD800, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x881A, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x100C, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x000E, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x100D, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3081, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x10CB, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1052, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x0038, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xC200, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xCA00, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xD230, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x8200, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x11AE, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xB041, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xD000, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x106D, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x101F, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x100E, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x100A, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3042, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3086, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x102F, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x3090, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x9010, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0xB000, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x30A0, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x1016, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x2510, 0x7FFF, 0x01},  /* SEQ_DATA_PORT */
     {0x3508, 0xAA80, 0x01},  /* RESERVED_MFR_3508 */
     {0x350A, 0xC5C0, 0x01},  /* RESERVED_MFR_350A */
     {0x350C, 0xC8C4, 0x01},  /* RESERVED_MFR_350C */
     {0x350E, 0x8C8C, 0x01},  /* RESERVED_MFR_350E */
     {0x3510, 0x8C88, 0x01},  /* RESERVED_MFR_3510 */
     {0x3512, 0x8C8C, 0x01},  /* RESERVED_MFR_3512 */
     {0x3514, 0xA0A0, 0x01},  /* RESERVED_MFR_3514 */
     {0x3518, 0x0040, 0x01},  /* RESERVED_MFR_3518 */
     {0x351A, 0x8600, 0x01},  /* RESERVED_MFR_351A */
     {0x351E, 0x0E40, 0x01},  /* RESERVED_MFR_351E */
     {0x3506, 0x004A, 0x01},  /* RESERVED_MFR_3506 */
     {0x3520, 0x0E19, 0x01},  /* RESERVED_MFR_3520 */
     {0x3522, 0x7F7F, 0x01},  /* RESERVED_MFR_3522 */
     {0x3524, 0x7F7F, 0x01},  /* RESERVED_MFR_3524 */
     {0x3526, 0x7F7F, 0x01},  /* RESERVED_MFR_3526 */
     {0x3528, 0x7F7F, 0x01},  /* RESERVED_MFR_3528 */
     {0x30FE, 0x00A8, 0x01},  /* RESERVED_MFR_30FE */
     {0x3584, 0x0000, 0x01},  /* RESERVED_MFR_3584 */
     {0x3540, 0x8308, 0x01},  /* RESERVED_MFR_3540 */
     {0x354C, 0x0031, 0x01},  /* RESERVED_MFR_354C */
     {0x354E, 0x535C, 0x01},  /* RESERVED_MFR_354E */
     {0x3550, 0x5C7F, 0x01},  /* RESERVED_MFR_3550 */
     {0x3552, 0x0011, 0x01},  /* RESERVED_MFR_3552 */
     {0x3370, 0x0111, 0x01},  /* DBLC_CONTROL */
     {0x337A, 0x0F50, 0x01},  /* RESERVED_MFR_337A */
     {0x337E, 0xFFF8, 0x01},  /* DBLC_OFFSET0 */
     {0x3110, 0x0011, 0x01},  /* HDR_CONTROL0 */
     {0x3100, 0x4000, 0x01},  /* DLO_CONTROL0 */
     {0x3364, 0x0173, 0x01},  /* DCG_TRIM */
     {0x3180, 0x0021, 0x01},  /* RESERVED_MFR_3180 */
     {0x3E4C, 0x0404, 0x01},  /* RESERVED_MFR_3E4C */
     {0x3E52, 0x0060, 0x01},  /* RESERVED_MFR_3E52 */
     {0x3180, 0x0021, 0x01},  /* RESERVED_MFR_3180 */
     {0x37A0, 0x0001, 0x01},  /* COARSE_INTEGRATION_AD_TIME */
     {0x37A4, 0x0000, 0x01},  /* COARSE_INTEGRATION_AD_TIME2 */
     {0x37A8, 0x0000, 0x01},  /* COARSE_INTEGRATION_AD_TIME3 */
     {0x37AC, 0x0000, 0x01},  /* COARSE_INTEGRATION_AD_TIME4 */
     {0x3372, 0xF50F, 0x01},  /* DBLC_FS0_CONTROL */
     {0x302A, 0x0003, 0x01},  /* VT_PIX_CLK_DIV */
     {0x302C, 0x0701, 0x01},  /* VT_SYS_CLK_DIV */
     {0x302E, 0x0009, 0x01},  /* PRE_PLL_CLK_DIV */
     {0x3030, 0x0087, 0x01},  /* PLL_MULTIPLIER */
     {0x3036, 0x0006, 0x01},  /* OP_WORD_CLK_DIV */
     {0x3038, 0x0001, 0x01},  /* OP_SYS_CLK_DIV */
     {0x303A, 0x0085, 0x01},  /* PLL_MULTIPLIER_ANA */
     {0x303C, 0x0003, 0x01},  /* PRE_PLL_CLK_DIV_ANA */
     {0x31B0, 0x0047, 0x01},  /* FRAME_PREAMBLE */
     {0x31B2, 0x0026, 0x01},  /* LINE_PREAMBLE */
     {0x31B4, 0x5187, 0x01},  /* RESERVED_MFR_31B4 */
     {0x31B6, 0x5248, 0x01},  /* RESERVED_MFR_31B6 */
     {0x31B8, 0x70CA, 0x01},  /* RESERVED_MFR_31B8 */
     {0x31BA, 0x028A, 0x01},  /* RESERVED_MFR_31BA */
     {0x31BC, 0x8A88, 0x01},  /* MIPI_TIMING_4 */
     {0x31BE, 0x0023, 0x01},  /* MIPI_CONFIG_STATUS */
     {0x3004, X_ADDR_START, 0x01},  /* X_ADDR_START_ */                 
     {0x3002, Y_ADDR_START, 0x01},  /* Y_ADDR_START_ */                 
     {0x3008, X_ADDR_END, 0x01},  /* X_ADDR_END_ */                 
     {0x3006, Y_ADDR_END, 0x01},  /* Y_ADDR_END_ */                 
     {0x32FC, 0x0000, 0x01},  /* READ_MODE2 */
     {0x37E0, 0x8421, 0x01},  /* ROW_TX_RO_ENABLE */
     {0x37E2, 0x8421, 0x01},  /* ROW_TX_RO_ENABLE_CB */
     {0x323C, 0x8421, 0x01},  /* ROW_TX_ENABLE */
     {0x323E, 0x8421, 0x01},  /* ROW_TX_ENABLE_CB */
     {0x3040, 0x0001, 0x01},  /* READ_MODE */
     {0x301D, 0x00, 0x01},  /* IMAGE_ORIENTATION_ */
     {0x3082, 0x0008, 0x01},  /* OPERATION_MODE_CTRL */
     {0x30BA, 0x1112, 0x01},  /* DIGITAL_CTRL */
     {0x3012, 0x0065, 0x01},  /* COARSE_INTEGRATION_TIME_ */
     {0x3212, 0x0009, 0x01},  /* COARSE_INTEGRATION_TIME2 */
     {0x3216, 0x0001, 0x01},  /* COARSE_INTEGRATION_TIME3 */
     {0x3238, 0x0222, 0x01},  /* EXPOSURE_RATIO */
     {0x3C06, 0x1C88, 0x01},  /* CONFIGURE_BUFFERS1 */
     {0x3C08, 0x0100, 0x01},  /* CONFIGURE_BUFFERS2 */
     {0x31D0, 0x0001, 0x01},  /* COMPANDING */
     {0x3362, 0x00FF, 0x01},  /* DC_GAIN */
     {0x3366, 0x0000, 0x01},  /* ANALOG_GAIN */
     {0x336A, 0x0000, 0x01},  /* ANALOG_GAIN2 */
     {0x32F6, 0x0001, 0x01},  /* MIDDLE_INTEGRATION_CTRL */
     {0x300C, 0x05C8, 0x01},  /* LINE_LENGTH_PCK_ */
     {0x300A, 0x08F8, 0x01},  /* FRAME_LENGTH_LINES_ */
     {0x33C0, 0x2000, 0x01},  /* OC_LUT_00 */
     {0x33C2, 0x4000, 0x01},  /* OC_LUT_01 */
     {0x33C4, 0x6000, 0x01},  /* OC_LUT_02 */
     {0x33C6, 0x8000, 0x01},  /* OC_LUT_03 */
     {0x33C8, 0xA000, 0x01},  /* OC_LUT_04 */
     {0x33CA, 0xC000, 0x01},  /* OC_LUT_05 */
     {0x33CC, 0xE000, 0x01},  /* OC_LUT_06 */
     {0x33CE, 0xF000, 0x01},  /* OC_LUT_07 */
     {0x33D0, 0xF400, 0x01},  /* OC_LUT_08 */
     {0x33D2, 0xF800, 0x01},  /* OC_LUT_09 */
     {0x33D4, 0xFC00, 0x01},  /* OC_LUT_10 */
     {0x33D6, 0xFFC0, 0x01},  /* OC_LUT_11 */
     {0x33D8, 0xFFC0, 0x01},  /* OC_LUT_12 */
     {0x33DA, 0xFFC0, 0x01},  /* OC_LUT_13 */
     {0x33DC, 0xFFC0, 0x01},  /* OC_LUT_14 */
     {0x33DE, 0xFFC0, 0x01},  /* OC_LUT_15 */
     {0x37A0, 0x0001, 0x01},  /* COARSE_INTEGRATION_AD_TIME */
     {0x37A4, 0x0001, 0x01},  /* COARSE_INTEGRATION_AD_TIME2 */
     {0x37A8, 0x0001, 0x01},  /* COARSE_INTEGRATION_AD_TIME3 */
     {0x37AC, 0x0000, 0x01},  /* COARSE_INTEGRATION_AD_TIME4 */
     {0x3280, 0x0FA0, 0x01},  /* T1_BARRIER_C0 */
     {0x3282, 0x0FA0, 0x01},  /* T1_BARRIER_C1 */
     {0x3284, 0x0FA0, 0x01},  /* T1_BARRIER_C2 */
     {0x3286, 0x0FA0, 0x01},  /* T1_BARRIER_C3 */
     {0x3288, 0x0FA0, 0x01},  /* T2_BARRIER_C0 */
     {0x328A, 0x0FA0, 0x01},  /* T2_BARRIER_C1 */
     {0x328C, 0x0FA0, 0x01},  /* T2_BARRIER_C2 */
     {0x328E, 0x0FA0, 0x01},  /* T2_BARRIER_C3 */
     {0x3290, 0x0FA0, 0x01},  /* T3_BARRIER_C0 */
     {0x3292, 0x0FA0, 0x01},  /* T3_BARRIER_C1 */
     {0x3294, 0x0FA0, 0x01},  /* T3_BARRIER_C2 */
     {0x3296, 0x0FA0, 0x01},  /* T3_BARRIER_C3 */
     {0x3298, 0x0FA0, 0x01},  /* T4_BARRIER_C0 */
     {0x329A, 0x0FA0, 0x01},  /* T4_BARRIER_C1 */
     {0x329C, 0x0FA0, 0x01},  /* T4_BARRIER_C2 */
     {0x329E, 0x0FA0, 0x01},  /* T4_BARRIER_C3 */
     {0x3100, 0x4000, 0x01},  /* DLO_CONTROL0 */
     {0x3102, 0x6064, 0x01},  /* DLO_CONTROL1 */
     {0x3104, 0x6064, 0x01},  /* DLO_CONTROL2 */
     {0x3106, 0x6064, 0x01},  /* DLO_CONTROL3 */
     {0x3108, 0x07D0, 0x01},  /* DLO_CONTROL4 */
     {0x31AE, 0x0204, 0x01},  /* SERIAL_FORMAT */
     {0x31AC, 0x140C, 0x01},  /* DATA_FORMAT_BITS */
     {0x301A, 0x0058, 0x0100},  /* RESET_REGISTER */
     {0x3064, 0x0000, 0x01},  /* SMIA_TEST */
     {0x301A, 0x0058, 0x0100},  /* RESET_REGISTER */
     {0x3064, 0x0000, 0x01},  /* SMIA_TEST */
     {0x305A, 0x0127, 0x01},  /* RED_GAIN */
     {0x35A4, 0x0127, 0x01},  /* RED_GAIN_T2 */
     {0x35AC, 0x0127, 0x01},  /* RED_GAIN_T3 */
     {0x35B4, 0x0127, 0x01},  /* RED_GAIN_T4 */
     {0x3058, 0x01EC, 0x01},  /* BLUE_GAIN */
     {0x35A2, 0x01EC, 0x01},  /* BLUE_GAIN_T2 */
     {0x35AA, 0x01EC, 0x01},  /* BLUE_GAIN_T3 */
     {0x35B2, 0x01EC, 0x01},  /* BLUE_GAIN_T4 */
};
#endif /* AR0820_WDR_CONFIG_H_ */
