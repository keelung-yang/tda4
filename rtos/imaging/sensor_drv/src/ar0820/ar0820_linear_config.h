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
#ifndef AR0820_LINEAR_CONFIG_H_
#define AR0820_LINEAR_CONFIG_H_

#define X_ADDR_START (0)
#define Y_ADDR_START (0)
#define X_ADDR_END (X_ADDR_START + AR0820_OUT_WIDTH - 1)
#define Y_ADDR_END (Y_ADDR_START + AR0820_OUT_HEIGHT - 1)

#define AR0820_LINEAR_CONFIG_SIZE (217)
static I2cParams ar0820LinearConfig[AR0820_LINEAR_CONFIG_SIZE] = 
{
    {0x301A, 0x0058, 0x01}, /* RESET_REGISTER     */
    {0x2512, 0x8000, 0x01}, /* RESERVED_MFR2_2512 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFF07, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xFFFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3001, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3010, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3006, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3020, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3008, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xB031, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xA824, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x003C, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x001F, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xB0F9, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x006D, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x00EF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x005C, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x106F, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xC013, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x016E, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xC806, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x106E, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x0017, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x0013, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x004B, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x0002, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x90F2, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x90FF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xD034, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1032, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x0000, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x0033, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x00D1, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x092E, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1333, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x123D, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x045B, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x11BB, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x133A, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x907D, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1017, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1115, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x14DB, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x00DD, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3088, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3084, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x2007, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x02DA, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xD80C, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x2009, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x01F0, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x14F0, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x018B, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x128B, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x00E4, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x0072, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x203B, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x8A28, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x10CC, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xC02A, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1064, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x0063, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1072, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x06BE, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x006E, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x100E, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x0019, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x0015, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x16EE, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x0071, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x10BE, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1063, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1671, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1095, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1019, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3088, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3084, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x2003, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x018B, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x128B, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x00E4, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x0072, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x20C4, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x10E4, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1072, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3041, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xD800, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x000A, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x100C, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x008E, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3081, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x10CB, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x10D2, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xC200, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xCA00, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xD230, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x8200, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x11AE, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1039, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xD000, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x106D, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x101F, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x100E, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x100A, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3042, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3086, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x102F, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x3090, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x9010, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0xB000, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x30A0, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x1016, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x7FFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x7FFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x7FFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x7FFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x7FFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x7FFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x2510, 0x7FFF, 0x01}, /* RESERVED_MFR2_2510 */
    {0x350A, 0xC1C1, 0x01}, /* RESERVED_MFR_350A  */
    {0x350C, 0xC1C1, 0x01}, /* RESERVED_MFR_350C  */
    {0x350E, 0x8D8D, 0x01}, /* RESERVED_MFR_350E  */
    {0x3510, 0x8D88, 0x01}, /* RESERVED_MFR_3510  */
    {0x3512, 0x8C8C, 0x01}, /* RESERVED_MFR_3512  */
    {0x3514, 0xA0A0, 0x01}, /* RESERVED_MFR_3514  */
    {0x351A, 0x8600, 0x01}, /* RESERVED_MFR_351A  */
    {0x3506, 0x004A, 0x01}, /* RESERVED_MFR_3506  */
    {0x3520, 0x1422, 0x01}, /* RESERVED_MFR_3520  */
    {0x3522, 0x3E37, 0x01}, /* RESERVED_MFR_3522  */
    {0x3524, 0x6E48, 0x01}, /* RESERVED_MFR_3524  */
    {0x3526, 0x4237, 0x01}, /* RESERVED_MFR_3526  */
    {0x3528, 0x6249, 0x01}, /* RESERVED_MFR_3528  */
    {0x30FE, 0x00A8, 0x01}, /* RESERVED_MFR_30FE  */
    {0x342A, 0x0011, 0x01}, /* RESERVED_MFR_342A  */
    {0x3584, 0x0000, 0x01}, /* ROW_AB_CTRL        */
    {0x354C, 0x0030, 0x01}, /* RESERVED_MFR_354C  */
    {0x3370, 0x0111, 0x01}, /* DBLC_CONTROL       */
    {0x337A, 0x0E74, 0x01}, /* RESERVED_MFR_337A  */
    {0x3110, 0x0011, 0x01}, /* HDR_CONTROL0       */
    {0x3100, 0x4000, 0x01}, /* RESERVED_MFR_3100  */
    {0x33FC, 0x00E4, 0x01}, /* COLOUR_ENABLE      */
    {0x33FE, 0x00E4, 0x01}, /* COLOUR_ENABLE_CB   */
    {0x301E, 0x00A8, 0x01}, /* DATA_PEDESTAL_     */
    {0x3180, 0x0021, 0x01}, /* RESERVED_MFR_3180  */
    {0x3372, 0x710F, 0x01}, /* DBLC_FS0_CONTROL   */
    {0x3E4C, 0x0404, 0x01}, /* RESERVED_MFR_3E4C  */
    {0x3180, 0x0021, 0x01}, /* RESERVED_MFR_3180  */
    {0x37A0, 0x0001, 0x01}, /* COARSE_INTEGRATION_AD_TIME*/
    {0x37A4, 0x0000, 0x01}, /* COARSE_INTEGRATION_AD_TIME2*/
    {0x37A8, 0x0000, 0x01}, /* COARSE_INTEGRATION_AD_TIME3*/
    {0x37AC, 0x0001, 0x01}, /* COARSE_INTEGRATION_AD_TIME4*/
    {0x302A, 0x0003, 0x01}, /* VT_PIX_CLK_DIV     */
    {0x302C, 0x0701, 0x01}, /* VT_SYS_CLK_DIV     */
    {0x302E, 0x0009, 0x01}, /* PRE_PLL_CLK_DIV    */
    {0x3030, 0x0082, 0x01}, /* PLL_MULTIPLIER     */
    /*{0x3030, 0x0062, 0x01}, PLL_MULTIPLIER      */
    {0x3036, 0x0006, 0x01}, /* OP_WORD_CLK_DIV    */
    {0x3038, 0x0001, 0x01}, /* OP_SYS_CLK_DIV     */
    {0x303A, 0x0085, 0x01}, /* PLL_MULTIPLIER_ANA */
    {0x303C, 0x0003, 0x01}, /* PRE_PLL_CLK_DIV_ANA*/
    {0x31B0, 0x0091, 0x01}, /* FRAME_PREAMBLE     */
    {0x31B2, 0x0060, 0x01}, /* LINE_PREAMBLE      */
    {0x31B4, 0x224B, 0x01}, /* RESERVED_MFR_31B4  */
    {0x31B6, 0x2391, 0x01}, /* RESERVED_MFR_31B6  */
    {0x31B8, 0xA04C, 0x01}, /* RESERVED_MFR_31B8  */
    {0x31BA, 0x0410, 0x01}, /* RESERVED_MFR_31BA  */
    {0x31BC, 0x930D, 0x01}, /* RESERVED_MFR_31BC  */
    {0x31BE, 0x5083, 0x01}, /* MIPI_CONFIG_STATUS */
    {0x3004, X_ADDR_START, 0x01},  /* X_ADDR_START_ */                 
    {0x3002, Y_ADDR_START, 0x01},  /* Y_ADDR_START_ */                 
    {0x3008, X_ADDR_END, 0x01},  /* X_ADDR_END_ */                 
    {0x3006, Y_ADDR_END, 0x01},  /* Y_ADDR_END_ */                 
    {0x32FC, 0x0000, 0x01}, /* READ_MODE2         */
    {0x37E0, 0x8421, 0x01}, /* ROW_TX_RO_ENABLE   */
    {0x37E2, 0x8421, 0x01}, /* ROW_TX_RO_ENABLE_CB*/
    {0x323C, 0x8421, 0x01}, /* ROW_TX_ENABLE      */
    {0x323E, 0x8421, 0x01}, /* ROW_TX_ENABLE_CB   */
    {0x3040, 0x0001, 0x01}, /* READ_MODE          */
    {0x301D, 0x0000, 0x01}, /* IMAGE_ORIENTATION_ */
    {0x3082, 0x0000, 0x01}, /* OPERATION_MODE_CTRL*/
    {0x30BA, 0x1100, 0x01}, /* DIGITAL_CTRL       */
    {0x3012, 0x0090, 0x01}, /* COARSE_INTEGRATION_TIME_*/
    {0x3014, 0x0000, 0x01}, /* FINE_INTEGRATION_TIME_*/
    {0x3362, 0x00FF, 0x01}, /* DC_GAIN            */
    {0x3366, 0x0000, 0x01}, /* ANALOG_GAIN        */
    {0x336A, 0x4444, 0x01}, /* ANALOG_GAIN2       */
    {0x300C, 0x1400, 0x01}, /* LINE_LENGTH_PCK_   */
    {0x300A, 0x08B0, 0x01}, /* FRAME_LENGTH_LINES_*/
    {0x31AE, 0x0204, 0x01}, /* SERIAL_FORMAT      */
    {0x31AC, 0x0C0C, 0x01}, /* DATA_FORMAT_BITS   */
    {0x301A, 0x0058, 0x01}, /* RESET_REGISTER     */
};

#endif /* AR0820_LINEAR_CONFIG_H_ */
