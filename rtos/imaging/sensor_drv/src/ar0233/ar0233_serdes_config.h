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
#ifndef _AR0233_SERDES_H_
#define _AR0233_SERDES_H_

/**
7-bit Alias addresses for sensor and serializer
Physical addresses must be programmed in UB96x config
SoC will communicate with the devices using alias adresses 
*/

//#define _TEST_PATTERN_ENABLE_


#ifdef _TEST_PATTERN_ENABLE_
#define AR0233_SER_CFG_SIZE    (37U)
#else
#define AR0233_SER_CFG_SIZE    (7U)
#endif

#define AR0233_I2C_ADDR        (0x10U)
#define AR0220_I2C_ADDR        (0x10U)
#define SER_0_I2C_ADDR         (0x18U)

I2cParams ub953SerCfg_AR0233[AR0233_SER_CFG_SIZE] = {
    {0x1, 0x2, 1000},
    {0x2, 0x73, 0x1},
    {0x6, 0x41, 0x1},
    {0x7, 0x22, 0x1},
    {0xE, 0x3c, 0x1},
    {0xD, 0x1 , 0x1},

#ifdef _TEST_PATTERN_ENABLE_
#define AR0233_WIDTH_HIBYTE  ((AR0233_OUT_WIDTH >> 8)& 0xFF)
#define AR0233_WIDTH_LOBYTE  (AR0233_OUT_WIDTH & 0xFF)
#define AR0233_HEIGHT_HIBYTE  ((AR0233_OUT_HEIGHT >> 8)& 0xFF)
#define AR0233_HEIGHT_LOBYTE  (AR0233_OUT_HEIGHT & 0xFF)

#define BAR_WIDTH_HIBYTE  (((AR0233_OUT_WIDTH/4) >> 8)& 0xFF)
#define BAR_WIDTH_LOBYTE  ((AR0233_OUT_WIDTH/4) & 0xFF)

    {0xB0, 0x00, 0x10}, // Indirect Pattern Gen Registers
    {0xB1, 0x01, 0x10}, // PGEN_CTL
    {0xB2, 0x01, 0x10}, //
    {0xB1, 0x02, 0x10}, // PGEN_CFG
    {0xB2, 0x33, 0x10}, //
    {0xB1, 0x03, 0x10}, // PGEN_CSI_DI
    {0xB2, 0x2C, 0x10}, //RAW12
    {0xB1, 0x04, 0x10}, // PGEN_LINE_SIZE1
    {0xB2, AR0233_WIDTH_HIBYTE, 0x10}, //
    {0xB1, 0x05, 0x10}, // PGEN_LINE_SIZE0
    {0xB2, AR0233_WIDTH_LOBYTE, 0x10}, //
    {0xB1, 0x06, 0x10}, // PGEN_BAR_SIZE1
    {0xB2, BAR_WIDTH_HIBYTE, 0x10}, //
    {0xB1, 0x07, 0x10}, // PGEN_BAR_SIZE0
    {0xB2, BAR_WIDTH_LOBYTE, 0x10}, //
    {0xB1, 0x08, 0x10}, // PGEN_ACT_LPF1
    {0xB2, AR0233_HEIGHT_HIBYTE, 0x10}, //
    {0xB1, 0x09, 0x10}, // PGEN_ACT_LPF0
    {0xB2, AR0233_HEIGHT_LOBYTE, 0x10}, //
    {0xB1, 0x0A, 0x10}, // PGEN_TOT_LPF1
    {0xB2, 0x04, 0x10}, //
    {0xB1, 0x0B, 0x10}, // PGEN_TOT_LPF0
    {0xB2, 0x1A, 0x10}, //
    {0xB1, 0x0C, 0x10}, // PGEN_LINE_PD1
    {0xB2, 0x0C, 0x10}, //
    {0xB1, 0x0D, 0x10}, // PGEN_LINE_PD0
    {0xB2, 0x67, 0x10}, //
    {0xB1, 0x0E, 0x10}, // PGEN_VBP
    {0xB2, 0x21, 0x10}, //
    {0xB1, 0x0F, 0x10}, // PGEN_VF
#endif

    {0xFFFF, 0x00, 0x0} //End of script
};


#define AR0220_SER_CFG_SIZE    (7U)
I2cParams ub953SerCfg_AR0220[AR0220_SER_CFG_SIZE] = {
    {0x01, 0x02, 1000},    // RESET_CTL
                           //    Resets the entire digital block including registers. This bit is self-clearing.

    {0x02, 0x73,   20},    // General_CFG
                           //    Continuous Clock; 4-lane configuration
                           //    Transmitter CRC Generator Enable
                           //    1.8V

    // {0x03, 0x12,   20},    // FC_MODE_SEL

    {0x06, 0x41,   20},    // CLKOUT_CTRL0
                           //    HS_CLK_DIV: 010: Div by 4
                           //    DIV_M_VAL: M value for M/N divider for CLKOUT

    {0x07, 0x22,   20},    // CLKOUT_CTRL1
                           //    N value for M/N divider for CLKOUT

    {0x0E, 0x3C,   20},    // GPIO_INPUT_CTRL
    {0x0D, 0x01,   20},    // LOCAL_GPIO_DATA

    {0xFFFF, 0x00, 00},    // End of script
};


#define AR0220_DES_CFG_SIZE    (18U)
I2cParams ub960DesCfg_AR0220[AR0220_DES_CFG_SIZE] = {
    {0x01, 0x02, 0x200},    // RESET_CTL, Digital Reset
    {0x1f, 0x00, 0x10},     // CSI_PLL_CTL

    {0x0D, 0x90, 0x10},     // I/O to 3V3 - Options not valid with datashee
    {0x0C, 0x01, 0x10},     // Enable Rx port 0

    {0x4C, 0x01, 0x10},     // Select Channel 0
    {0x58, 0x5E, 0x10},     // Enable Back channel, set to 50Mbs
    {0x32, 0x01, 0x10},     // Enable Tx port 0
    {0x33, 0x02, 0x10},     // Enable Continuous clock mode and CSI output
    {0xBC, 0x00, 0x10},     // Frame Valid Minimum Time, FIXME
    {0x5C, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5D, (SER_0_I2C_ADDR << 1),  0x10},   // Serializer I2C Address
    {0x65, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5E, (AR0233_I2C_ADDR << 1), 0x10},   // Sensor I2C Address
    {0x66, (SENSOR_0_I2C_ALIAS << 1), 0x10},
    {0x6D, 0x6C, 0x10}, /*CSI Mode*/
    {0x72, 0x00, 0x10}, /*VC Map - All to 0 */

    {0x20, 0xE0, 0x10}, /*Forwarding and using CSIport 0 */

    {0xFFFF, 0x00, 0x00} //End of script
};


#define IMX390_D3_DES_CFG_SIZE_AR0220    (63)
I2cParams ub960DesCfg_D3IMX390_AR0220_0[IMX390_D3_DES_CFG_SIZE_AR0220] = {
    {0x01, 0x02, 0x20},    // RESET_CTL, Digital Reset
    {0x1f, 0x00, 0x00},    // CSI_PLL_CTL

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
    {0x5C, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_0_I2C_ALIAS << 1U), 0x1},
    {0x5E, (AR0233_I2C_ADDR << 1), 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_0_I2C_ALIAS << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0x00,0x0}, /*VC Map - All to 0 */

    {0x4C, 0x12, 0x10}, /* 0x12 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5C, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_1_I2C_ALIAS << 1U), 0x1},
    {0x5E, (AR0233_I2C_ADDR << 1), 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_1_I2C_ALIAS << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0x55,0x0}, /*VC Map - All to 1 */

    {0x4C, 0x24, 0x10}, /* 0x24 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5C, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_2_I2C_ALIAS<< 1U), 0x1},
    {0x5E, (AR0233_I2C_ADDR << 1), 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_2_I2C_ALIAS << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0xaa,0x0}, /*VC Map - All to 2 */

    {0x4C, 0x38, 0x10}, /* 0x38 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5C, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_3_I2C_ALIAS << 1U), 0x1},
    {0x5E, (AR0233_I2C_ADDR << 1), 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_3_I2C_ALIAS<< 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0xFF,0x0}, /*VC Map - All to 3 */
    {0xFFFF, 0x00, 0x0} //End of script
};

I2cParams ub960DesCfg_D3IMX390_AR0220_1[IMX390_D3_DES_CFG_SIZE_AR0220] = {
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
    {0x5C, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_4_I2C_ALIAS << 1U), 0x1},
    {0x5E, (AR0233_I2C_ADDR << 1), 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_4_I2C_ALIAS << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0x00,0x0}, /*VC Map - All to 0 */

    {0x4C, 0x12, 0x10}, /* 0x12 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5C, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_5_I2C_ALIAS << 1U), 0x1},
    {0x5E, (AR0233_I2C_ADDR << 1), 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_5_I2C_ALIAS << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0x55,0x0}, /*VC Map - All to 1 */

    {0x4C, 0x24, 0x10}, /* 0x24 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5C, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_6_I2C_ALIAS<< 1U), 0x1},
    {0x5E, (AR0233_I2C_ADDR << 1), 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_6_I2C_ALIAS << 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0xaa,0x0}, /*VC Map - All to 2 */

    {0x4C, 0x38, 0x10}, /* 0x38 */
    {0x32, 0x01, 0x1}, /*Enable TX port 0*/
    {0x33, 0x02, 0x1}, /*Enable Continuous clock mode and CSI output*/
    {0xBC, 0x00, 0x1}, /*Unknown*/
    {0x5C, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5D, 0x30, 0x1}, /*Serializer I2C Address*/
    {0x65, (SER_7_I2C_ALIAS << 1U), 0x1},
    {0x5E, (AR0233_I2C_ADDR << 1), 0x1}, /*Sensor I2C Address*/
    {0x66, (SENSOR_7_I2C_ALIAS<< 1U), 0x1},
    {0x6D, 0x6C,0x0}, /*CSI Mode*/
    {0x72, 0xFF,0x0}, /*VC Map - All to 3 */
    {0xFFFF, 0x00, 0x0} //End of script
};

#define AR0233_DES_CFG_SIZE    (66U)
I2cParams ub960DesCfg_AR0233[AR0233_DES_CFG_SIZE] = {
    {0x1, 0x2, 1000},
    {0x1f, 0x5, 1000},
    {0xB0, 0x1C,0x1},  
    {0xB1, 0x16,0x1},  
    {0xB2, 0x00,0x1},  
    {0xB1, 0x17,0x1},  
    {0xB2, 0x00,0x1},  
    {0xB1, 0x18,0x1},  
    {0xB2, 0x00,0x1},  
    {0xB1, 0x19,0x1},  
    {0xB2, 0x00,0x1},  
    {0xB0, 0x1C,0x1},  
    {0xB1, 0x15,0x1},  
    {0xB2, 0x0A,0x1},  
    {0xB2, 0x00,0x10}, 

    {0x0D, 0x90, 0x1}, /*I/O to 3V3 - Options not valid with datashee*/
    {0x0C, 0x0F, 0x1}, /*Enable All ports*/                            

    /*Select Channel 0*/                                               
    {0x4C, 0x01, 0x1},
    {0x58, 0x5e, 0x1},
    {0x72, 0x00, 0x1}, /*VC map*/                                      

    /*Select Channel 1*/                                               
    {0x4C, 0x12, 0x1},
    {0x58, 0x5e, 0x1},/*Enable Back channel, set to 50Mbs*/            

    /*Select Channel 2*/                                               
    {0x4C, 0x24, 0x1},
    {0x58, 0x5e, 0x1},/*Enable Back channel, set to 50Mbs*/            

    /*Select Channel 3*/                                               
    {0x4C, 0x38, 0x1}, 
    {0x58, 0x5e, 0x1},/*Enable Back channel, set to 50Mbs*/            

    /*Select Channel 0*/                                               
    {0x4C, 0x01, 0x1}, 
    {0xB0, 0x04, 0x1}, 
    {0xB1, 0x03, 0x1}, 
    {0xB2, 0x20, 0x1}, 
    {0xB1, 0x13, 0x1}, 
    {0xB2, 0x20, 0x1}, 
    {0xB0, 0x04, 0x1}, 
    {0xB1, 0x04, 0x1}, 
    {0xB2, 0x3F, 0x1}, 
    {0xB1, 0x14, 0x1}, 
    {0xB2, 0x3F, 0x1}, 
    {0x42, 0x71, 0x1}, /*Unknown*/                                     
    {0x41, 0xF0, 0x1}, /*Unknown*/                                     
    {0xB9, 0x18, 0x1}, 

    {0x32, 0x01, 0x1}, /*Enable TX port 0*/                            
    {0x20, 0x00, 0x1}, /*Forwarding and using CSIport 0 */             
                       
    /*Sets GPIOS*/     
    {0x10, 0x83, 0x1}, 
    {0x11, 0xA3, 0x1}, 
    {0x12, 0xC3, 0x1}, 
    {0x13, 0xE3, 0x1}, 

    {0x4C, 0x01, 0x1},
    {0x58, 0x5e, 0x1},
    {0x5C, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5D, (SENSOR_0_I2C_ALIAS << 1), 0x10},
    {0x65, (SER_0_I2C_ALIAS << 1), 0x10},
    {0x5E, (AR0233_I2C_ADDR << 1), 0x10},
    {0x66, (SENSOR_0_I2C_ALIAS << 1), 0x10},
    {0x6D, 0x6C,0x1}, /*CSI Mode*/                                     
    {0x72, 0x00,0x1}, /*VC Map - All to 0 */                           
    {0x7C, 0x20, 0x10}, /*Line Valid active high, Frame Valid active high*/
    {0xD5, 0xF3, 0x10}, /*Auto Attenuation*/       
    {0xB0,0x1C, 0x1},  
    {0xB1,0x15, 0x1},  
    {0xB2,0x0A, 0x1},  
    {0xB2,0x00, 0x1},                      

    {0x32, 0x21, 0x1},
    {0x33, 0x02, 0x1},

    {0x21, 0x03, 0x1},
    {0x20, 0x00, 0x1},
    {0xFFFF, 0x00, 0x0} //End of script
};

I2cParams ub960AR0233DesCSI2Enable[2u] = {
    {0x33, 0x03, 0x10},
    {0xFFFF, 0x00, 0x0} //End of script
};

I2cParams ub960AR0233DesCSI2Disable[2u] = {
    {0x33, 0x02, 0x10},
    {0xFFFF, 0x00, 0x0} //End of script
};

#endif /* _AR0233_SERDES_H_ */


