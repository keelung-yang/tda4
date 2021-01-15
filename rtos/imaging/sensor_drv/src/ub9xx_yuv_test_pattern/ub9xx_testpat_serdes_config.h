#ifndef _UB96XTESTPAT_SERDES_H_
#define _UB96XTESTPAT_SERDES_H_

#define TESTPAT_OUT_WIDTH   		(640U)
#define TESTPAT_OUT_HEIGHT  		(480U)
#define TESTPAT_TOTAL_LINES  		(TESTPAT_OUT_HEIGHT+45)

#define TESTPAT_DES_CFG_SIZE    (36U)

/*UYVY*/
#define TESTPAT_OUT_BYTES_PER_LINE (TESTPAT_OUT_WIDTH * 2 )

#define TESTPAT_BPLN_HIGH ((TESTPAT_OUT_BYTES_PER_LINE & 0xFF00)>>8)
#define TESTPAT_BPLN_LOW  (TESTPAT_OUT_BYTES_PER_LINE & 0x00FF)

#define TESTPAT_OUT_HEIGHT_HIGH ((TESTPAT_OUT_HEIGHT & 0xFF00)>>8)
#define TESTPAT_OUT_HEIGHT_LOW  (TESTPAT_OUT_HEIGHT & 0x00FF)

#define TESTPAT_TOTAL_LINES_HIGH ((TESTPAT_TOTAL_LINES & 0xFF00)>>8)
#define TESTPAT_TOTAL_LINES_LOW  (TESTPAT_TOTAL_LINES & 0x00FF)

I2cParams ub9xxDesCfg_testpat[TESTPAT_DES_CFG_SIZE] = {
    {0x32, 0x01, 0x1},                            
    {0x33, 0x02, 0x1},                
    {0xB0, 0x02, 0x1},                // IA_AUTO_INC=1
    {0xB1, 0x01, 0x1},                // PGEN_CTL
    {0xB2, 0x01, 0x1},                // PGEN_ENABLE=1
    {0xB2, 0x34, 0x1},                // PGEN_CFG
    {0xB2, 0x1E, 0x1},                // PGEN_CSI_DI
    {0xB2, TESTPAT_BPLN_HIGH, 0x1},                // PGEN_LINE_SIZE1
    {0xB2, TESTPAT_BPLN_LOW, 0x1},                // PGEN_LINE_SIZE0
    {0xB2, 0x00, 0x1},                // PGEN_BAR_SIZE1
    {0xB2, 0xA0, 0x1},                // PGEN_BAR_SIZE0
    {0xB2, TESTPAT_OUT_HEIGHT_HIGH, 0x1},                // PGEN_ACT_LPF1
    {0xB2, TESTPAT_OUT_HEIGHT_LOW, 0x1},                // PGEN_ACT_LPF0
    {0xB2, TESTPAT_TOTAL_LINES_HIGH, 0x1},                // PGEN_TOT_LPF1
    {0xB2, TESTPAT_TOTAL_LINES_LOW, 0x1},                // PGEN_TOT_LPF0
    {0xB2, 0x18, 0x1},                // PGEN_LINE_PD1
    {0xB2, 0xCD, 0x1},                // PGEN_LINE_PD0
    {0xB2, 0x21, 0x1},                // PGEN_VBP
    {0xB2, 0x0A, 0x1},                // PGEN_VFP
    {0xB2, 0xAA, 0x1},                // PGEN_COLOR0
    {0xB2, 0x33, 0x1},                // PGEN_COLOR1
    {0xB2, 0xF0, 0x1},                // PGEN_COLOR2
    {0xB2, 0x7F, 0x1},                // PGEN_COLOR3
    {0xB2, 0x55, 0x1},                // PGEN_COLOR4
    {0xB2, 0xCC, 0x1},                // PGEN_COLOR5
    {0xB2, 0x0F, 0x1},                // PGEN_COLOR6
    {0xB2, 0x80, 0x1},                // PGEN_COLOR7
    {0xB2, 0x00, 0x1},                // PGEN_COLOR8
    {0xB2, 0x00, 0x1},                // PGEN_COLOR9
    {0xB2, 0x00, 0x1},                // PGEN_COLOR10
    {0xB2, 0x00, 0x1},                // PGEN_COLOR11
    {0xB2, 0x00, 0x1},                // PGEN_COLOR12
    {0xB2, 0x00, 0x1},                // PGEN_COLOR13
    {0xB2, 0x00, 0x1},                // PGEN_COLOR14
    {0xB2, 0x00, 0x1},                // Reserved
	{0xFFFF, 0x00, 0x0} /*End of script */
};

I2cParams ub9xxtestpatDesCSI2Enable[10u] = {
    {0x33, 0x03, 0x1},
    {0xFFFF, 0x00, 0x0} //End of script
};

I2cParams ub9xxtestpatDesCSI2Disable[2u] = {
    {0x33, 0x02, 0x10},
    {0xFFFF, 0x00, 0x0} /*End of script */
};

#endif /* _UB96XTESTPAT_SERDES_H_ */

