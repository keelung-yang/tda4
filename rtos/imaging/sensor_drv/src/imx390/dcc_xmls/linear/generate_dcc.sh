DCC_TOOL_PATH=../../../../../tools/dcc_tools/
OUT_PATH=../../../../include

rm *.bin
rm $OUT_PATH/dcc_viss_imx390.h
$DCC_TOOL_PATH/dcc_gen_linux IMX390_ipipe_rgb2rgb_1_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux IMX390_viss_h3a_aewb_cfg.xml
$DCC_TOOL_PATH/dcc_gen_linux IMX390_viss_nsf4.xml
$DCC_TOOL_PATH/dcc_gen_linux IMX390_viss_blc.xml
$DCC_TOOL_PATH/dcc_gen_linux IMX390_flxd_cfa.xml
$DCC_TOOL_PATH/dcc_gen_linux IMX390_rawfe_decompand.xml
$DCC_TOOL_PATH/dcc_gen_linux IMX390_viss_h3a_mux_luts_cfg.xml
cat *.bin > ../../dcc_bins/dcc_viss.bin
cp *.bin ../../dcc_bins/linear/
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_viss.bin $OUT_PATH/dcc_viss_imx390.h dcc_viss_imx390

rm *.bin
rm $OUT_PATH/dcc_2a_imx390.h
$DCC_TOOL_PATH/dcc_gen_linux IMX390_awb_alg_ti3_tuning.xml
$DCC_TOOL_PATH/dcc_gen_linux IMX390_viss_h3a_aewb_cfg.xml
cat *.bin > ../../dcc_bins/dcc_2a.bin
cp *.bin ../../dcc_bins/linear/
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_2a.bin $OUT_PATH/dcc_2a_imx390.h dcc_2a_imx390

rm *.bin
rm $OUT_PATH/dcc_ldc_imx390.h
$DCC_TOOL_PATH/dcc_gen_linux IMX390_ldc.xml
cat *.bin > ../../dcc_bins/dcc_ldc.bin
cp *.bin ../../dcc_bins/linear/
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_ldc.bin $OUT_PATH/dcc_ldc_imx390.h dcc_ldc_imx390

rm *.bin

