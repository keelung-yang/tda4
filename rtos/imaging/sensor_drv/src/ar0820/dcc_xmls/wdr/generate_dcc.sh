DCC_TOOL_PATH=../../../../../tools/dcc_tools/
OUT_PATH=../../../../include

rm *.bin
rm $OUT_PATH/dcc_viss_ar0820_wdr.h
$DCC_TOOL_PATH/dcc_gen_linux AR0820_ipipe_rgb2rgb_1_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux AR0820_viss_h3a_aewb_cfg.xml
$DCC_TOOL_PATH/dcc_gen_linux AR0820_viss_h3a_mux_luts_cfg.xml
$DCC_TOOL_PATH/dcc_gen_linux AR0820_viss_nsf4.xml
$DCC_TOOL_PATH/dcc_gen_linux AR0820_viss_blc.xml
$DCC_TOOL_PATH/dcc_gen_linux AR0820_flxd_cfa.xml
$DCC_TOOL_PATH/dcc_gen_linux AR0820_rawfe_decompand.xml
$DCC_TOOL_PATH/dcc_gen_linux AR0820_viss_glbce.xml

cat *.bin > ../../dcc_bins/dcc_viss_wdr.bin

$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_viss_wdr.bin $OUT_PATH/dcc_viss_ar0820_wdr.h dcc_viss_ar0820_wdr

  
rm *.bin
rm $OUT_PATH/dcc_2a_ar0820_wdr.h
$DCC_TOOL_PATH/dcc_gen_linux AR0820_awb_alg_ti3_tuning.xml
$DCC_TOOL_PATH/dcc_gen_linux AR0820_viss_h3a_aewb_cfg.xml
$DCC_TOOL_PATH/dcc_gen_linux AR0820_viss_h3a_mux_luts_cfg.xml
cat *.bin > ../../dcc_bins/dcc_2a_wdr.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_2a_wdr.bin $OUT_PATH/dcc_2a_ar0820_wdr.h dcc_2a_ar0820_wdr
  
rm *.bin
rm $OUT_PATH/dcc_ldc_ar0820_wdr.h
$DCC_TOOL_PATH/dcc_gen_linux AR0820_ldc.xml
cat *.bin > ../../dcc_bins/dcc_ldc_wdr.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_ldc_wdr.bin $OUT_PATH/dcc_ldc_ar0820_wdr.h dcc_ldc_ar0820_wdr

rm *.bin
