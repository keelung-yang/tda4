DCC_TOOL_PATH=../../../../../tools/dcc_tools/
OUT_PATH=../../../../include

rm *.bin
bin_folder=../../dcc_bins/
if [ ! -d "$bin_folder" ]
then
    mkdir "$bin_folder"
fi

bin_folder=../../dcc_bins/UB9xxx_RAW12_TESTPATTERN/
if [ ! -d "$bin_folder" ]
then
    mkdir "$bin_folder"
fi

bin_folder=../../dcc_bins/UB9xxx_RAW12_TESTPATTERN/linear/
if [ ! -d "$bin_folder" ]
then
    mkdir "$bin_folder"
fi

rm $OUT_PATH/dcc_viss_ub9xx_raw_test_pattern_linear.h
$DCC_TOOL_PATH/dcc_gen_linux ub9xx_raw_test_pattern_rgb2rgb_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux ub9xx_raw_test_pattern_h3a_aewb_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux ub9xx_raw_test_pattern_viss_nsf4.xml
$DCC_TOOL_PATH/dcc_gen_linux ub9xx_raw_test_pattern_viss_blc.xml
$DCC_TOOL_PATH/dcc_gen_linux ub9xx_raw_test_pattern_cfa_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux ub9xx_raw_test_pattern_h3a_mux_luts_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux ub9xx_raw_test_pattern_lsc_dcc.xml 
cp *.bin $bin_folder/
cat *.bin > ../../dcc_bins/dcc_viss.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_viss.bin $OUT_PATH/dcc_viss_ub9xx_raw_test_pattern.h dcc_viss_ub9xx_raw_test_pattern

rm *.bin
rm $OUT_PATH/dcc_2a_ub9xx_raw_test_pattern_linear.h
$DCC_TOOL_PATH/dcc_gen_linux ub9xx_raw_test_pattern_awb_alg_ti3_tuning.xml
$DCC_TOOL_PATH/dcc_gen_linux ub9xx_raw_test_pattern_h3a_aewb_dcc.xml
cp *.bin $bin_folder/
cat *.bin > ../../dcc_bins/dcc_2a.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_2a.bin $OUT_PATH/dcc_2a_ub9xx_raw_test_pattern.h dcc_2a_ub9xx_raw_test_pattern

rm *.bin
rm $OUT_PATH/dcc_ldc_ub9xx_raw_test_pattern_linear.h
$DCC_TOOL_PATH/dcc_gen_linux ub9xx_raw_test_pattern_mesh_ldc_dcc.xml
cp *.bin $bin_folder/
cat *.bin > ../../dcc_bins/dcc_ldc.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_ldc.bin $OUT_PATH/dcc_ldc_ub9xx_raw_test_pattern.h dcc_ldc_ub9xx_raw_test_pattern

rm *.bin
