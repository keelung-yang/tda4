DCC_TOOL_PATH=../../../../../tools/dcc_tools/
OUT_PATH=../../../../include

rm *.bin
bin_folder=../../dcc_bins/
if [ ! -d "$bin_folder" ]
then
    mkdir "$bin_folder"
fi

bin_folder=../../dcc_bins/AR0233-UB953_MARS/
if [ ! -d "$bin_folder" ]
then
    mkdir "$bin_folder"
fi

bin_folder=../../dcc_bins/AR0233-UB953_MARS/linear/
if [ ! -d "$bin_folder" ]
then
    mkdir "$bin_folder"
fi

rm $OUT_PATH/dcc_viss_ar0233_linear.h
$DCC_TOOL_PATH/dcc_gen_linux ar0233_rgb2rgb_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux ar0233_h3a_aewb_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux ar0233_viss_nsf4.xml
$DCC_TOOL_PATH/dcc_gen_linux ar0233_viss_blc.xml
$DCC_TOOL_PATH/dcc_gen_linux ar0233_cfa_dcc.xml
$DCC_TOOL_PATH/dcc_gen_linux ar0233_h3a_mux_luts_dcc.xml
cp *.bin $bin_folder/
cat *.bin > ../../dcc_bins/dcc_viss.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_viss.bin $OUT_PATH/dcc_viss_ar0233.h dcc_viss_ar0233

rm *.bin
rm $OUT_PATH/dcc_2a_ar0233_linear.h
$DCC_TOOL_PATH/dcc_gen_linux ar0233_awb_alg_ti3_tuning.xml
$DCC_TOOL_PATH/dcc_gen_linux ar0233_h3a_aewb_dcc.xml
cp *.bin $bin_folder/
cat *.bin > ../../dcc_bins/dcc_2a.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_2a.bin $OUT_PATH/dcc_2a_ar0233.h dcc_2a_ar0233

rm *.bin
rm $OUT_PATH/dcc_ldc_ar0233_linear.h
$DCC_TOOL_PATH/dcc_gen_linux ar0233_mesh_ldc_dcc.xml
cp *.bin $bin_folder/
cat *.bin > ../../dcc_bins/dcc_ldc.bin
$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_ldc.bin $OUT_PATH/dcc_ldc_ar0233.h dcc_ldc_ar0233

rm *.bin
