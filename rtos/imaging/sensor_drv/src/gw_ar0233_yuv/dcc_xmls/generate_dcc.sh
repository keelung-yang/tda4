DCC_TOOL_PATH=../../../../tools/dcc_tools/
OUT_PATH=../../../include

rm *.bin
rm $OUT_PATH/dcc_ldc_gw_ar0233.h
$DCC_TOOL_PATH/dcc_gen_linux gw_ar0233_ldc.xml
cat *.bin > ../dcc_bins/dcc_ldc.bin
$DCC_TOOL_PATH/dcc_bin2c ../dcc_bins/dcc_ldc.bin $OUT_PATH/dcc_ldc_gw_ar0233.h dcc_ldc_gw_ar0233

rm *.bin
