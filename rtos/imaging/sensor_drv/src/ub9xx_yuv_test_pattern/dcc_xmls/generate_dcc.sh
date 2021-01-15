DCC_TOOL_PATH=../../../../tools/dcc_tools/
OUT_PATH=../../../include

rm *.bin
rm $OUT_PATH/dcc_ldc_ub96x_uyvytestpat.h
$DCC_TOOL_PATH/dcc_gen_linux testpat_ldc.xml
cat *.bin > ../dcc_bins/dcc_ldc.bin
$DCC_TOOL_PATH/dcc_bin2c ../dcc_bins/dcc_ldc.bin $OUT_PATH/dcc_ldc_ub96x_uyvytestpat.h dcc_ldc_ub96x_uyvytestpat

rm *.bin
