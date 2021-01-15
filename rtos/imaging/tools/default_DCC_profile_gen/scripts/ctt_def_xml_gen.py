import os
import utils
import sys
import dccxml
import dccxml_awb
import dccxml_rgb2rgb
import dccxml_cfai
import dccxml_h3a_aewb
import dccxml_ldc
import dccxml_viss_blc
import dccxml_glbce
import dccxml_nsf4
import dccxml_decompand
import dccxml_h3amux

version= '0.1'

def init_generic_params(params,sparams):
    #
    #    GENERIC SENSOR PARAMS
    #
    params['SENSOR'] = sparams['SENSOR_NAME']
    params['SENSOR_DCC_NAME'] = sparams['SENSOR_DCC_NAME']
    params['CID']    = int(sparams['SENSOR_ID'])
    params['VENDOR_ID'] = 1
    params['VERSION']   = 0
    params['SENSOR_WIDTH'] = int(sparams['WIDTH'])
    params['SENSOR_HEIGHT'] = int(sparams['HEIGHT'])
    params['BAYER_PATTERN']    = int(sparams['BAYER_PATTERN'])
    params['BIT_DEPTH']    = int(sparams['BIT_DEPTH'])
    
    return

def init_dcc_regions(sys_params,Params):
    #
    region = {}
    region['PREVIEW'] = {}
    region['PREVIEW']['NUM_REGIONS'] = 1
    region['PREVIEW']['NUM_VARS']    = 2
    region['PREVIEW']['VAR_ID']      = [   1,   0]
    region['PREVIEW']['RGN_LIMITS']  = [((     0,250000), (     0,     100000))]
    #
    #    DCC REGION DEFINITION
    #
    Params['NUM_USE_CASE'] = 1
    Params['USE_CASE'] = {}
    Params['SYS_PARAMS'] = {}
    Params['USE_CASE'][0] = {}
    #
    #    DCC REGION DEFINITION
    #
    #   ID = 65535
    Params['USE_CASE'][0]['ID'] = 65535
    Params['USE_CASE'][0]['RGN_DEFN'] = {}
    Params['USE_CASE'][0]['RGN_DEFN'] = region['PREVIEW']

    return

def generate_nsf4_xml(sys_params,nsf4_params,params):
    nsf4_File = 'viss_nsf4'
    nsf4_Dir  = sys_params['PRJ_DIR'] + '/dcc_xmls/'+params['WDR_MODE']
    dccxml_nsf4.GenNSF4XML(nsf4_Dir, nsf4_File, params, nsf4_params)
    #dccxml.GenBinary(nsf4_Dir, nsf4_File, params, sys_params)
    return

def generate_glbce_xml(sys_params,glbce_params,params):
    glbce_File = 'wdr_glbce_dcc'
    glbce_Dir  = sys_params['PRJ_DIR'] + '/dcc_xmls/'+params['WDR_MODE']
    dccxml_glbce.GenGLBCEXML(glbce_Dir, glbce_File, params, glbce_params)
    #dccxml.GenBinary(glbce_Dir, glbce_File, params, sys_params)
    return

def generate_viss_blc_xml(sys_params,viss_blc_params,params):
    viss_blc_File = 'viss_blc'
    viss_blc_Dir  = sys_params['PRJ_DIR'] + '/dcc_xmls/'+params['WDR_MODE']
    dccxml_viss_blc.GenVISS_BLCXML(viss_blc_Dir, viss_blc_File, params, viss_blc_params)
    #dccxml.GenBinary(viss_blc_Dir, viss_blc_File, params, sys_params)
    return

def generate_ldc_xml(sys_params,ldc_params,params):
    ldc_File = 'mesh_ldc_dcc'
    ldc_Dir  = sys_params['PRJ_DIR'] + '/dcc_xmls/'+params['WDR_MODE']
    dccxml_ldc.GenLDCXML(ldc_Dir, ldc_File, params, ldc_params)
    #dccxml.GenBinary(ldc_Dir, ldc_File, params, sys_params)
    return

def generate_h3a_aewb_xml(sys_params,h3a_aewb_params,params):
    H3A_AEWB_File = 'h3a_aewb_dcc'
    H3A_AEWB_Dir  = sys_params['PRJ_DIR'] + '/dcc_xmls/'+params['WDR_MODE']
    dccxml_h3a_aewb.GenH3A_AEWBXML(H3A_AEWB_Dir, H3A_AEWB_File, params, h3a_aewb_params)
    #dccxml.GenBinary(H3A_AEWB_Dir, H3A_AEWB_File, params, sys_params)
    return

def generate_h3amux_xml(sys_params,h3amux_params,params):
    h3amux_File = 'h3a_mux_luts_dcc'
    h3amux_Dir  = sys_params['PRJ_DIR'] + '/dcc_xmls/'+params['WDR_MODE']
    h3amux_params['WDR_MODE'] = params['WDR_MODE']
    dccxml_h3amux.GenH3AMUXXML(h3amux_Dir, h3amux_File, params, h3amux_params)
    #dccxml.GenBinary(h3amux_Dir, h3amux_File, params, sys_params)
    return

def generate_cfai_xml(sys_params,cfai_params,params):
    CFAI_File = 'cfa_dcc'
    CFAI_Dir  = sys_params['PRJ_DIR'] + '/dcc_xmls/'+params['WDR_MODE']
    dccxml_cfai.GenCFAIXML(CFAI_Dir, CFAI_File, params, cfai_params)
    #dccxml.GenBinary(CFAI_Dir, CFAI_File, params, sys_params)
    return

def generate_rgb2rgb1_xml(sys_params,rgb2rgb1_params,params):
    RGB2RGB1_File = 'rgb2rgb_dcc'
    RGB2RGB1_Dir  = sys_params['PRJ_DIR'] + '/dcc_xmls/'+params['WDR_MODE']
    dccxml_rgb2rgb.GenRGB2RGBXML(RGB2RGB1_Dir, RGB2RGB1_File, params, rgb2rgb1_params)
    #dccxml.GenBinary(RGB2RGB1_Dir, RGB2RGB1_File, params, sys_params)
    return

def generate_awb_xml(sys_params,awb_params,params):
    awb_File = 'awb_alg_ti3_tuning'
    awb_Dir  = sys_params['PRJ_DIR'] + '/dcc_xmls/'+params['WDR_MODE']
    dccxml_awb.GenAWBXML(awb_Dir, awb_File, params, awb_params)
    #dccxml.GenBinary(awb_Dir, awb_File, params, sys_params)
    return

def generate_decmp_xml(sys_params,decmp_params,params):
    decmp_File = 'wdr_decompand_dcc'
    decmp_Dir  = sys_params['PRJ_DIR'] + '/dcc_xmls/'+params['WDR_MODE']
    dccxml_decompand.GenDECMPXML(decmp_Dir, decmp_File, params, decmp_params)
    #dccxml.GenBinary(decmp_Dir, decmp_File, params, sys_params)
    return

def generate_dcc_gen_script(sys_params,params):
    dcc_gen_File = 'generate_dcc.sh'
    dcc_gen_Dir  = sys_params['PRJ_DIR'] + '/dcc_xmls/'+params['WDR_MODE']

    if (os.path.exists(dcc_gen_Dir) == False):
        print ('Creating directory:  %s\n' %dcc_gen_Dir)
        try:
            os.makedirs(dcc_gen_Dir)
        except OSError as err:
            utils.error('%s: %s' %(err.strerror, err.filename), skip=True)

    filename = os.path.join(dcc_gen_Dir, '%s' %(dcc_gen_File));
    print ('Creating XML File:  %s\n' %filename)

    handle = dccxml.OpenFile(filename)
    if(params['WDR_MODE'] == 'linear'):
        wdr_suffix = ""
    else:
        wdr_suffix = "_wdr"

    handle.write('DCC_TOOL_PATH=../../../../../tools/dcc_tools/\n')
    handle.write('OUT_PATH=../../../../include\n')
    handle.write('\n')
    handle.write('rm *.bin\n')

    handle.write('bin_folder=../../dcc_bins/\n')
    handle.write('if [ ! -d "$bin_folder" ]\n')
    handle.write('then\n')
    handle.write('    mkdir "$bin_folder"\n')
    handle.write('fi\n')
    handle.write('\n')
    handle.write('bin_folder=../../dcc_bins/%s/\n' %params['SENSOR_DCC_NAME'])
    handle.write('if [ ! -d "$bin_folder" ]\n')
    handle.write('then\n')
    handle.write('    mkdir "$bin_folder"\n')
    handle.write('fi\n')
    handle.write('\n')
    handle.write('bin_folder=../../dcc_bins/%s/%s/\n' %(params['SENSOR_DCC_NAME'], params['WDR_MODE']))
    handle.write('if [ ! -d "$bin_folder" ]\n')
    handle.write('then\n')
    handle.write('    mkdir "$bin_folder"\n')
    handle.write('fi\n')
    handle.write('\n')

    handle.write('rm $OUT_PATH/dcc_viss_%s_%s.h\n' %(params['SENSOR'], params['WDR_MODE']))
    handle.write('$DCC_TOOL_PATH/dcc_gen_linux %s_rgb2rgb_dcc.xml\n' %params['SENSOR'])
    handle.write('$DCC_TOOL_PATH/dcc_gen_linux %s_h3a_aewb_dcc.xml\n' %params['SENSOR'])
    handle.write('$DCC_TOOL_PATH/dcc_gen_linux %s_viss_nsf4.xml\n' %params['SENSOR'])
    handle.write('$DCC_TOOL_PATH/dcc_gen_linux %s_viss_blc.xml\n' %params['SENSOR'])
    handle.write('$DCC_TOOL_PATH/dcc_gen_linux %s_cfa_dcc.xml\n' %params['SENSOR'])
    if(params['WDR_MODE'] == 'wdr'):
        handle.write('$DCC_TOOL_PATH/dcc_gen_linux %s_wdr_decompand_dcc.xml\n' %params['SENSOR'])
        handle.write('$DCC_TOOL_PATH/dcc_gen_linux %s_wdr_glbce_dcc.xml\n' %params['SENSOR'])
    handle.write('$DCC_TOOL_PATH/dcc_gen_linux %s_h3a_mux_luts_dcc.xml\n' %params['SENSOR'])
    handle.write('cp *.bin %s/\n' %"$bin_folder")
    handle.write('cat *.bin > ../../dcc_bins/dcc_viss%s.bin\n' %wdr_suffix)
    handle.write('$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_viss%s.bin $OUT_PATH/dcc_viss_%s%s.h dcc_viss_%s%s\n' %(wdr_suffix, params['SENSOR'], wdr_suffix, params['SENSOR'], wdr_suffix))
    handle.write('\n')
    handle.write('rm *.bin\n')

    handle.write('rm $OUT_PATH/dcc_2a_%s_%s.h\n' %(params['SENSOR'], params['WDR_MODE']))
    handle.write('$DCC_TOOL_PATH/dcc_gen_linux %s_awb_alg_ti3_tuning.xml\n' %params['SENSOR'])
    handle.write('$DCC_TOOL_PATH/dcc_gen_linux %s_h3a_aewb_dcc.xml\n' %params['SENSOR'])
    handle.write('cp *.bin %s/\n' %"$bin_folder")
    handle.write('cat *.bin > ../../dcc_bins/dcc_2a%s.bin\n'  %wdr_suffix)
    handle.write('$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_2a%s.bin $OUT_PATH/dcc_2a_%s%s.h dcc_2a_%s%s\n' %(wdr_suffix, params['SENSOR'], wdr_suffix, params['SENSOR'], wdr_suffix))
    handle.write('\n')
    handle.write('rm *.bin\n')
    
    handle.write('rm $OUT_PATH/dcc_ldc_%s_%s.h\n' %(params['SENSOR'], params['WDR_MODE']) )
    handle.write('$DCC_TOOL_PATH/dcc_gen_linux %s_mesh_ldc_dcc.xml\n' %params['SENSOR'])
    handle.write('cp *.bin %s/\n' %"$bin_folder")
    handle.write('cat *.bin > ../../dcc_bins/dcc_ldc%s.bin\n' %wdr_suffix)
    handle.write('$DCC_TOOL_PATH/dcc_bin2c ../../dcc_bins/dcc_ldc%s.bin $OUT_PATH/dcc_ldc_%s%s.h dcc_ldc_%s%s\n' %(wdr_suffix, params['SENSOR'], wdr_suffix, params['SENSOR'], wdr_suffix))
    handle.write('\n')
    handle.write('rm *.bin\n')

    return

def defaultXMLGen_Simulator(sensor_config_file):
    print('\n Default Components : Version [%s]' %version)
    print('\n Config File  %s' %sensor_config_file)
    #print('\n Config File  %s' %sys.argv[1]
    #if (len(sys.argv) < 2 ) :
    #    utils.error('Usage default XML <input_filename>')
    #    return
    #sensor_config_file=sys.argv[1]
    """
    System Parameters Related to Project
    """
    sys_params ={}
    utils.init_params(sys_params)
    #
    utils.get_params(sensor_config_file,sys_params)
    #    
    
    for wdr_mode in ['linear', 'wdr']:
        GEN_Params = {}
        init_generic_params(GEN_Params,sys_params)
        GEN_Params['WDR_MODE'] = wdr_mode
        
        AWB_Params = {}
        init_dcc_regions(sys_params,AWB_Params)
        generate_awb_xml(sys_params,AWB_Params,GEN_Params)

        RGB2RGB1_Params = {}
        init_dcc_regions(sys_params,RGB2RGB1_Params)
        generate_rgb2rgb1_xml(sys_params,RGB2RGB1_Params,GEN_Params)

        CFAI_Params = {}
        init_dcc_regions(sys_params,CFAI_Params)
        generate_cfai_xml(sys_params,CFAI_Params,GEN_Params)

        H3A_AEWB_Params = {}
        init_dcc_regions(sys_params,H3A_AEWB_Params)
        generate_h3a_aewb_xml(sys_params,H3A_AEWB_Params,GEN_Params)

        LDC_Params = {}
        init_dcc_regions(sys_params,LDC_Params)
        generate_ldc_xml(sys_params,LDC_Params,GEN_Params)

        viss_blc_Params = {}
        init_dcc_regions(sys_params,viss_blc_Params)
        generate_viss_blc_xml(sys_params,viss_blc_Params,GEN_Params)
        
        NSF4_Params = {}
        init_dcc_regions(sys_params,NSF4_Params)
        generate_nsf4_xml(sys_params,NSF4_Params,GEN_Params)

        H3AMUX_Params = {}
        init_dcc_regions(sys_params,H3AMUX_Params)
        generate_h3amux_xml(sys_params,H3AMUX_Params,GEN_Params)

        if(wdr_mode=='wdr'):
            GLBCE_Params = {}
            init_dcc_regions(sys_params,GLBCE_Params)
            generate_glbce_xml(sys_params,GLBCE_Params,GEN_Params)

            DECMP_Params = {}
            init_dcc_regions(sys_params,DECMP_Params)
            generate_decmp_xml(sys_params,DECMP_Params,GEN_Params)
        
        generate_dcc_gen_script(sys_params,GEN_Params)
    
    return

if __name__ == '__main__':
    if (len(sys.argv) < 2 ) :
        utils.error('Usage: python ctt_def_xml_gen.py <sensor_properties_filename>')
    else:
        sensor_config_file=sys.argv[1]

        defaultXMLGen_Simulator(sensor_config_file)
        
