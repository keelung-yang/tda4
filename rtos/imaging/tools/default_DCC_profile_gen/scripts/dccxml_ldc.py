"""
    DCC XML Generator Functions
    
    Mesh LDC
"""

import os
import shutil
import dcc
import dccxml
   
def GenLDCParams(handle, LDC_params, cls_id):
	return

def GenLDCXML(directory, filebase, params, LDC_params):

    if (os.path.exists(directory) == False):
        print ('Creating directory:  %s\n' %directory)
        try:
            os.makedirs(directory)
        except OSError as err:
            utils.error('%s: %s' %(err.strerror, err.filename), skip=True)

    filename = os.path.join(directory, '%s_%s.xml' %(params['SENSOR'], filebase));
    print ('Creating XML File:  %s\n' %filename	)

    module_params = {}
    module_params['NAME']   = 'ldc_odc'
    module_params['STRUCT_NAME'] = 'cfg_ldc_vars'
    module_params['DCC_ID'] = dcc.DCC_ID_MESH_LDC
    module_params['FUNC_GENPARAMS'] = GenLDCParams
    width = params['SENSOR_WIDTH']
    height = params['SENSOR_HEIGHT']

    handle = dccxml.OpenFile(filename)
    dccxml.GenHeader(handle, params, module_params)
    
    # Create the DCC Structure definition
    handle.write('  <!--=======================================================================-->\n')
    handle.write('      <typedef>\n')
    handle.write('      <%s type="struct"> \n' %module_params['STRUCT_NAME'])

    handle.write('                      <ldc_en type="uint16"> </ldc_en>\n')
    handle.write('                      <ldc_ldmapen type="uint16"> </ldc_ldmapen>\n')
    handle.write('                      <ldc_datamode type="uint16"> </ldc_datamode>\n')
    handle.write('                      <ldc_opdatamode type="uint16"> </ldc_opdatamode>\n')
    handle.write('                      <ldc_ip_dfmt type="uint16"> </ldc_ip_dfmt>\n')
    handle.write('                      <ldc_pwarpen type="uint16"> </ldc_pwarpen>\n')
    handle.write('                      <ldc_yint_typ type="uint16"> </ldc_yint_typ>\n')
    handle.write('                      <ldc_regmode_en type="uint16"> </ldc_regmode_en>\n')
    handle.write('                      <ldc_meshtable_m type="uint16"> </ldc_meshtable_m>\n')
    handle.write('                      <ldc_mesh_frsz_w type="uint16"> </ldc_mesh_frsz_w>\n')
    handle.write('                      <ldc_mesh_frsz_h type="uint16"> </ldc_mesh_frsz_h>\n')
    handle.write('                      <ldc_compute_frsz_w type="uint16"> </ldc_compute_frsz_w>\n')
    handle.write('                      <ldc_compute_frsz_h type="uint16"> </ldc_compute_frsz_h>\n')
    handle.write('                      <ldc_initx type="uint16"> </ldc_initx>\n')
    handle.write('                      <ldc_inity type="uint16"> </ldc_inity>\n')
    handle.write('                      <ldc_input_frsz_w type="uint16"> </ldc_input_frsz_w>\n')
    handle.write('                      <ldc_input_frsz_h type="uint16"> </ldc_input_frsz_h>\n')
    handle.write('                      <ldc_obw type="uint16"> </ldc_obw>\n')
    handle.write('                      <ldc_obh type="uint16"> </ldc_obh>\n')
    handle.write('                      <ldc_pixpad type="uint16"> </ldc_pixpad>\n')
    handle.write('                      <ldc_a type="int16"> </ldc_a>\n')
    handle.write('                      <ldc_b type="int16"> </ldc_b>\n')
    handle.write('                      <ldc_c type="int16"> </ldc_c>\n')
    handle.write('                      <ldc_d type="int16"> </ldc_d>\n')
    handle.write('                      <ldc_e type="int16"> </ldc_e>\n')
    handle.write('                      <ldc_f type="int16"> </ldc_f>\n')
    handle.write('                      <ldc_g type="int16"> </ldc_g>\n')
    handle.write('                      <ldc_h type="int16"> </ldc_h>\n')
    handle.write('                      <ldc_sf_width type="uint16[3]"> </ldc_sf_width>\n')
    handle.write('                      <ldc_sf_height type="uint16[3]"> </ldc_sf_height>\n')
    handle.write('                      <ldc_sf_en  type="uint16[3][3]"> </ldc_sf_en>\n')
    handle.write('                      <ldc_sf_obw type="uint16[3][3]"> </ldc_sf_obw>\n')
    handle.write('                      <ldc_sf_obh type="uint16[3][3]"> </ldc_sf_obh>\n')
    handle.write('                      <ldc_sf_pad type="uint16[3][3]"> </ldc_sf_pad>\n')
    handle.write('                      <ldc_ylut_en type="uint16"> </ldc_ylut_en>\n')
    handle.write('                      <ldc_yin_bitdpth type="uint16"> </ldc_yin_bitdpth>\n')
    handle.write('                      <ldc_yout_bitdpth type="uint16"> </ldc_yout_bitdpth>\n')
    handle.write('                      <ldc_clut_en type="uint16"> </ldc_clut_en>\n')
    handle.write('                      <ldc_cin_bitdpth type="uint16"> </ldc_cin_bitdpth>\n')
    handle.write('                      <ldc_cout_bitdpth type="uint16"> </ldc_cout_bitdpth>\n')
    handle.write('                      <ldc_y_lut type="uint16[513]"> </ldc_y_lut>\n')
    handle.write('                      <ldc_c_lut type="uint16[513]"> </ldc_c_lut>\n')
    handle.write('              		<mesh_table_pitch_in_bytes type="uint32"> </mesh_table_pitch_in_bytes>\n')
    handle.write('              		<mesh_table_size type="uint32"> </mesh_table_size>\n')
    handle.write('              		<mesh_lut type="uint16*"> </mesh_lut>\n')

    handle.write('      </%s> \n' %module_params['STRUCT_NAME'])
    
    handle.write('   </typedef>\n')
    handle.write('  <!--=======================================================================-->\n')


    handle.write('		<use_case val="65535"> \n')
    handle.write('        <usecase_general>\n')
    handle.write('            <ldc_dcc type="%s" main="general">\n' %module_params['STRUCT_NAME'])

    handle.write('            {\n')
    handle.write('                0     // LDC_CTRL  LDC_EN(0)         LDC Enable, 0: Disable,  1: Enable\n')
    handle.write('                1     // LDC_CTRL  LDMAPEN(1)        LD Mapping enable, 0: disable, 1: enable\n')
    handle.write('                2     // LDC_CTRL  DATAMODE(4:3)     Input data mode, 0:YUV422, 1:Y only, 2:YUV420, 3:YUV420 UV\n')
    handle.write('                1     // LDC_CTRL  OP_DATAMODE  Output data mode, 0: keep UYVY; 1: convert to 420\n')
    handle.write('                0     // LDC_CTRL  IP_DFMT(6:5) Input pixel format, 0:8b, 1:12b packed, 2:12b unpacked\n')
    handle.write('                1     // LDC_CTRL  PWARPEN(7)   Perspective warp 0: Disable . 1: Enable\n')
    handle.write('                1     // LDC_CFG   YINT_TYP(6)  Interpolation type for Y .  0: Bicubic,  1: Bilinear\n')
    handle.write('                0     // LDC_CFG   REGMODE_EN           Region mode, 0: disable, 1: enable\n')
    handle.write('                3     // LDC_MESHTABLE_CFG     M(2:0)   Mesh table subsampling factor (0-7)\n')
    handle.write('                %d  // LDC_MESH_FRSZ       W(13:0)  Mesh frame width (0-8192)\n' %width)
    handle.write('                %d  // LDC_MESH_FRSZ       H(29:16)             Mesh frame height (0-8192)\n' %height)
    handle.write('                %d   // LDC_COMPUTE_FRSZ      W(13:0)  Compute width (0-8192)\n' %width)
    handle.write('                %d   // LDC_COMPUTE_FRSZ      H(29:16)             Compute height (0-8192)\n' %height)
    handle.write('                  0     // LDC_INITXY    INITX(13:0)  Output starting horizontal coordinate (0-8192)\n')
    handle.write('                  0     // LDC_INITXY    INITY(29:16) Output starting vertical coordinate (0-8192)\n')
    handle.write('                %d  // LDC_INPUT_FRSZ        W(29:16)             Input frame width\n' %width)
    handle.write('                %d  // LDC_INPUT_FRSZ        H(13:0)  Input frame height\n' %height)
    handle.write('                128    // LDC_BLOCK_SIZE        OBW(7:0)             Output block width (0-255)\n')
    handle.write('                64    // LDC_BLOCK_SIZE        OBH(15:8)            Output block height (0-255)\n')
    handle.write('                1     // LDC_BLOCK_SIZE        PIXPAD(19:16)        Pixel pad (0-15)\n')
    handle.write('                4096  // LDC_AB    A(15:0)  Affine Transform warp, A S16Q12\n')
    handle.write('                0     // LDC_AB    B(31:16)             Affine Transform warp, B S16Q12\n')
    handle.write('                0     // LDC_CD    C(15:0)  Affine Transform warp, C S16Q3\n')
    handle.write('                0     // LDC_CD    D(31:16)             Affine Transform warp, D S16Q12\n')
    handle.write('                4096  // LDC_EF    E(15:0)  Affine Transform warp, E S16Q12\n')
    handle.write('                0     // LDC_EF    F(31:16)             Affine Transform warp, F S16Q3\n')
    handle.write('                0     // LDC_GH    G(15:0)  Affine Transform warp, G S16Q23\n')
    handle.write('                0     // LDC_GH    H(31:16)             Affine Transform warp, H S16Q23\n')
    handle.write('                {0, 0, 0}     //ldc_sf_width [3]\n')
    handle.write('                {0, 0, 0}     //ldc_sf_height[3]\n')
    handle.write('                {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}     //ldc_sf_en [3][3]\n')
    handle.write('                {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}     //ldc_sf_obw[3][3]\n')
    handle.write('                {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}     //ldc_sf_obh[3][3]\n')
    handle.write('                {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}     //ldc_sf_pad[3][3]\n')
    handle.write('                0     // LDC_DUALOUT_CFG       YLUT_EN  Luma LUT enable (0-1)\n')
    handle.write('                8     // LDC_DUALOUT_CFG       YIN_BITDPTH          Luma input bit depth (8-12)\n')
    handle.write('                8     // LDC_DUALOUT_CFG       YOUT_BITDPTH         Luma output bit depth (8-12)\n')
    handle.write('                0     // LDC_DUALOUT_CFG       CLUT_EN  Chroma LUT enable (0-1)\n')
    handle.write('                8     // LDC_DUALOUT_CFG       CIN_BITDPTH          Chroma input bit depth (8-12)\n')
    handle.write('                8     // LDC_DUALOUT_CFG       COUT_BITDPTH         Chroma output bit depth (8-12)\n')
    handle.write('                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},\n')

    handle.write('                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},\n')

    handle.write('                1,\n')
    handle.write('                1,\n')
    handle.write('                {0}\n')
    handle.write('            }\n')

    handle.write('            </ldc_dcc>\n')
    handle.write('        </usecase_general>\n')
    handle.write('		</use_case> \n')

    dccxml.GenFooter(handle, module_params)
    dccxml.CloseFile(handle)
