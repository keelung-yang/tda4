"""
    DCC XML Generator Functions
    
    VISS NSF4
"""

import os
import dcc
import dccxml
		
def GenNSF4Params(handle, nsf4_params, cls_id):

    handle.write('							0, //enable\n')
    handle.write('							16, //mode: 0 => nsf3 2x2 mode, 1~31 => nsf4 2x2 mode\n')
    handle.write('							0, //shading gain enable\n')
    
    handle.write('							32,    // U1  knee points for U,  U0.6 (0 ~ 63)\n')
    handle.write('							64,   //  Tn1 scaling factor,     U3.5 (1.0 = 32)\n')
    handle.write('							32,   //  Tn2 scaling factor,     U3.5 (1.0 = 32)\n')
    handle.write('							16,   //  Tn3 scaling factor,     U3.5 (1.0 = 32)\n')
    
    handle.write('							// Threshold parameters\n')
    handle.write('							//noise_thr_x\n')
    handle.write('							{\n')
    handle.write('								{0, 64, 256, 1024, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096},\n')
    handle.write('								{0, 64, 256, 1024, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096},\n')
    handle.write('								{0, 64, 256, 1024, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096},\n')
    handle.write('								{0, 64, 256, 1024, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096},\n')
    handle.write('							},\n')
    handle.write('							//noise_thr_y\n')
    handle.write('							{\n')
    handle.write('								{16, 20, 38, 76, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('								{16, 20, 38, 76, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('								{16, 20, 38, 76, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('								{16, 20, 38, 76, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('							},\n')
    handle.write('							//noise_thr_s\n')
    handle.write('							{\n')
    handle.write('								{128,  192,  100, 52, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('								{128,  192,  100, 52, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('								{128,  192,  100, 52, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('								{128,  192,  100, 52, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('							},\n')
    
    handle.write('							// Shading gain parameters\n')
    handle.write('							0, //shd_x\n')
    handle.write('							0, //shd_y\n')
    handle.write('							0, //shd_t\n')
    handle.write('							0, //shd_kh\n')
    handle.write('							0, //shd_kv\n')
    handle.write('							0, //shd_gmax\n')
    handle.write('							0, //shd_set_sel\n')
    
    handle.write('							//shd_lut_x\n')
    handle.write('							{\n')
    handle.write('								{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('								{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('							},\n')
    handle.write('							//shd_lut_y\n')
    handle.write('							{\n')
    handle.write('								{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('								{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('							},\n')
    handle.write('							//shd_lut_s\n')
    handle.write('							{\n')
    handle.write('								{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('								{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},\n')
    handle.write('							},\n')
    handle.write('						  //wb_gains\n')
    handle.write('						  {512, 512, 512, 512}\n')  


def GenNSF4XML(directory, filebase, params, nsf4_params):

    if (os.path.exists(directory) == False):
        print ('Creating directory:  %s\n' %directory)
        try:
            os.makedirs(directory)
        except OSError as err:
            utils.error('%s: %s' %(err.strerror, err.filename), skip=True)

    filename = os.path.join(directory, '%s_%s.xml' %(params['SENSOR'], filebase));
    print ('Creating XML File:  %s\n' %filename)
	
    module_params = {}
    module_params['NAME']   = 'VISS_NSF4_CFG'
    module_params['STRUCT_NAME'] = 'viss_nsf4_cfg'
    module_params['DCC_ID'] = dcc.DCC_ID_NSF_CFG
    module_params['FUNC_GENPARAMS'] = GenNSF4Params
    
    handle = dccxml.OpenFile(filename)
    dccxml.GenHeader(handle, params, module_params)
    
    # Create the DCC Structure definition

    handle.write('  <!--=======================================================================-->\n')
    handle.write('    <typedef>\n')
    handle.write('		<%s type="struct">\n' %module_params['STRUCT_NAME'])

    handle.write('				  <enable type="int32"> </enable> <!-- enable nsf4-->\n')
    handle.write('				  <mode type="int32"> </mode>\n')
    handle.write('				  <shading_gain type="int32"> </shading_gain>\n')
    
    handle.write('				  <u1_knee type="int32"> </u1_knee>\n')
    handle.write('				  <tn1 type="int32"> </tn1>\n')
    handle.write('				  <tn2 type="int32"> </tn2>\n')
    handle.write('				  <tn3 type="int32"> </tn3>\n')
    
    handle.write('				  <noise_thr_x type="int32[4][12]"> </noise_thr_x>\n')
    handle.write('				  <noise_thr_y type="int32[4][12]"> </noise_thr_y>\n')
    handle.write('				  <noise_thr_s type="int32[4][12]"> </noise_thr_s>\n')
    
    handle.write('				  <shd_x type="int32"> </shd_x>\n')
    handle.write('				  <shd_y type="int32"> </shd_y>\n')
    handle.write('				  <shd_t type="int32"> </shd_t>\n')
    handle.write('				  <shd_kh type="int32"> </shd_kh>\n')
    handle.write('				  <shd_kv type="int32"> </shd_kv>\n')
    handle.write('				  <shd_gmax type="int32"> </shd_gmax>\n')
    handle.write('				  <shd_set_sel type="int32"> </shd_set_sel>\n')
    
    handle.write('				  <shd_lut_x type="int32[2][16]"> </shd_lut_x>\n')
    handle.write('				  <shd_lut_y type="int32[2][16]"> </shd_lut_y>\n')
    handle.write('				  <shd_lut_s type="int32[2][16]"> </shd_lut_s>\n')
    
    handle.write('				  <wb_gains type="int32[4]"> </wb_gains>\n')

    handle.write('        </%s>\n' %module_params['STRUCT_NAME'])
    handle.write('    </typedef>\n')
    handle.write('  <!--=======================================================================-->\n')
	

    # Create a DCC Use Case
    for i in range(nsf4_params['NUM_USE_CASE']):
        dccxml.AddUseCase(handle, module_params, nsf4_params['USE_CASE'][i])
    
    dccxml.GenFooter(handle, module_params)
    dccxml.CloseFile(handle)
