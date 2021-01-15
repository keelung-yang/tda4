"""
    DCC XML Generator Functions
    
    VISS RGB2RGB
"""

import os
import dcc
import dccxml

def GenRGB2RGB1Params(handle, rgb2rgb_params, cls_id):
    handle.write('                {\n')
    handle.write('                    { 256,   0,   0 , 0},\n')
    handle.write('                    {   0, 256,   0 , 0},\n')
    handle.write('                    {   0,   0, 256 , 0}\n')
    handle.write('                },\n')
    handle.write('                {0,0,0}\n')

def GenRGB2RGBXML(directory, filebase, params, RGB2RGB_params):

    if (os.path.exists(directory) == False):
        print ('Creating directory:  %s\n' %directory)
        try:
            os.makedirs(directory)
        except OSError as err:
            utils.error('%s: %s' %(err.strerror, err.filename), skip=True)

    filename = os.path.join(directory, '%s_%s.xml' %(params['SENSOR'], filebase));
    print ('Creating XML File:  %s\n' %filename)
    	
    module_params = {}
    module_params['NAME']   = 'ipipe_rgb2rgb_dcc.xml'
    module_params['STRUCT_NAME'] = 'cfg_rgb2rgb'
    module_params['DCC_ID'] = dcc.DCC_ID_IPIPE_RGB_RGB_1
    module_params['FUNC_GENPARAMS'] = GenRGB2RGB1Params
    
    handle = dccxml.OpenFile(filename)
    dccxml.GenHeader(handle, params, module_params)
    
    # Create the DCC Structure definition
    handle.write('    <!--=======================================================================-->\n')
    handle.write('  <typedef>\n')
    handle.write('			<cfg_rgb2rgb type="struct">\n')
    handle.write('        <!-- [RR] [GR] [BR] [CR]\n')
    handle.write('             [RG] [GG] [BG] [CG]\n')
    handle.write('             [RB] [GB] [BB] [CB]-->\n')
    handle.write('        <!-- Blending + Chroma values (S12Q8 format) -->\n')
    handle.write('			    <ccm type="int16[3][4]"> </ccm> <!-- ipipe rgb2rgb matrix: S12Q8 -->\n')
    handle.write('					<!-- Blending offset value for R,G,B - (S13) -->\n')
    handle.write('					<offset type="int16[3]"> </offset> <!-- ipipe rgb2rgb1 offset: S13 -->\n')
    handle.write('    </cfg_rgb2rgb>\n')
    handle.write('</typedef>\n')
    handle.write('<!--=======================================================================-->\n')
    
    # Create a DCC Use Case
    for i in range(RGB2RGB_params['NUM_USE_CASE']):
        dccxml.AddUseCase(handle, module_params, RGB2RGB_params['USE_CASE'][i])
    dccxml.GenFooter(handle, module_params)
    dccxml.CloseFile(handle)
    return 