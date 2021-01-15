"""
    DCC XML Generator Functions

    H3A MUX
"""

import os
import dcc
import dccxml
import shutil

wdr_mode = 0

def GenH3AMUXParams(handle, h3amux_params, cls_id):
    handle.write('                1, //enable\n')
    handle.write('                1, //number of LUTs\n')
    handle.write('                {\n')

    if(wdr_mode == 1):
        handle.write('                    {#include "lut_h3a_16b_to_10b_g07.txt"},\n')
        handle.write('                    {#include "lut_h3a_16b_to_10b_g07.txt"},\n')
        handle.write('                    {#include "lut_h3a_16b_to_10b_g07.txt"},\n')
    elif(wdr_mode == 0):
        handle.write('                    {#include "lut_h3a_12b_to_10b.txt"},\n')
        handle.write('                    {#include "lut_h3a_12b_to_10b.txt"},\n')
        handle.write('                    {#include "lut_h3a_12b_to_10b.txt"},\n')
    
    handle.write('                },\n')

def GenH3AMUXXML(directory, filebase, params, h3amux_params):

    if (os.path.exists(directory) == False):
        print ('Creating directory:  %s\n' %directory)
        try:
            os.makedirs(directory)
        except OSError as err:
            utils.error('%s: %s' %(err.strerror, err.filename), skip=True)

    filename = os.path.join(directory, '%s_%s.xml' %(params['SENSOR'], filebase));
    print ('Creating XML File:  %s\n' %filename)
    
    global wdr_mode
    if(h3amux_params['WDR_MODE'] == 'wdr'):
        wdr_mode = 1
    else:
        wdr_mode = 0

    module_params = {}
    module_params['NAME']   = 'VISS_H3A_MUX_LUTS_CFG'
    module_params['STRUCT_NAME'] = 'iss_h3a_mux_luts'
    module_params['DCC_ID'] = dcc.DCC_ID_H3A_MUX
    module_params['FUNC_GENPARAMS'] = GenH3AMUXParams

    handle = dccxml.OpenFile(filename)
    dccxml.GenHeader(handle, params, module_params)

    # Create the DCC Structure definition
    handle.write('  <!--=======================================================================-->\n')
    handle.write('      <typedef>\n')
    handle.write('    <%s type="struct">\n' %module_params['STRUCT_NAME'])

    handle.write('       <enable type="uint16"> </enable> <!-- enable -->\n')
    handle.write('       <num_luts type="uint16"> </num_luts> <!-- number of LUTs (0 ~ 3) -->\n')
    handle.write('       <h3a_mux_luts type="uint16[3][639]"> </h3a_mux_luts> <!-- H3A LUTs -->\n')

    handle.write('      </%s>\n' %module_params['STRUCT_NAME'])
    handle.write('   </typedef>\n')
    handle.write('  <!--=======================================================================-->\n')

    # Create a DCC Use Case
    for i in range(h3amux_params['NUM_USE_CASE']):
        dccxml.AddUseCase(handle, module_params, h3amux_params['USE_CASE'][i])

    if(wdr_mode==1):
        r_table = '../tables/lut_h3a_16b_to_10b_g07.txt'
    else:
        r_table = '../tables/lut_h3a_12b_to_10b.txt'
    shutil.copy(r_table, directory)

    dccxml.GenFooter(handle, module_params)
    dccxml.CloseFile(handle)
