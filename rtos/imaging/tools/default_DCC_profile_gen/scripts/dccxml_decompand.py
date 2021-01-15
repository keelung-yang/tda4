"""
    DCC XML Generator Functions

    WDR Decompand
"""

import os
import dcc
import dccxml
import shutil

def GenDECMPParams(handle, decmp_params, cls_id):
    handle.write('		          1     // enable\n')
    handle.write('		          4095  // mask\n')
    handle.write('		          0     // shift\n')
    handle.write('		          12    // lutBitdepth\n')
    handle.write('		          65535 // lutClip\n')
    handle.write('		          {\n')
    handle.write('		              #include "lut_rawfe_pwl_vshort.txt"\n')
    handle.write('		          }\n')

def GenDECMPXML(directory, filebase, params, decmp_params):

    if (os.path.exists(directory) == False):
        print ('Creating directory:  %s\n' %directory)
        try:
            os.makedirs(directory)
        except OSError as err:
            utils.error('%s: %s' %(err.strerror, err.filename), skip=True)

    filename = os.path.join(directory, '%s_%s.xml' %(params['SENSOR'], filebase));
    print ('Creating XML File:  %s\n' %filename)

    module_params = {}
    module_params['NAME']   = 'VISS_RAWFE_decmp_cfg'
    module_params['STRUCT_NAME'] = 'cfg_rawfe_decompand'
    module_params['DCC_ID'] = dcc.DCC_ID_IPIPE_DECMP
    module_params['FUNC_GENPARAMS'] = GenDECMPParams

    handle = dccxml.OpenFile(filename)
    dccxml.GenHeader(handle, params, module_params)

    # Create the DCC Structure definition
    handle.write('  <!--=======================================================================-->\n')
    handle.write('      <typedef>\n')
    handle.write('    <%s type="struct">\n' %module_params['STRUCT_NAME'])

    handle.write('			<pwl_vshort_lut_en type="uint16"> </pwl_vshort_lut_en>\n')
    handle.write('			<pwl_vshort_mask type="uint16"> </pwl_vshort_mask>\n')
    handle.write('			<pwl_vshort_shift type="uint16"> </pwl_vshort_shift>\n')
    handle.write('			<pwl_vshort_lut_bitdepth type="uint16"> </pwl_vshort_lut_bitdepth>\n')
    handle.write('			<pwl_vshort_lut_clip type="uint16"> </pwl_vshort_lut_clip>\n')
    handle.write('			<pwl_vshort_lut type="uint16[639]"> </pwl_vshort_lut>\n')

    handle.write('      </%s>\n' %module_params['STRUCT_NAME'])
    handle.write('   </typedef>\n')
    handle.write('  <!--=======================================================================-->\n')

    # Create a DCC Use Case
    for i in range(decmp_params['NUM_USE_CASE']):
        dccxml.AddUseCase(handle, module_params, decmp_params['USE_CASE'][i])

    r_table = '../tables/lut_rawfe_pwl_vshort.txt'
    shutil.copy(r_table, directory)

    dccxml.GenFooter(handle, module_params)
    dccxml.CloseFile(handle)
