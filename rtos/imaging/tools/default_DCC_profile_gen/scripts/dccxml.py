"""
    DCC XML Generator Generic Functions
"""

import os
import utils
import dcc

def OpenFile(filename):
    try:
        handle = open(filename, 'w')
    except(IOError, err):
        utils.error('Could not create XML file: %s' %err.filename, skip=True)
        return 1

    return handle
    

def GenHeader(handle, params, module_params):
    handle.write('<?xml version="1.0" encoding="utf-8"?>\n')
    handle.write('<%s xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xsd="http://www.w3.org/2001/XMLSchema">\n'
                  % (module_params['NAME'].upper()))
    handle.write('  <!--this is comment-->\n')             
    handle.write('  <dcc_name>%s</dcc_name>\n'
                  % module_params['NAME'])
    handle.write('  <dcc_header>\n')
    handle.write('    <camera_module_id>%8d</camera_module_id>\n' 
                  % params['CID'])
    handle.write('    <dcc_descriptor_id>%4d</dcc_descriptor_id>\n'
                  % module_params['DCC_ID'])
    handle.write('    <algorithm_vendor_id>%4d</algorithm_vendor_id>\n'
                  % params['VENDOR_ID'])
    handle.write('    <tunning_tool_version>%8d</tunning_tool_version>\n'
                  % params['VERSION'])
    handle.write('  </dcc_header>\n')

def GenBinary(directory, filebase, params, sys_params):
    filename_xml = os.path.join(directory, '%s_%s.xml' %(params['SENSOR'], filebase));
    filename_bin = os.path.join(directory, '%s%s_%s_%s.bin' %('CID', params['CID'], params['SENSOR'], filebase));
    print('Creating BIN File:  %s\n' %filename_bin)
    cmdline = '..\dcc_gen_win.exe %s' %filename_xml
    os.system(cmdline)
    BIN_DIR = sys_params['PRJ_DIR'] + '\\dcc_binaries\\'
    cmdline = 'copy /y %s %s' %(filename_bin, BIN_DIR)
    #print('EXECUTING:  %s\n' %cmdline)
    os.system(cmdline)


def GenFooter(handle, module_params):
    handle.write('</%s>\n' % module_params['NAME'].upper())

def AddUseCase(handle, module_params, usecase_params):
    regions = usecase_params['RGN_DEFN']

    handle.write('\n')
    handle.write('  <use_case val="%d">\n' % usecase_params['ID'])
    
    # Add the Region space definition
    handle.write('    <n-space>\n')
    for i in range(regions['NUM_REGIONS']):
        handle.write('      <region%d class="%d">\n' %(i, i))
        for j in range(regions['NUM_VARS']):
            if regions['VAR_ID'][j] == 0:
                var_str = 'gain'
            elif regions['VAR_ID'][j] == 1:
                var_str = 'exposure'
            elif regions['VAR_ID'][j] == 2:
                var_str = 'color_temperature'
            var_id = int(regions['VAR_ID'][j])
        
            handle.write('        <%s val="%d" min="%d" max="%d">  </%s>\n' 
                         %(var_str, 
                           regions['VAR_ID'][j], 
                           regions['RGN_LIMITS'][i][var_id][0],
                           regions['RGN_LIMITS'][i][var_id][1],
                           var_str))
                           
        handle.write('      </region%d>\n' %i)
        
    handle.write('    </n-space>\n')
    
    # Write out each parameter package
    for i in range(regions['NUM_REGIONS']):
        handle.write('    <parameter_package>\n')
        handle.write('    <!-- Parameters for photospace class: %d -->\n' % i)
        handle.write('        <%s_dcc type="%s">\n' %(module_params['NAME'], module_params['STRUCT_NAME']))
        handle.write('        {\n')
        
        module_params['FUNC_GENPARAMS'](handle, usecase_params, i)
        
        handle.write('        }\n')
        handle.write('        </%s_dcc>\n' %module_params['NAME'])
        handle.write('    </parameter_package>\n')
        
    handle.write('  </use_case>\n')
    
def CloseFile(handle):
    handle.close()

