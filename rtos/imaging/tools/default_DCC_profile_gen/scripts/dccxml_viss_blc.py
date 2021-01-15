"""
    DCC XML Generator Functions
    
    VISS Black Level Correction
"""

import os
import dcc
import dccxml
   
def GenVISS_BLCParams(handle, VISS_BLC_params, cls_id):

    handle.write('   				 //viss_clamp_vshort : For linear mode only this should be set to sensor Black Level\n')
    handle.write('   				  {\n')
    handle.write('   						0, // dcoffset_clamp_value 0\n')
    handle.write('   						0, // dcoffset_clamp_value 1\n')
    handle.write('   						0, // dcoffset_clamp_value 2\n')
    handle.write('   						0, // dcoffset_clamp_value 3\n')
    handle.write('   				  },\n')
    handle.write('   				 //viss_clamp_short : To be used only in WDR mode. \n')
    handle.write('   				  {\n')
    handle.write('   						0, // dcoffset_clamp_value 0\n')
    handle.write('   						0, // dcoffset_clamp_value 1\n')
    handle.write('   						0, // dcoffset_clamp_value 2\n')
    handle.write('   						0, // dcoffset_clamp_value 3\n')
    handle.write('   				  },\n')
    handle.write('   				 //viss_clamp_long : To be used only in WDR mode.\n')
    handle.write('   				  {\n')
    handle.write('   						0, // dcoffset_clamp_value 0\n')
    handle.write('   						0, // dcoffset_clamp_value 1\n')
    handle.write('   						0, // dcoffset_clamp_value 2\n')
    handle.write('   						0, // dcoffset_clamp_value 3\n')
    handle.write('   				  }\n')

def GenVISS_BLCXML(directory, filebase, params, VISS_BLC_params):

    if (os.path.exists(directory) == False):
        print ('Creating directory:  %s\n' %directory)
        try:
            os.makedirs(directory)
        except OSError as err:
            utils.error('%s: %s' %(err.strerror, err.filename), skip=True)

    filename = os.path.join(directory, '%s_%s.xml' %(params['SENSOR'], filebase));
    print ('Creating XML File:  %s\n' %filename	)

    module_params = {}
    module_params['NAME']   = 'VISS_BLC'
    module_params['STRUCT_NAME'] = 'viss_clamp'
    module_params['DCC_ID'] = dcc.DCC_ID_ISIF_BLACK_CLAMP
    module_params['FUNC_GENPARAMS'] = GenVISS_BLCParams
    
    handle = dccxml.OpenFile(filename)
    dccxml.GenHeader(handle, params, module_params)
    
    # Create the DCC Structure definition
    handle.write('  <!--=======================================================================-->\n')
    handle.write('      <typedef>\n')
    handle.write('      <%s type="struct"> \n' %module_params['STRUCT_NAME'])

    handle.write('       			<viss_clamp_vshort type="struct">\n')
    handle.write('       				<dcoffset_clamp_value_0 type="int16"> </dcoffset_clamp_value_0> <!-- additive dc offset, S13 -->\n')
    handle.write('       				<dcoffset_clamp_value_1 type="int16"> </dcoffset_clamp_value_1> <!-- additive dc offset, S13 -->\n')
    handle.write('       				<dcoffset_clamp_value_2 type="int16"> </dcoffset_clamp_value_2> <!-- additive dc offset, S13 -->\n')
    handle.write('       				<dcoffset_clamp_value_3 type="int16"> </dcoffset_clamp_value_3> <!-- additive dc offset, S13 -->\n')
    handle.write('       			</viss_clamp_vshort>\n')

    handle.write('       			<viss_clamp_short type="struct">\n')
    handle.write('       				<dcoffset_clamp_value_0 type="int16"> </dcoffset_clamp_value_0> <!-- additive dc offset, S13 -->\n')
    handle.write('       				<dcoffset_clamp_value_1 type="int16"> </dcoffset_clamp_value_1> <!-- additive dc offset, S13 -->\n')
    handle.write('       				<dcoffset_clamp_value_2 type="int16"> </dcoffset_clamp_value_2> <!-- additive dc offset, S13 -->\n')
    handle.write('       				<dcoffset_clamp_value_3 type="int16"> </dcoffset_clamp_value_3> <!-- additive dc offset, S13 -->\n')
    handle.write('       			</viss_clamp_short>\n')
    
    handle.write('       			<viss_clamp_long type="struct">\n')
    handle.write('       				<dcoffset_clamp_value_0 type="int16"> </dcoffset_clamp_value_0> <!-- additive dc offset, S13 -->\n')
    handle.write('       				<dcoffset_clamp_value_1 type="int16"> </dcoffset_clamp_value_1> <!-- additive dc offset, S13 -->\n')
    handle.write('       				<dcoffset_clamp_value_2 type="int16"> </dcoffset_clamp_value_2> <!-- additive dc offset, S13 -->\n')
    handle.write('       				<dcoffset_clamp_value_3 type="int16"> </dcoffset_clamp_value_3> <!-- additive dc offset, S13 -->\n')
    handle.write('       			</viss_clamp_long>  \n')
  
    handle.write('      </%s> \n' %module_params['STRUCT_NAME'])
  
    handle.write('   </typedef>\n')
    handle.write('  <!--=======================================================================-->\n')
    
    # Create a DCC Use Case
    for i in range(VISS_BLC_params['NUM_USE_CASE']):
        dccxml.AddUseCase(handle, module_params, VISS_BLC_params['USE_CASE'][i])
    
    dccxml.GenFooter(handle, module_params)
    dccxml.CloseFile(handle)
