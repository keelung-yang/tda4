"""
    DCC XML Generator Functions
    
    GLBCE
"""

import os
import dcc
import dccxml
   
def GenGLBCEParams(handle, GLBCE_params, cls_id):

    handle.write('                  255, //strength\n')
    handle.write('                  12,  //intensity_var\n')
    handle.write('                  7,   //space_var\n')
    handle.write('                  64,  //slope_min_lim\n')
    handle.write('                  72,  //slope_max_lim\n')
    handle.write('                  1,\n')
    handle.write('                  {    //fwd_prcpt_lut\n')
    handle.write('                      0, 4622, 8653, 11684, 14195, 16380, 18335, 20118, 21766, 23304,\n')
    handle.write('                      24751, 26119, 27422, 28665, 29857, 31003, 32108, 33176, 34209, 35211,\n')
    handle.write('                      36185, 37132, 38055, 38955, 39834, 40693, 41533, 42355, 43161, 43951,\n')
    handle.write('                      44727, 45488, 46236, 46971, 47694, 48405, 49106, 49795, 50475, 51145,\n')
    handle.write('                      51805, 52456, 53099, 53733, 54360, 54978, 55589, 56193, 56789, 57379,\n')
    handle.write('                      57963, 58539, 59110, 59675, 60234, 60787, 61335, 61877, 62414, 62946,\n')
    handle.write('                      63473, 63996, 64513, 65026, 65535,\n')
    
    handle.write('                  },\n')
    handle.write('                  1,\n')
    handle.write('                  {    //rev_prcpt_lut\n')
    handle.write('                      0, 228, 455, 683, 910, 1138, 1369, 1628, 1912, 2221,\n')
    handle.write('                      2556, 2916, 3304, 3717, 4158, 4626, 5122, 5645, 6197, 6777,\n')
    handle.write('                      7386, 8024, 8691, 9387, 10113, 10869, 11654, 12471, 13317, 14194,\n')
    handle.write('                      15103, 16042, 17012, 18014, 19048, 20113, 21210, 22340, 23501, 24696,\n')
    handle.write('                      25922, 27182, 28475, 29800, 31159, 32552, 33977, 35437, 36930, 38458,\n')
    handle.write('                      40019, 41615, 43245, 44910, 46609, 48343, 50112, 51916, 53755, 55630,\n')
    handle.write('                      57539, 59485, 61466, 63482, 65535\n')
    handle.write('                  },\n')
    handle.write('                  {    //asym_lut\n')
    handle.write('                          0,  12173,  20997,  27687,  32934,\n')
    handle.write('                      37159,  40634,  43543,  46014,  48138,\n')
    handle.write('                      49984,  51603,  53035,  54310,  55453,\n')
    handle.write('                      56483,  57416,  58265,  59041,  59753,\n')
    handle.write('                      60409,  61015,  61577,  62099,  62585,\n')
    handle.write('                      63039,  63464,  63863,  64237,  64590,\n')
    handle.write('                      64923,  65237,  65535,\n')
    handle.write('                  },\n')

def GenGLBCEXML(directory, filebase, params, GLBCE_params):

    if (os.path.exists(directory) == False):
        print( 'Creating directory:  %s\n' %directory)
        try:
            os.makedirs(directory)
        except OSError as err:
            utils.error('%s: %s' %(err.strerror, err.filename), skip=True)

    filename = os.path.join(directory, '%s_%s.xml' %(params['SENSOR'], filebase));
    print ('Creating XML File:  %s\n' %filename	)

    module_params = {}
    module_params['NAME']   = 'viss_glbce'
    module_params['STRUCT_NAME'] = 'viss_glbce'
    module_params['DCC_ID'] = dcc.DCC_ID_ISS_GLBCE
    module_params['FUNC_GENPARAMS'] = GenGLBCEParams
    
    handle = dccxml.OpenFile(filename)
    dccxml.GenHeader(handle, params, module_params)
    
    # Create the DCC Structure definition
    handle.write('  <!--=======================================================================-->\n')
    handle.write('      <typedef>\n')
    handle.write('      <%s type="struct"> \n' %module_params['STRUCT_NAME'])

    handle.write('                        <strength      type="uint32"> </strength>\n')
    handle.write('                        <intensity_var type="uint32"> </intensity_var>     <!-- U4 -->\n')
    handle.write('                        <space_var     type="uint32"> </space_var>         <!-- U4 -->\n')
    handle.write('                        <slope_min_lim type="uint32">  </slope_min_lim>     <!-- U8 -->\n')
    handle.write('                        <slope_max_lim type="uint32">  </slope_max_lim>     <!-- U8 -->\n')
    handle.write('                        <fwd_prcpt_en  type="uint32">  </fwd_prcpt_en>\n')
    handle.write('                        <fwd_prcpt_lut type="uint32[65]"> </fwd_prcpt_lut>\n')
    handle.write('                        <rev_prcpt_en  type="uint32">  </rev_prcpt_en>\n')
    handle.write('                        <rev_prcpt_lut type="uint32[65]"> </rev_prcpt_lut>\n')
    handle.write('                        <asym_lut      type="uint32[33]"> </asym_lut>      <!-- U16 -->\n')

    handle.write('      </%s> \n' %module_params['STRUCT_NAME'])
    
    handle.write('   </typedef>\n')
    handle.write('  <!--=======================================================================-->\n')
    
    # Create a DCC Use Case
    for i in range(GLBCE_params['NUM_USE_CASE']):
        dccxml.AddUseCase(handle, module_params, GLBCE_params['USE_CASE'][i])
    
    dccxml.GenFooter(handle, module_params)
    dccxml.CloseFile(handle)
