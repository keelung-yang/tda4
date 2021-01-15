"""
    DCC XML Generator Functions
    
    CFAI
"""

import os
import dcc
import dccxml
import shutil
   
def write_FIR_Coeff_R(handle):
    handle.write('							  { 0,  1,  4,  3,  1,  0, \n ')
    handle.write('							   -1,  0,  -48,  0,  -19,  0, \n ')
    handle.write('							   4,  18,  123,  45,  51,  1, \n ')
    handle.write('							   1,  0,  45,  0,  18,  0, \n ')
    handle.write('							   1,  -19,  51,  -48,  21,  -1, \n ')
    handle.write('							   0,  0,  3,  0,  1,  0}, \n ')

def write_FIR_Coeff_Gr(handle):
    handle.write('							  { 0,  1,  3,  4,  1,  0, \n ')
    handle.write('							   0,  -19,  0,  -48,  0,  -1, \n ')
    handle.write('							   1,  51,  45,  123,  18,  4, \n ')
    handle.write('							   0,  18,  0,  45,  0,  1, \n ')
    handle.write('							   -1,  21,  -48,  51,  -19,  1, \n ')
    handle.write('							   0,  1,  0,  3,  0,  0}, \n ')

def write_FIR_Coeff_Gb(handle):   
    handle.write('							  { 0,  0,  3,  0,  1,  0, \n ')
    handle.write('							   1,  -19,  51,  -48,  21,  -1, \n ')
    handle.write('							   1,  0,  45,  0,  18,  0, \n ')
    handle.write('							   4,  18,  123,  45,  51,  1, \n ')
    handle.write('							   -1,  0,  -48,  0,  -19,  0, \n ')
    handle.write('							   0,  1,  4,  3,  1,  0}, \n ')
   

def write_FIR_Coeff_B(handle):  
    handle.write('							  { 0,  1,  0,  3,  0,  0, \n ')
    handle.write('							   -1,  21,  -48,  51,  -19,  1, \n ')
    handle.write('							   0,  18,  0,  45,  0,  1, \n ')
    handle.write('							   1,  51,  45,  123,  18,  4, \n ')
    handle.write('							   0,  -19,  0,  -48,  0,  -1, \n ')
    handle.write('							   0,  1,  3,  4,  1,  0} \n ')

def write_FIR_Coeffs(handle, bayer_phase):  

    if(bayer_phase == 0):
        write_FIR_Coeff_R(handle)
        write_FIR_Coeff_Gr(handle)
        write_FIR_Coeff_Gb(handle)
        write_FIR_Coeff_B(handle)
    elif(bayer_phase == 1):
        write_FIR_Coeff_Gr(handle)
        write_FIR_Coeff_R(handle)
        write_FIR_Coeff_B(handle)
        write_FIR_Coeff_Gb(handle)
    elif(bayer_phase == 2):
        write_FIR_Coeff_Gb(handle)
        write_FIR_Coeff_B(handle)
        write_FIR_Coeff_R(handle)
        write_FIR_Coeff_Gr(handle)
    elif(bayer_phase == 3):
        write_FIR_Coeff_B(handle)
        write_FIR_Coeff_Gb(handle)
        write_FIR_Coeff_Gr(handle)
        write_FIR_Coeff_R(handle)
    else:
        handle.write('Unsupported CFA Phase')

 
def write_FIR_Coeff_dummy(handle):  
    handle.write('							  { 0,  0,  0,  0,  0,  0, \n ')
    handle.write('							   0,  0,  0,  0,  0,  0, \n ')
    handle.write('							   0,  0,  0,  0,  0,  0, \n ')
    handle.write('							   0,  0,  0,  0,  0,  0, \n ')
    handle.write('							   0,  0,  0,  0,  0,  0, \n ')
    handle.write('							   0,  0,  0,  0,  0,  0}, \n ')

bayer_phase = 0
    
def GenCFAIParams(handle, cfai_params, cls_id):

    handle.write('              16,   // image bits \n ')
    handle.write('              1,    // lut_enable \n ')

    handle.write('					{175, 95, 95, 175}, //Set0GradHzMask \n ')
    handle.write('					{175, 95, 95, 175}, //Set0GradVtMask \n ')
    handle.write('					{0,1,2,3},//Set0IntensityMask \n ')
    handle.write('					{4,5,6,7}, //Set0IntensityShift \n ')
    handle.write('					{500,600,700,800,900,1000,1100}, //Set0Thr \n ')

    handle.write('					{175, 95, 95, 175}, //Set1GradHzMask \n ')
    handle.write('					{276,196,196,276}, //Set1GradVtMask \n ')
    handle.write('					{8,9,10,11},//Set1IntensityMask \n ')
    handle.write('					{12,13,14,15}, //Set1IntensityShift \n ')
    handle.write('					{0,100,200,300,400,500,600}, //Set1Thr \n ')

    handle.write('					{2,2,2,2},//blendMode \n ')
    handle.write('					{0,0,0,0},//bitMaskSel \n ')
    handle.write('					//FirCoefs \n ')
    handle.write('					{ \n ')
    handle.write('						//FirCoef 0 - Rh \n ')
    handle.write('						{       \n ')
    handle.write('							{ \n ')
    write_FIR_Coeffs(handle, bayer_phase)
    handle.write('							}, \n ')
    handle.write('						},			 \n ')			      
    
    handle.write('						//FirCoef 1 : RV \n ')
    handle.write('						{ \n ')
    handle.write('							{ \n ')
    write_FIR_Coeffs(handle, bayer_phase)
    handle.write('							}, \n ')
    handle.write('						}, \n ')
    
    handle.write('						//FirCoef 2 : RN \n ')
    handle.write('						{ \n ')
    handle.write('							{ \n ')
    write_FIR_Coeffs(handle, bayer_phase)

    handle.write('							}, \n ')
    handle.write('						}, \n ')
    
    handle.write('						//FirCoef 3 : Gh \n ')
    handle.write('						{ \n ')
    handle.write('							{ \n ')
    write_FIR_Coeffs(handle, bayer_phase)
    handle.write('							}, \n ')
    handle.write('						}, \n ')
    
    handle.write('						//FirCoef 4 : GV \n ')
    handle.write('						{ \n ')
    handle.write('							{ \n ')
    write_FIR_Coeffs(handle, bayer_phase)
    handle.write('							}, \n ')
    handle.write('						}, \n ')
    
    handle.write('						//FirCoef 5 : GN \n ')
    handle.write('						{ \n ')
    handle.write('							{ \n ')
    write_FIR_Coeffs(handle, bayer_phase)
    handle.write('							}, \n ')
    handle.write('						}, \n ')

    handle.write('						//FirCoef 6 : BH \n ')
    handle.write('						{ \n ')
    handle.write('							{ \n ')
    write_FIR_Coeffs(handle, bayer_phase)
    handle.write('							}, \n ')
    handle.write('						}, \n ')
    
    handle.write('						//FirCoef 7 : BV \n ')
    handle.write('						{ \n ')
    handle.write('							{ \n ')
    write_FIR_Coeffs(handle, bayer_phase)
    handle.write('							}, \n ')
    handle.write('						}, \n ')
    
    handle.write('						//FirCoef 8 : BN \n ')
    handle.write('						{ \n ')
    handle.write('							{ \n ')
    write_FIR_Coeffs(handle, bayer_phase)
    handle.write('							}, \n ')
    handle.write('						}, \n ')
    
    handle.write('						//FirCoef 9 \n ')
    handle.write('						{ \n ')
    handle.write('							{ \n ')
    write_FIR_Coeffs(handle, bayer_phase)
    handle.write('							}, \n ')
    handle.write('						}, \n ')

    handle.write('						//FirCoef 10 \n ')
    handle.write('						{ \n ')
    handle.write('							{ \n ')
    write_FIR_Coeffs(handle, bayer_phase)
    handle.write('							}, \n ')
    handle.write('						}, \n ')

    handle.write('						//FirCoef 11 \n ')
    handle.write('						{ \n ')
    handle.write('							{ \n ')
    write_FIR_Coeffs(handle, bayer_phase)
    handle.write('							}, \n ')
    handle.write('						}, \n ')    

    handle.write('					},//FirCoefs \n ')
    handle.write('					{ //cfa_lut \n ')
    handle.write('						#include "lut_cfa_16to12_g1o4_global.txt" \n ')
    handle.write('					} \n ')

    return

def GenCFAIXML(directory, filebase, params, cfai_params):

    if (os.path.exists(directory) == False):
        print ('Creating directory:  %s\n' %directory)
        try:
            os.makedirs(directory)
        except OSError as err:
            utils.error('%s: %s' %(err.strerror, err.filename), skip=True)

    filename = os.path.join(directory, '%s_%s.xml' %(params['SENSOR'], filebase));
    print ('Creating XML File:  %s\n' %filename	)

    module_params = {}
    module_params['NAME']   = 'cfg_flxd_cfa'
    module_params['STRUCT_NAME'] = 'cfg_flxd_cfa_vars'
    module_params['DCC_ID'] = dcc.DCC_ID_IPIPE_CFA
    global bayer_phase
    bayer_phase = params['BAYER_PATTERN']
    module_params['FUNC_GENPARAMS'] = GenCFAIParams
    
    handle = dccxml.OpenFile(filename)
    dccxml.GenHeader(handle, params, module_params)
    
    # Create the DCC Structure definition
    handle.write('  <!--=======================================================================-->\n')

    handle.write('            	<typedef>\n')
    handle.write('            		<FLXD_FirCoefs type="struct">\n')
    handle.write('            			<matrix type="uint32[4][36]"></matrix>\n')
    handle.write('            		</FLXD_FirCoefs>\n')
    handle.write('            	</typedef>\n')
    
    handle.write('            	<typedef>\n')
    handle.write('            		<cfg_flxd_cfa_vars type="struct">\n')
    
    handle.write('            			<image_bits type="uint32"> </image_bits>\n')
    handle.write('            			<lut_enable type="uint32"> </lut_enable>\n')
    
    handle.write('            			<Set0GradHzMask type="uint32[4]"> </Set0GradHzMask>\n')
    handle.write('            			<Set0GradVtMask type="uint32[4]"> </Set0GradVtMask>\n')
    handle.write('            			<Set0IntensityMask type="uint32[4]"> </Set0IntensityMask>\n')
    handle.write('            			<Set0IntensityShift type="uint32[4]"> </Set0IntensityShift>\n')
    handle.write('            			<Set0Thr type="uint32[7]"> </Set0Thr>\n')
    
    handle.write('            			<Set1GradHzMask type="uint32[4]"> </Set1GradHzMask>\n')
    handle.write('            			<Set1GradVtMask type="uint32[4]"> </Set1GradVtMask>\n')
    handle.write('            			<Set1IntensityMask type="uint32[4]"> </Set1IntensityMask>\n')
    handle.write('            			<Set1IntensityShift type="uint32[4]"> </Set1IntensityShift>\n')
    handle.write('            			<Set1Thr type="uint32[7]"> </Set1Thr>\n')
    
    handle.write('            			<blendMode type="uint32[4]"> </blendMode>\n')
    handle.write('            			<bitMaskSel type="uint32[4]"> </bitMaskSel>\n')
    
    handle.write('            			<FirCoefs type="FLXD_FirCoefs[12]"> </FirCoefs>\n')
    handle.write('            			<cfa_lut type="uint32[639]"> </cfa_lut>\n')
    
    handle.write('            		</cfg_flxd_cfa_vars>\n')
    handle.write('                </typedef>    \n')
    
    handle.write('  <!--=======================================================================-->\n')
    
    # Create a DCC Use Case
    for i in range(cfai_params['NUM_USE_CASE']):
        dccxml.AddUseCase(handle, module_params, cfai_params['USE_CASE'][i])
    
    dccxml.GenFooter(handle, module_params)
    dccxml.CloseFile(handle)

    shutil.copy2('../tables/lut_cfa_16to12_g1o4_global.txt', directory)
