"""
    J7 VPAC specific configurations and methods.
"""

import os
import inspect


REPORT_CALLER = True

WAIT_ON_EXIT = False
AWB_MAX_IMG_CNT = 20 

def init_params(sys_params):
    # INITIALIZATION START
    sys_params['LSC'] = {}
    sys_params['AWB'] = {}
    sys_params['AE'] = {}
    sys_params['AE']['IMAGE'] = {}
    sys_params['AE']['VIDEO'] = {}
    sys_params['AE']['WIGHTTABLE'] = {}
    for _ in range (0,4):
        sys_params['LSC'][_] = {}
    for _ in range (0,AWB_MAX_IMG_CNT):
        sys_params['AWB'][_] = {}
    sys_params['LUT3D']={}
    sys_params['GAMMA'] = {}
    sys_params['GAMMA']['VIDEO'] = {}
    sys_params['GAMMA']['IMAGE'] = {}
    sys_params['GAMMA']['IMAGE']['OUTDOOR']={}
    sys_params['GAMMA']['IMAGE']['INDOOR']={}
    sys_params['GAMMA']['IMAGE']['INDOOR_DARK']={}
    sys_params['GAMMA']['VIDEO']['OUTDOOR']={}
    sys_params['GAMMA']['VIDEO']['INDOOR']={}
    sys_params['GAMMA']['VIDEO']['INDOOR_DARK']={}
    sys_params['NOISE_PIPE'] ={}
    for _ in range (0,6):
        sys_params['NOISE_PIPE'][_] = {}
    sys_params['LSC']['FILECNT'] =0
    sys_params['AWB']['FILECNT'] =0
    sys_params['INPUT_TYPE'] = '.raw'
    # INITIALIZATION FINISH
    return


def error(message, skip=False, warn=False):
    """
        Print error message and quit if not skip.
        Print warning instead of error if warn.
    """

    if REPORT_CALLER:
        try:
            message = '[%s:%d] %s' %(inspect.stack()[2][3], inspect.stack()[2][2], message,)
        except IndexError as err:
            pass

    if warn:
        message = '[WARNING] %s' %message
    else:
        message = '[ERROR] %s' %message

    print(message)

    if not skip:
        if WAIT_ON_EXIT:
            raw_input("Press ENTER to exit")
        os.sys.exit(1)
def get_params(fname, params):
    """
        Load configuration from file into specified dict.
        Lines beginning with # will be ignored as comments.
        Anything after the KEY VALUE double will be ignored.
        Single or multiple whitespace is exclusive delimiter.
    """
    try:
        config_file = open(fname, 'r')
    except IOError as err:
        error('Params file not found: %s' %err.filename, skip=True)
        return 1
    fileindex = 0
    for line in config_file:
        if line[0] == '#':
            continue
        config = line.split()[:10]
        """
            Config Length 2 for Sensor Parameters
            > 2 for LSC, AWB
        """
        comp_name_idx = line.find("AE")
        if ( comp_name_idx == 0 ):
            if len(config) == 3:
                params['AE'][config[1]] =config[2].strip()
            else:
                usecase = config[1]
                params['AE'][usecase][config[2]] = config[3]
            continue
        comp_name_idx = line.find("LUT3D")
        if ( comp_name_idx == 0 ):
            params['LUT3D']['TGT_REF_IDX'] =config[1].strip()
            continue 
        if len(config) == 2:
            try:
                params[config[0]] = config[1].strip()
            except ValueError:
                params[config[0]] = config[1]
            continue
        # LSC IMG_TYPE TEMP FILENAME %GECORRECT VALID
        comp_name_idx = line.find("LSC")
        if ( comp_name_idx == 0 ):
            val = int(config[5])
            temperature = int(config[2].strip())
            ShdCorrect = int(config[4].strip())
            if val == 1 :
                fileindex = params['LSC']['FILECNT']
                params['LSC'][fileindex]['TEMP'] = temperature
                try:
                    params['LSC'][fileindex]['FILENAME'] = config[3]
                except ValueError:
                    params['LSC'][fileindex]['FILENAME']  = config[3]
                params['LSC'][fileindex]['SHDCORECT'] = ShdCorrect
                params['LSC']['FILECNT'] = params['LSC']['FILECNT'] + 1
            continue
        # # AWB STD/ADL TEMP GAIN EXP FLASH APERTURE FILENAME VALID
        comp_name_idx = line.find("AWB")
        if ( comp_name_idx == 0 ):
            val = int(config[9])
            ImageType = config[1].strip()
            temperature = int(config[3].strip())
            Gain = int(config[4].strip())
            Exposure = int(config[5].strip())
            Flash = int(config[6].strip())
            Aperture = int(config[7].strip())
            if val == 1 :
                fileindex = params['AWB']['FILECNT']
                params['AWB'][fileindex]['IMG_TYP'] = ImageType
                params['AWB'][fileindex]['TEMP'] = temperature
                params['AWB'][fileindex]['GAIN'] = Gain
                params['AWB'][fileindex]['EXP'] = Exposure
                params['AWB'][fileindex]['FLASH'] = Flash
                params['AWB'][fileindex]['APERTURE'] = Aperture
                params['AWB'][fileindex]['FILENAME'] = config[8]
                params['AWB']['FILECNT'] = params['AWB']['FILECNT'] + 1
            continue
        # GAMMA VIDEO/IMAGE OUTDOOR/INDOOR/INDOORDARK GAMMAFILE VALID         
        comp_name_idx = line.find("GAMMA")
        if ( comp_name_idx == 0 ):
            usecase = config[1].strip()
            region = config[2].strip()
            params['GAMMA'][usecase][region]['FILENAME']=config[3].strip()
            continue 
        comp_name_idx = line.find("NOISE_PIPE")
        # NOISE_PIPE OUTDOOR?INDOOR OUTDOOR/HIGH/MID_HIGH/MID/MID_LOW/LOW NOISE SHARPNESS
        RegionList = {'OUTDOOR':0,'HIGH':1,'MID_HIGH':2,'MID':3,'MID_LOW':4,'LOW':5}
        if ( comp_name_idx == 0 ):
            key = str(config[2].strip())
            RegionIdx = RegionList[key]
            params['NOISE_PIPE'][RegionIdx]['NOISE'] = int(config[3].strip())
            params['NOISE_PIPE'][RegionIdx]['SHARPNESS'] = int(config[4].strip())
            continue
    config_file.close()
    return 0
