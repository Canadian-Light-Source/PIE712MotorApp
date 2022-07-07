import sys
import os
import configparser
from e712_pam_exclude_addrs import exclude_parm_addrs

TOP = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

def load_param_file(fname):
    '''
    This function is meant to be called directly from the E712 driver file load_param_file.cpp
    that function creates an instance of the python interpreter and then calls this function to process the
    .pam file and return a single string of E712 commands seperated by ; where they are then sent one at a time
    by the driver itself to the E712 controller, 
    :param fname: 
    :return: 
    '''

    #path = r'C:\controls\epics\R3.14.12.4\modules\support\motor-6-8\paramFiles'
    path = os.path.join(TOP, 'paramFiles')
    full_path = os.path.join(path, fname)
    print('load_it: using [%s]' % full_path)

    if(not os.path.isfile(full_path)):
        print('file: [%s] does not exist' % full_path)
        return

    cmnd_dct = {}
    addrs_dct = {}
    Config = configparser.RawConfigParser()
    Config.optionxform = str
    Config.read(full_path)
    for section in Config.sections():
        for option in Config.options(section):
            if ((section.find('PAM FORMAT') > -1) or (section.find('DEVICE') > -1) or (section.find('PAM_CONTENT') > -1) ):
                pass
            else:
                if (option.find('_A') > -1):
                    #print 'the following has an address'
                    parm_addr, axis_chan = option.split('_A')
                    val = Config.get(section, option)
                    val = val.replace('\"','')
                    if(parm_addr not in exclude_parm_addrs):
                        #print('[%s] has option [%s] with a value of [%s]: parm_addr=%s axis_chan=%s' % (section, option, Config.get(section, option), parm_addr, axis_chan))
                        cmnd_dct['%s %s' % (axis_chan, parm_addr)] = {'cmd': 'SPA %s 0x%s %s' % (axis_chan, parm_addr, val)}
                        addrs_dct[parm_addr] = parm_addr
                    else:
                        #print('EXCLUDING: %s %s' % (axis_chan, parm_addr))
                        pass

                else:
                    #print('[%s] has option [%s] with a value of [%s]' % (section, option, Config.get(section, option)))
                    pass

    s_keys = sorted(cmnd_dct.keys())
    #a_keys = sorted(addrs_dct.keys())
    lst = ['CCL 1 advanced']
    for addr in s_keys:
        lst.append(cmnd_dct[addr]['cmd'])

    cmnd_string = ";".join(lst) + ';'
    return(cmnd_string)


if __name__ == "__main__":

    fname = sys.argv[1:]
    fname = 'test_param_file.dat'
    load_param_file(fname)