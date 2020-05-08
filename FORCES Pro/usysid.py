import ctypes
# do not delete this comment!
import os
import numpy as np
import numpy.ctypeslib as npct
import sys

def getSysId(input = ''):

    # determine library name
    if sys.platform.startswith('win'):
        if sys.maxsize > 2**32:
            libname = 'usysidw64.dll'
        else:
            libname = 'usysidw32.dll'
    elif sys.platform == "darwin":
        libname = 'libusysidm64.so'
    elif sys.platform.startswith('linux'):
        if sys.maxsize > 2**32:
            libname = 'libusysidl64.so'
        else:
            libname = 'libusysidl32.so'
    else:
        raise Exception("Unknown platform")
	

    # function to call
    try:
        _lib = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)),'lib',libname))
        cfunc = getattr(_lib,'get_fingerprinters_number___FORCESsolver__')
        cfunc_main = getattr(_lib,'compute_system_unique_id_all___FORCESsolver__')
    except:
        _lib = ctypes.CDLL(os.path.join(os.path.dirname(os.path.abspath(__file__)),libname))
        cfunc = getattr(_lib,'get_fingerprinters_number___FORCESsolver__')
        cfunc_main = getattr(_lib,'compute_system_unique_id_all___FORCESsolver__')

    cfunc.restype = ctypes.c_int
    fingerprinters_number = ctypes.c_int()
    fingerprinters_number = _lib.get_fingerprinters_number___FORCESsolver__()

    # determine data types for solver function prototype 
    intarrayType = ctypes.c_ushort * 5
    fingerprinterType = ctypes.POINTER(intarrayType) * fingerprinters_number
    cfunc_main.restype = ctypes.POINTER(fingerprinterType)

    UID = fingerprinterType()
    P_UID = ctypes.pointer(UID)
	
    P_UID = _lib.compute_system_unique_id_all___FORCESsolver__()
	
    if (not input == "all"):
        fingerprinters_number = 1
    
    data = []
    for i in range(0, fingerprinters_number):
        data.append(npct.as_array(P_UID.contents[i].contents))
    
    if(len(data) == 1):
        data = data[0]
    sysid = np.array(data)

    return sysid

def usysid2string(usysid):
    fingerprint_str = ''
    for i in range(0, 5):
        fingerprint_str += "{:04x}".format(usysid[i]) + "-"
    fingerprint_str = fingerprint_str.rstrip("-")
    return fingerprint_str

if __name__ == '__main__':
    if(len(sys.argv) < 2):
        input = ''
    else:
        input = sys.argv[1]
    returned_sysid = getSysId(input)
    if(returned_sysid.ndim == 1):
        print("System Fingerprint: " + usysid2string(returned_sysid))
    else:
        print("System Fingerprints")
        print("========================")
        for i in range(0, returned_sysid.shape[0]):
            print(usysid2string(returned_sysid[i]))
    sys.exit(0)

