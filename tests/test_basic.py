from ctypes import CDLL, c_int

# Load the shared library
lib = CDLL('/home/jetbot/iterative_setup/build/libmylib.so')

# Set the return types of the functions
#lib.add.restype = c_int
#lib.subtract.restype = c_int

# Call the functions
lib.april_test()
