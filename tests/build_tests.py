import ctypes

# Load the shared library
lib = ctypes.CDLL('/home/jetbot/iterative_setup/build/libmylib.so')

# Set the return types of the functions
#lib.add.restype = ctypes.c_int
#lib.subtract.restype = ctypes.c_int

# Call the functions
lib.testApriltag()

directory = "../docs/"

# Define the function signature
lib.testOpencv.argtypes = [ctypes.c_char_p]
lib.testOpencv.restype = None

# Call the C++ function with a string
lib.testOpencv(directory)
