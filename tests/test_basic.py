from ctypes import CDLL, c_int

# Load the shared library
lib = CDLL('/home/jetbot/pybind11_test/build/libmylib.so')

# Set the return types of the functions
lib.add.restype = c_int
lib.subtract.restype = c_int

# Call the functions
result_add = lib.add(5, 3)
result_subtract = lib.subtract(10, 4)

print(f"Addition result: {result_add}")
print(f"Subtraction result: {result_subtract}")
