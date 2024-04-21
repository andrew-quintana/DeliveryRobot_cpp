# cython_module.pyx
from libcpp.string cimport string
from libcpp.memory cimport unique_ptr

cdef extern from "DeliveryFSM.h":
    cdef cppclass DeliveryFSM:
        @staticmethod
        DeliveryFSM* create()
        void test_functionality(string)

cdef class PyDeliveryFSM:
    cdef unique_ptr[DeliveryFSM] _thisptr

    def __cinit__(self):
        self._thisptr.reset(DeliveryFSM.create())

    def __dealloc__(self):
        # Deallocate the C++ object
        self._thisptr.reset()

    def test_functionality(self, img_path: str):
        self._thisptr.get().test_functionality(img_path.encode())
