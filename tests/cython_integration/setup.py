import sys
import os
import shutil

from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

# clean previous build
for root, dirs, files in os.walk(".", topdown=False):
    for name in files:
        if (name.startswith("cython_module") and not(name.endswith(".pyx") or name.endswith(".pxd"))):
            os.remove(os.path.join(root, name))
    for name in dirs:
        if (name == "build"):
            shutil.rmtree(name)

# build "myext.so" python extension to be added to "PYTHONPATH" afterwards...
setup(
    cmdclass = {'build_module': build_ext},
    ext_modules = [
        Extension("myext", 
                  sources=["cython_module.pyx",
                           "SomeAdditionalCppClass1.cpp",
                           "SomeAdditionalCppClass2.cpp"
                       ],
                  libraries=["delivery_exe"],          # refers to "libdelivery_exe.so"
                  language="c++",                   # remove this if C and not C++
                  extra_compile_args=["-fopenmp", "-O3"],
                  extra_link_args=["-DSOME_DEFINE_OPT", 
                                   "-L./some/extra/dependency/dir/"]
             )
        ]
)   
