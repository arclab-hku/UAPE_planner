#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jul  2 18:21:05 2021

@author: chen
"""

from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension
from Cython.Distutils import build_ext
import numpy

extension = Extension(
            "FET_utils_c",
            sources=["FET_utils_c.pyx"],  #linesafe.pyx  ellipse.pyx FET_utils_c
            include_dirs=[numpy.get_include()], # 如果用到numpy
            language="c"
)

setup(
        cmdclass = {'build_ext': build_ext},
        ext_modules = cythonize(extension),
)
# python setup.py build_ext --inplace


# setup(
#     name="ellipse_test",
#     ext_modules=cythonize("ellipse_test.pyx", include_path=[numpy.get_include()]),
# )