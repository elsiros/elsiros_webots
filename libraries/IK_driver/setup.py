from distutils.core import setup, Extension

setup(name='starkit', version='1.0',  \
      ext_modules=[Extension('starkit', ['starkit.c'])])