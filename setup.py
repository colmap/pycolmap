""" Author: Mihai-Dusmanu (mihaidusmanu) """

import os
import re
import sys
import platform
import subprocess
from pathlib import Path

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.1.0':
                raise RuntimeError("CMake >= 3.1.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = [
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
            '-DPYTHON_EXECUTABLE=' + sys.executable,
            '-DVERSION_INFO={}'.format(self.distribution.get_version()),
        ]
        eigen_dir = os.environ.get('EIGEN3_INCLUDE_DIRS')
        if eigen_dir is not None:
            cmake_args += ['-DEIGEN3_INCLUDE_DIRS={}'.format(eigen_dir)]
        qt5_dir = os.environ.get("Qt5_DIR")
        if qt5_dir is not None:
            cmake_args += ['-DQt5_DIR={}'.format(qt5_dir)]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            if os.environ.get('CMAKE_TOOLCHAIN_FILE') is not None:
                cmake_toolchain_file = os.environ.get('CMAKE_TOOLCHAIN_FILE')
                # print(f'-DCMAKE_TOOLCHAIN_FILE={cmake_toolchain_file}')
                cmake_args += [f'-DCMAKE_TOOLCHAIN_FILE={cmake_toolchain_file}']
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir)]
            if sys.maxsize > 2**32:
                if os.environ.get('CMAKE_TOOLCHAIN_FILE') is not None:
                    cmake_args += ['-DVCPKG_TARGET_TRIPLET=x64-windows']
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get('CXXFLAGS', ''),
            self.distribution.get_version()
        )
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        print(['cmake', ext.sourcedir] + cmake_args)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=self.build_temp)


long_description = (Path(__file__).parent / 'README.md').read_text()

setup(
    name='pycolmap',
    version='0.4.0',
    author='Mihai Dusmanu',
    author_email='mihai.dusmanu@inf.ethz.ch',
    description='COLMAP bindings',
    long_description=long_description,
    long_description_content_type='text/markdown',
    ext_modules=[CMakeExtension('colmap')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)
