import numpy as np
from setuptools import setup, Extension
from Cython.Build import cythonize

module_name = 'vec_test'

extensions = [
    Extension(
        module_name,
        sources=[f'bindings.pyx'],
        language='c++',
        extra_compile_args=['-std=c++17', '-Ofast'],
        include_dirs=[np.get_include()],
        define_macros=[('NPY_NO_DEPRECATED_API', 'NPY_1_7_API_VERSION')]
    )
]

setup(
    name=module_name,
    ext_modules=cythonize(
        extensions,
        compiler_directives=dict(
            language_level=3,
        )
    )
)
