from setuptools import find_packages, setup, Extension
from Cython.Build import cythonize

ext_modules = [
    Extension(
        "nest_graph.elem_graph",
        sources=["nest_graph/elem_graph.pyx"],
        language="c++",
        extra_compile_args=['-std=c++17', '-Ofast'],
    )
]

setup(
    name="nest_graph",
    packages=find_packages(include=["nest_graph"]),
    ext_modules=cythonize(
        ext_modules,
        compiler_directives=dict(
            language_level=3,
        )
    ),
)
