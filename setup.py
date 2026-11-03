"""向 Catkin 注册 src 下的独立 Python 分析模块。"""
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    py_modules=['autotune_analysis'],
    # 空键供 setuptools 定位模块，模块名键供 Catkin 生成 devel 转发入口。
    package_dir={'': 'src', 'autotune_analysis': 'src'},
))
