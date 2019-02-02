from setuptools import setup, find_packages

setup(name='pmmcontrol',
      version='0.1',
      description='Code to control the PMM array with an arduino',
      url='http://github.com/zobristnicholas/pmmcontrol',
      author='Nicholas Zobrist',
      license='GNU GPLv3',
      packages=find_packages(),
      install_requires=["numpy",
                        "pyserial",
                        "scipy"],
      zip_safe=False)
