from setuptools import setup, find_packages
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.rst'), encoding='utf-8') as f:
    long_description = f.read()

setup(name="py_pursuit_pathing",
      version="0.0.2",
      description='Path following using pure pursuit',
      long_description=long_description,
      long_description_content_type='text/x-rst',
      url='https://github.com/nickschatz/purepursuit',
      author='Nick Schatz',
      author_email='nick@nickschatz.com',
      license="MIT",
      classifiers=[
          # How mature is this project? Common values are
          #   3 - Alpha
          #   4 - Beta
          #   5 - Production/Stable
          'Development Status :: 3 - Alpha',

          # Indicate who your project is intended for
          'Intended Audience :: Developers',
          'Topic :: Software Development :: Libraries',

          # Pick your license as you wish (should match "license" above)
          'License :: OSI Approved :: MIT License',

          # Specify the Python versions you support here. In particular, ensure
          # that you indicate whether you support Python 2, Python 3 or both.
          'Programming Language :: Python :: 3.6',
      ],
      python_requires='>=3.6',
      packages=find_packages(exclude=['contrib', 'docs', 'tests']),
      install_requires=['numpy'],
      package_data={
          'py_pursuit_pathing': ['2018-field.gif'],
      },
      )
