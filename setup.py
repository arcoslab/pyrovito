#!/usr/bin/env python

from distutils.core import setup


setup(name='pyrovito',
      version='0.1',
      description='Python Robot Visualization Tool',
      author='Federico Ruiz Ugalde',
      author_email='memeruiz@gmail.com',
      url='http://www.arcoslab.org/',
      package_dir={'pyrovito': ''},
      packages=['pyrovito'],
      scripts=['pyrovito', 'vis_sink.py'])
