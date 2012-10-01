#!/usr/bin/env python
from distutils.core import setup
import sys
from xml.etree.ElementTree import ElementTree

try:
    root = ElementTree(None, 'package.xml')
    version = root.findtext('version')
except Exception, e:
    print >>sys.stderr, 'Could not extract version from your package.xml:\n%s' % e
    sys.exit(-1)

setup(name='smclib',
      version=version,
      description='Python module of the state machien compiler',
      packages=['smclib'],
      package_dir={'':'python'}
)
