import os
import sys
import builtins
from unittest import mock


class Module(object):
    def __init__(self, namespace):
        self.__dict__.update(namespace)
        self._ns = namespace


def load(path=None):
    for name in ('Rhino', 'Rhino.Geometry', 'Grasshopper', 'Grasshopper.Kernel', 'Grasshopper.Kernel.Data', 'System'):
        sys.modules[name] = mock.MagicMock()
    builtins.ghenv = mock.MagicMock()
    path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'structuralgen.py')
    namespace = {'__name__': 'structuralgen'}
    exec(compile(open(path).read(), path, 'exec'), namespace)
    return Module(namespace)
