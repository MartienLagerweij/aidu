#!/usr/bin/env python
__author__ = 'Rolf Jagerman'

import roslib; roslib.load_manifest('aidu_gui')
import sys
from manager import Manager


def main():
    """
    Creates the GUI and runs a GUI event thread until it is shut down.
    """
    Manager.setup()
    exit_code = Manager.execute()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()

