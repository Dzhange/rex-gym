#!/home/jianrenw/anaconda3/envs/rex/bin/python
# -*- coding: utf-8 -*-
import re
import sys
from rex_gym.cli.entry_point import cli
if __name__ == '__main__':    
    sys.argv[0] = re.sub(r'(-script\.pyw|\.exe)?$', '', sys.argv[0])    
    sys.exit(cli())
