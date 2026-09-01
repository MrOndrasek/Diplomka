import argparse
import importlib
import math
import os
import sys
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence, Type
from iiwaPy3 import iiwaPy3

DEFAULT_IP = "172.31.1.147"
DEFAULT_PORT = 30001

try:
    iiwa=iiwaPy3(DEFAULT_IP)
    print("connected")
except:
    print("error")