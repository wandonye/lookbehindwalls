# -*- coding: utf-8 -*-
"""
Dongning Wang
"""
import json
from PIL import Image, ImageDraw
import numpy as np
from math import pi, cos, sin
from collections import defaultdict
import matplotlib.pyplot as plt

def loadConfig(fileName):
    file = open(fileName)
    conf = json.load(file)
    return conf

def shootRay()
