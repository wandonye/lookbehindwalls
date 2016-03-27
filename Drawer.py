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

def drawConfig(confDict):

    im = Image.new('L', (400, 400))
    draw = ImageDraw.Draw(im)
    walls = confDict["walls"]

    for level in walls:
        for wall in walls[level]:
            draw.line((walls[level][wall]["start"]["x"],walls[level][wall]["start"]["y"],
                   walls[level][wall]["end"]["x"],walls[level][wall]["end"]["y"]), fill=255)

    return im

def findEllipsePts(xCenter, yCenter, xRadius, yRadius, phi):
    N = 360

    ellipse = defaultdict(dict)
    for i in range(N):
        theta = 2*pi*i/N
        x0 = xRadius * cos(theta)
        y0 = yRadius * sin(theta)
        x = round(x0*cos(phi) - y0*sin(phi)+xCenter)
        y = round(x0*sin(phi) + y0*cos(phi)+ yCenter)
        if x not in ellipse or y not in ellipse[x]:
            ellipse[x][y] = 1

    ellipsePts = []
    for x in ellipse:
        for y in ellipse[x]:
            ellipsePts.append((x,y))

    return ellipsePts

def drawEllipse(im, xCenter, yCenter, xRadius, yRadius, phi):

    draw = ImageDraw.Draw(im)
    ellipsePts = findEllipsePts(xCenter, yCenter, xRadius, yRadius, phi)

    draw.point(ellipsePts,fill=255)
    return im

def drawPath(im,path):
    draw = ImageDraw.Draw(im)
    for i in range(len(path)-1):
        draw.line((path[i][0],path[i][1],path[i+1][0],path[i+1][1]),fill=255)
    return im

#def draw
