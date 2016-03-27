# -*- coding: utf-8 -*-
"""
Dongning Wang
"""
import json
from PIL import Image, ImageDraw
import numpy as np
from math import pi, cos, sin, sqrt
from collections import defaultdict
import matplotlib.pyplot as plt
from Drawer import drawConfig, drawEllipse

def loadConfig(fileName):
    file = open(fileName)
    conf = json.load(file)
    return conf

def intersectionOfLineSegments(b1,e1,b2,e2):
    #To find out if a line segment intersect a line or not,
    #check if the two end points of the line segment fall in
    #different sides of the line.
    #To determine if two line segments intersects
    v1 = e1-b1
    w1 = e2-b1




def firstObjOnRay(config,origin,to):
    #to is represented by a vector
    closest = float('inf')
    collideLevel = None
    firstWall = None
    firstIntersection = None
    for level in config["walls"]:
        for wall in config["walls"][level]:
            w1_x = config["walls"][level][wall]["start"]["x"]-origin[0]
            w1_y = config["walls"][level][wall]["start"]["y"]-origin[1]
            w2_x = config["walls"][level][wall]["end"]["x"]-origin[0]
            w2_y = config["walls"][level][wall]["end"]["y"]-origin[1]
            v_x = config["walls"][level][wall]["end"]["x"]-config["walls"][level][wall]["start"]["x"]
            v_y = config["walls"][level][wall]["end"]["y"]-config["walls"][level][wall]["start"]["y"]

            if (to[0]*w1_y-to[1]*w1_x)*(to[0]*w2_y-to[1]*w2_x)<0:
                #solve for intersection
                #remember to check range(it may on the wrong side of the ray)
                #equation of the two lines:
                #v_y*x-v_x*y=C1, to[1]*x-to[0]*y=C2
                D = np.matrix([[v_y,-v_x],[to[1],to[0]]])
                c = np.matrix([v_y*config["walls"][level][wall]["start"]["x"]-v_x*config["walls"][level][wall]["start"]["y"],
                                to[1]*origin[0]-to[0]*origin[1]]).getT()
                interPt = D.getI()*c
                if (interPt[0]-origin[0])*to[0]+(interPt[1]-origin[1])*to[1]>0:
                    dist = sqrt(pow(interPt[0]-origin[0],2)+pow(interPt[1]-origin[1],2))
                    if dist<closest:
                        closest = dist
                        collideLevel = level
                        firstWall = wall
                        firstIntersection = interPt

    return [collideLevel,firstWall,firstIntersection]


config = loadConfig("config.json")
configIm = drawConfig(config)
configIm.save("config.png")

print(firstObjOnRay(config,(config["cam"]["x"],config["cam"]["y"]),(0,10)))
