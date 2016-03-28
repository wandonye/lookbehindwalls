# -*- coding: utf-8 -*-
"""
Dongning Wang
"""
import json
from PIL import Image, ImageDraw
import numpy as np
from random import randint
from collections import defaultdict
from math import pi, cos, sin, sqrt, atan2, copysign
from Drawer import drawConfig, drawEllipse, drawPath

def loadConfig(fileName):
    file = open(fileName)
    conf = json.load(file)
    for level in conf["walls"]:
        for wall_id in conf["walls"][level]:
            conf["walls"][level][wall_id]["id"] = wall_id

    return conf

def firstObjOnRay(config,origin,to, ambient_wall = None):
    #to is represented by a vector
    closest = float('inf')
    collideLevel = None
    firstWall = None
    firstIntersection = None
    for level in config["walls"]:
        wall_in_level = dict(config["walls"][level])
        if ambient_wall in wall_in_level: del wall_in_level[ambient_wall]
        for wall in wall_in_level:

            w1_x = config["walls"][level][wall]["start"]["x"]-origin[0]
            w1_y = config["walls"][level][wall]["start"]["y"]-origin[1]
            w2_x = config["walls"][level][wall]["end"]["x"]-origin[0]
            w2_y = config["walls"][level][wall]["end"]["y"]-origin[1]
            v_x = config["walls"][level][wall]["end"]["x"]-config["walls"][level][wall]["start"]["x"]
            v_y = config["walls"][level][wall]["end"]["y"]-config["walls"][level][wall]["start"]["y"]
            #To find out if a line segment intersect a line or not,
            #check if the two end points of the line segment fall in
            #different sides of the line.
            if (to[0]*w1_y-to[1]*w1_x)*(to[0]*w2_y-to[1]*w2_x)<0:
                #solve for intersection
                #remember to check range(it may on the wrong side of the ray)
                #equation of the two lines:
                #v_y*x-v_x*y=C1, to[1]*x-to[0]*y=C2
                D = np.matrix([[v_y,-v_x],[to[1],-to[0]]])
                c = np.matrix([v_y*config["walls"][level][wall]["start"]["x"]-v_x*config["walls"][level][wall]["start"]["y"],
                                to[1]*origin[0]-to[0]*origin[1]]).getT()
                interPt = (D.getI()*c).getA1()

                if (interPt[0]-origin[0])*to[0]+(interPt[1]-origin[1])*to[1]>0:

                    dist = sqrt(pow(interPt[0]-origin[0],2)+pow(interPt[1]-origin[1],2))
                    if dist<closest:
                        closest = dist
                        collideLevel = level
                        firstWall = wall
                        firstIntersection = interPt

    if firstWall:
        return (collideLevel,firstWall,firstIntersection,closest)
    else:
        return (None,None,None,None)


def scatteredTo(config, src_pt, pt, wall):
    res = defaultdict(lambda : defaultdict(list))
    vector_wall = (wall["end"]["x"]-wall["start"]["x"],wall["end"]["y"]-wall["start"]["y"])
    tilt = atan2(vector_wall[1], vector_wall[0])
    vector_wall_to_src = (src_pt[0]-wall["start"]["x"],src_pt[1]-wall["start"]["y"])
    if vector_wall[0]*vector_wall_to_src[1]-vector_wall[1]*vector_wall_to_src[0]<0:
        tilt = tilt + pi
    N = 360
    for i in range(N):
        theta = pi*i/N
        direction = (10*cos(tilt+theta),10*sin(tilt+theta))
        level, wall_id, next_pt, dist = firstObjOnRay(config,pt,direction,ambient_wall=wall["id"])
        if level:
            res[level][wall_id].append((next_pt[0],next_pt[1]))

    return res

def scatteredToLevel(config, src_pt, lighting_pt, lighting_wall, lighting_level, increase_level):
    res = defaultdict(list)
    next_collisions = scatteredTo(config, src_pt, lighting_pt, lighting_wall)
    increment = int(copysign(1,increase_level))
    next_level = "level "+str(lighting_level+increment)

    if next_level not in next_collisions:
     #a safe check, in general we don't know level info
        return res

    if increase_level==1 or increase_level==-1:
        for wall in next_collisions[next_level]:
            res[wall] += next_collisions[next_level][wall]
        return res

    for wall_id in next_collisions[next_level]:
        for pt in next_collisions[next_level][wall_id]:
            wall = config["walls"][next_level][wall]
            wall["id"] = wall_id
            new_res = scatteredToLevel(config, lighting_pt, pt, wall,
                    lighting_level+increment, increase_level-increment)
            for wallname in new_res:
                res[wallname] += new_res[wallname]
    return res

def scatterPathByLevel(config, src_pt, lighting_pt, lighting_wall, lighting_level, increase_level):
    paths = defaultdict(list)

    next_collisions = scatteredTo(config, src_pt, lighting_pt, lighting_wall)

    increment = int(copysign(1,increase_level))
    next_level = "level "+str(lighting_level+increment)

    if next_level not in next_collisions:
     #a safe check, in general we don't know level info
        return paths

    if increase_level==1 or increase_level==-1:
        for wallname in next_collisions[next_level]:
            for pt in next_collisions[next_level][wallname]:
                #use "x y" as key
                paths[str(int(pt[0]))+' '+str(int(pt[1]))].append([src_pt,lighting_pt,pt])
        return paths

    for wall_id in next_collisions[next_level]:
        #print(wall)
        for pt in next_collisions[next_level][wall_id]:
            wall = config["walls"][next_level][wall_id]
            next_paths = scatterPathByLevel(config, lighting_pt, pt, wall,
                    lighting_level+increment, increase_level-increment)
            for ptkey in next_paths:
                paths[ptkey] += [[src_pt]+path for path in next_paths[ptkey]]
    return paths

def kBounceIntoCamAimed(config,n,shoot_pt,observe_pt):
    #n is odd
    k = (n-1)/2
    #attack in the middle trick
    wall = config["walls"]["level 1"]["wall 1"]
    cam = (config["cam"]["x"],config["cam"]["y"])
    way_out = scatterPathByLevel(config, cam, shoot_pt, wall, 1, k)
    way_back = scatterPathByLevel(config, cam, observe_pt, wall, 1, k)
    paths = []
    for ptkey in way_out:
        if ptkey in way_back:
            for up in way_out[ptkey]:
                for down in way_back[ptkey]:
                    paths.append(up+list(reversed(down)))

    return paths

def travelDist(path):
    d = 0
    for i in range(len(path)-1):
        d += sqrt(pow(path[i][0]-path[i+1][0],2),pow(path[i][1]-path[i+1][1],2))
    return d

def unobservedTravelDist(path):
    d = 0
    for i in range(1,len(path)-2):
        d += sqrt(pow(path[i][0]-path[i+1][0],2)+pow(path[i][1]-path[i+1][1],2))
    return d

if __name__ == "__main__":
    config = loadConfig("config.json")
    im = drawConfig(config)

    randpt1 = (randint(config["walls"]["level 1"]["wall 1"]["start"]["x"],config["walls"]["level 1"]["wall 1"]["end"]["x"]),config["walls"]["level 1"]["wall 1"]["start"]["y"])
    randpt2 = (randint(config["walls"]["level 1"]["wall 1"]["start"]["x"],config["walls"]["level 1"]["wall 1"]["end"]["x"]),config["walls"]["level 1"]["wall 1"]["start"]["y"])

    print("shooting: ", randpt1)
    print("observing: ", randpt2)

    paths = kBounceIntoCamAimed(config, 5, randpt1, randpt2)
    for path in paths:
        im = drawPath(im, path)

    im.save("light_path_simulator.png")
