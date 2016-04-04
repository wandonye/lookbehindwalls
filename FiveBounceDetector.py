from Simulator import loadConfig, unobservedTravelDist, kBounceIntoCamAimed
from Drawer import drawConfig, drawEllipse, drawPath, findEllipsePts
from math import sqrt,atan2
from random import randint
import copy
from collections import defaultdict
from PIL import ImageDraw, Image
from pylab import cm
import numpy as np
import datetime

config = loadConfig("config3_leftonly.json")
know_env = copy.deepcopy(config)
del know_env["walls"]["level 3"]
know_Im = drawConfig(know_env)
path_Im = drawConfig(config)
#im = know_Im.copy()
image_name='examples/fiveBounce_{:%m_%d_%H_%M_%S}.png'.format(datetime.datetime.now())

voter = defaultdict(int)
density = np.zeros((400,400))
fociList = [((175,210),(200,210)),((190,210),(220,210))]
tilt = 0
discrete_spots = [(config["walls"]["level 2"]["wall 2"]["start"]["x"],y) for y in range(config["walls"]["level 2"]["wall 2"]["start"]["y"],config["walls"]["level 2"]["wall 2"]["end"]["y"],5)]
print(discrete_spots)

for t in range(5):
    randpt1 = (randint(config["walls"]["level 1"]["wall 1"]["start"]["x"],config["walls"]["level 1"]["wall 1"]["end"]["x"]),config["walls"]["level 1"]["wall 1"]["start"]["y"])
    randpt2 = (randint(config["walls"]["level 1"]["wall 1"]["start"]["x"],config["walls"]["level 1"]["wall 1"]["end"]["x"]),config["walls"]["level 1"]["wall 1"]["start"]["y"])
    print("shooting: ", randpt1)
    print("observing: ", randpt2)

    paths = kBounceIntoCamAimed(config, 5, randpt1, randpt2)
    #print(paths)
    firstReturn = None
    shortestTime = float("inf")
    for path in paths:
        d = unobservedTravelDist(path)
        if d<shortestTime:
            shortestTime = d
            firstReturn = path

        #drawPath(know_Im,firstReturn)
        for f1 in discrete_spots:
            for f2 in discrete_spots:
                longAxis = d-sqrt(pow(f1[0]-randpt1[0],2)+pow(f1[1]-randpt1[1],2))-sqrt(pow(f1[0]-randpt2[0],2)+pow(f1[1]-randpt2[1],2))
                b2 = pow(longAxis,2)-(pow(f1[0]-f2[0],2)+pow(f1[1]-f2[1],2))
                if longAxis>0 and b2>0:
                    pts = findEllipsePts((f1[0]+f2[0])/2,(f1[1]+f2[1])/2,
                                longAxis/2,sqrt(b2)/2,
                                tilt)

                    for pt in pts:
                        if pt[0]>-1 and pt[0]<400 and pt[1]>-1 and pt[1]<400:
                            voter[str(pt[0])+' '+str(pt[1])] += 1
                            density[pt[1]][pt[0]] += 1

M = max(voter.values())
im = Image.fromarray(cm.gist_earth(density/M, bytes=True))
im = drawConfig(config,im)
im.show()
im.save(image_name)

threshold = M*0.7
highVotePts = [(int(k.split()[0]),int(k.split()[1])) for k, v in voter.items() if v > threshold]
draw = ImageDraw.Draw(know_Im)
draw.point(highVotePts,fill=255)
know_Im.show()
