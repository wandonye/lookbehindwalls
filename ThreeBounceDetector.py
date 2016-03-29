from Simulator import loadConfig, unobservedTravelDist, kBounceIntoCamAimed
from Drawer import drawConfig, drawEllipse, drawPath, findEllipsePts
from math import sqrt,atan2
from random import randint
import copy
from collections import defaultdict
from PIL import ImageDraw, Image
from pylab import cm
import numpy as np

config = loadConfig("config1.json")
know_env = copy.deepcopy(config)
del know_env["walls"]["level 2"]
del know_env["walls"]["level 3"]
know_Im = drawConfig(know_env)
#im = know_Im.copy()

voter = defaultdict(int)
density = np.zeros((400,400))
fociList = [((175,210),(200,210)),((190,210),(220,210))]
tilt = 0

for t in range(50):
    #randpt1 = fociList[t][0]
    #randpt2 = fociList[t][1]
    randpt1 = (randint(config["walls"]["level 1"]["wall 1"]["start"]["x"],config["walls"]["level 1"]["wall 1"]["end"]["x"]),config["walls"]["level 1"]["wall 1"]["start"]["y"])
    randpt2 = (randint(config["walls"]["level 1"]["wall 1"]["start"]["x"],config["walls"]["level 1"]["wall 1"]["end"]["x"]),config["walls"]["level 1"]["wall 1"]["start"]["y"])
    print("shooting: ", randpt1)
    print("observing: ", randpt2)

    paths = kBounceIntoCamAimed(config, 3, randpt1, randpt2)
    firstReturn = None
    shortestTime = float("inf")
    for path in paths:
        #im = drawPath(im, path)
        d = unobservedTravelDist(path)
        if d<shortestTime:
            shortestTime = d
            firstReturn = path

        #drawPath(know_Im,firstReturn)
        pts = findEllipsePts((randpt1[0]+randpt2[0])/2,(randpt1[1]+randpt2[1])/2,
                    d/2,sqrt(pow(d,2)-pow(randpt1[0]-randpt2[0],2)-pow(randpt1[1]-randpt2[1],2))/2,
                    tilt)

        for pt in pts:
            if pt[0]>-1 and pt[0]<400 and pt[1]>-1 and pt[1]<400:
                voter[str(pt[0])+' '+str(pt[1])] += 1
                density[pt[1]][pt[0]] += 1

M = max(voter.values())
im = Image.fromarray(cm.gist_earth(density/M, bytes=True))
im = drawConfig(config,im)
im.show()
im.save("examples/exampe2.png")

threshold = M*0.7
highVotePts = [(int(k.split()[0]),int(k.split()[1])) for k, v in voter.items() if v > threshold]
draw = ImageDraw.Draw(know_Im)
draw.point(highVotePts,fill=255)
know_Im.save("detecting.png")
