from Simulator import loadConfig, unobservedTravelDist, kBounceIntoCamAimed, travelDist
from Drawer import drawConfig, drawEllipse, drawPath, findEllipsePts
from math import sqrt,atan2
from random import randint
import copy
from collections import defaultdict
from PIL import ImageDraw, Image
from pylab import cm
import numpy as np
import datetime

config = loadConfig("config.json")
know_env = copy.deepcopy(config)
del know_env["walls"]["level 2"]
del know_env["walls"]["level 3"]
know_Im = drawConfig(know_env)
#im = know_Im.copy()

geo_filename='examples/decouple_geometry_{:%m_%d_%H_%M_%S}.png'.format(datetime.datetime.now())
all_filename='examples/decouple_before_{:%m_%d_%H_%M_%S}.png'.format(datetime.datetime.now())
decouple_filename='examples/decoupled_{:%m_%d_%H_%M_%S}.png'.format(datetime.datetime.now())

voter = defaultdict(int)
geo_voter = defaultdict(int)
decouple_voter = defaultdict(int)

density = np.zeros((400,400))
geo_density = np.zeros((400,400))
decouple_density = np.zeros((400,400))
fociList = [((175,210),(200,210)),((190,210),(220,210))]
tilt = 0
discrete_spots = [(x,config["walls"]["level 1"]["wall 1"]["start"]["y"]) for x in range(config["walls"]["level 1"]["wall 1"]["start"]["x"],config["walls"]["level 1"]["wall 1"]["end"]["x"],2)]
print(discrete_spots)
ellip_dict = defaultdict(int)

for t in range(10):
    randpt1 = (randint(config["walls"]["level 1"]["wall 1"]["start"]["x"],config["walls"]["level 1"]["wall 1"]["end"]["x"]),config["walls"]["level 1"]["wall 1"]["start"]["y"])
    randpt2 = (randint(config["walls"]["level 1"]["wall 1"]["start"]["x"],config["walls"]["level 1"]["wall 1"]["end"]["x"]),config["walls"]["level 1"]["wall 1"]["start"]["y"])
    print("shooting: ", randpt1)
    print("observing: ", randpt2)

    paths = kBounceIntoCamAimed(config, 3, randpt1, randpt2)
    firstReturn = None
    shortestTime = float("inf")
    for path in paths:
        #im = drawPath(im, path)
        d = travelDist(path)
        if d<shortestTime:
            shortestTime = d
            firstReturn = path

        #drawPath(know_Im,firstReturn)
        for f1 in discrete_spots:
            for f2 in discrete_spots:
                longAxis = d - sqrt(pow(randpt1[0]-config["cam"]["x"],2)+pow(randpt1[1]-config["cam"]["y"],2))-sqrt(pow(randpt2[0]-config["cam"]["x"],2)+pow(randpt2[1]-config["cam"]["y"],2))
                #if d>0: print(d)
                b2 = pow(longAxis,2)-(pow(f1[0]-f2[0],2)+pow(f1[1]-f2[1],2))
                if longAxis>0 and b2>0:
                    c_x = (f1[0]+f2[0])/2
                    c_y = (f1[1]+f2[1])/2
                    ellip_id = str(c_x)+"_"+str(c_y)+"_"+str(int(longAxis))+"_"+str(int(b2))


                    pts = findEllipsePts(c_x,c_y,
                                longAxis/2,sqrt(b2)/2,
                                tilt)

                    if ellip_dict[ellip_id]<2:
                        ellip_dict[ellip_id] += 1
                        for pt in pts:
                            if pt[0]>-1 and pt[0]<400 and pt[1]>-1 and pt[1]<400:
                                #geo_voter[str(pt[0])+' '+str(pt[1])] += 1
                                geo_density[pt[1]][pt[0]] += 1
                    else:
                        for pt in pts:
                            if pt[0]>-1 and pt[0]<400 and pt[1]>-1 and pt[1]<400:
                                #decouple_voter[str(pt[0])+' '+str(pt[1])] += 1
                                decouple_density[pt[1]][pt[0]] += 1

                    for pt in pts:
                        if pt[0]>-1 and pt[0]<400 and pt[1]>-1 and pt[1]<400:
                            voter[str(pt[0])+' '+str(pt[1])] += 1
                            density[pt[1]][pt[0]] += 1

M = max(voter.values())
im = Image.fromarray(cm.gist_earth(density/M, bytes=True))
im = drawConfig(config,im)
im.show()
im.save(all_filename)

geo_im = Image.fromarray(cm.gist_earth(geo_density/M, bytes=True))
geo_im = drawConfig(config,geo_im)
geo_im.show()
geo_im.save(geo_filename)

decouple_im = Image.fromarray(cm.gist_earth(decouple_density/M, bytes=True))
decouple_im = drawConfig(config,decouple_im)
decouple_im.show()
decouple_im.save(decouple_filename)

threshold = M*0.7
highVotePts = [(int(k.split()[0]),int(k.split()[1])) for k, v in voter.items() if v > threshold]
draw = ImageDraw.Draw(know_Im)
draw.point(highVotePts,fill=255)
know_Im.show()
