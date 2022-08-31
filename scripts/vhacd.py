#!/usr/bin/python

import pybullet as p
import pybullet_data as pd
import os
import sys

if len(sys.argv) is 3:
  path = sys.argv[1]
  obj_name = sys.argv[2]

  p.connect(p.DIRECT)
  p.vhacd(path + "/" + obj_name + ".obj", path + "/" + obj_name + "_vhacd.obj", path + "/log.txt", resolution=16000000, depth=32, maxNumVerticesPerCH=1024)#, alpha=0.01, beta=-0.01, gamma=0.000125 )
else:
  print("provide the mesh path and its name in argument of this script")