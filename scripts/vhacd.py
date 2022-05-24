#!/usr/bin/python

import pybullet as p
import pybullet_data as pd
import os
import sys

if len(sys.argv) is 3:
  path = sys.argv[1]
  obj_name = sys.argv[2]

  p.connect(p.DIRECT)
  p.vhacd(path + "/" + obj_name + ".obj", path + "/" + obj_name + "_vhacd.obj", path + "/log.txt", alpha=0.04,resolution=50000 )
else:
  print("provide the mesh path and its name in argument of this script")