#!/usr/bin/env python
# Script to sort bag files

import os
import shutil
import numpy as np
import rosbag

if __name__=="__main__":
    for f in os.listdir("."):
        if f.endswith(".bag"):
            bag = rosbag.Bag(f)
            for topic, m, _ in bag.read_messages():
                if topic == "/turtlepi_navigate/episode_result":
                    if m.data == "SUCCEEDED":
                        shutil.move(os.path.join(".", f),
                                    os.path.join("./success", f))
                    else:
                        shutil.move(os.path.join(".", f),
                                    os.path.join("./nosuccess", f))
            
