import os
dir = "/RRT example/media"
for f in os.listdir(dir):
    os.remove(os.path.join(dir, f))