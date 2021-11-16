import os
from shutil import copyfile

inpath = "/Users/meganfinch/documents/projects/colmap-tests/lion/images"
outpath = "/Users/meganfinch/documents/projects/colmap-tests/lion5/images"
imgs = os.listdir(inpath)

for img in imgs:
    camera = int(img.split(".")[1])
    if (camera % 5 == 0):
        copyfile(inpath + "/" + img, outpath + "/" + img)
