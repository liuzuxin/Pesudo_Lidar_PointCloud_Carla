import cv2
import re
import os
import numpy as np
from PIL import Image
import argparse

parser = argparse.ArgumentParser(description='Create video from png pictures. RUN IN PYTHON 3.5!! ')
parser.add_argument('--img-path', metavar='path',
                    dest='path', default='',
                    help='Path to folder where images is stored. The WHOLE path is needed.')
parser.add_argument('--save-path', metavar='path',
                    dest='save_path', default='',
                    help='Where to save the video. (default: same folder as images)')
parser.add_argument('--filename', metavar='name', dest='name', default='img_',
                    help='Prefix of name (before the number). Eg. \'img_\' in \'img_1.png\'. (default \'img_\')')
parser.add_argument('--xRes', metavar='N', type=int,
                    dest='xRes', default=640,
                    help='x resulution of images (default: 640)')
parser.add_argument('--yRes', metavar='N', type=int,
                    dest='yRes', default=480,
                    help='y resulution of images (default: 480)')
parser.add_argument('--fps', metavar='N', type=int,
                    dest='fps', default=10,
                    help='fps of video (default: 10)')

args = parser.parse_args()
if not args.save_path:
    args.save_path = args.path

image_folder = args.path
def create_video():
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video = cv2.VideoWriter(args.save_path + 'video.avi', fourcc, args.fps, (args.xRes,args.yRes)) #(640,480)

    images = []
    lenght = len(os.listdir(image_folder))
    indices = sorted(getIndices(image_folder))
    print(indices)
    for f in indices:
        frame = cv2.imread(image_folder + '%i.png' %f)
        images.append(frame)
        frame = np.array(frame)
        video.write(frame)

    video.release()
    cv2.destroyAllWindows()

def getIndices(path):
    res = []
    for filename in os.listdir(path):
        match = re.search('\d+(?=\.png)', filename)
        if match is not None:
            res.append(int(match.group(0)))
    return res

if __name__ == "__main__":
    create_video()
