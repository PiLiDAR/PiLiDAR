# http://paulbourke.net/panorama/cubemaps/

import cv2
import numpy as np
import matplotlib.pyplot as plt
# import random

from lib.pano_utils import vector_to_longlat, longlat_to_spherical, sample_color


def denormalize_position(pos, size):
    """
    :param pos: tuple(x,y)  # normalized 0-1
    :param shape: tuple(width, height)
    :return: tuple(x,y)  # absolute pixel values
    """
    x, y = pos
    w, h = size
    return int(x * w), int(y * h)


def scatterplot(x, y, z=None, colors=None):
    fig = plt.figure(figsize=(12, 12))

    projection = None if z is None else "3d"
    ax = fig.add_subplot(projection=projection)


    ax.scatter(x, y, z, c=colors)
    plt.show()


img_path = "images/1707902930.332706.jpg"
img_BGR = cv2.imread(img_path)
img_RGB = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2RGB)

height, width, channels = img_RGB.shape
size = (width, height)

# ar = 5  # angular resolution in °
# x_list = []
# y_list = []
# colors = []
#
# for y in np.linspace(0, 1, num=int(180 / ar), endpoint=False):
#     for x in np.linspace(0, 1, num=int(360 / ar), endpoint=False):
#         xy = denormalize_position((x, y), (width, height))
#         colors.append(sample_color(img_RGB, xy, normalize_color=True))
#         # print(xy)
#         x_list.append(x)
#         y_list.append(-y)


# # dummy pointcloud
# x_list = list(range(0, 500))
# y_list = x_list.copy()
# z_list = x_list.copy()
# random.shuffle(y_list)
# random.shuffle(z_list)


u, v = np.mgrid[0:2 * np.pi:180j, 0:np.pi:90j]
x_list = np.cos(u) * np.sin(v)
y_list = np.sin(u) * np.sin(v)
z_list = np.cos(v)

x_list = x_list.flatten() + 0.01  # TODO bug: samples in line n, but should in n-1
y_list = y_list.flatten()
z_list = z_list.flatten()

br = 600
colors = []
for i, x in enumerate(x_list):
    point3d = (x_list[i], y_list[i], z_list[i])
    longlat = vector_to_longlat(point3d)
    uv = longlat_to_spherical(longlat)
    print(uv)

    color = sample_color(img_RGB, uv, normalize_color=True)
    colors.append(color)
    if i == br-1:
        break

scatterplot(x_list[0:br], y_list[0:br], z=z_list[0:br], colors=colors)
