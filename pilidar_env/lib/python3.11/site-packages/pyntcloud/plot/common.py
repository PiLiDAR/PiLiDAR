import numpy as np


def get_colors(cloud, use_as_color, cmap):
    try:
        colors = cloud.points[use_as_color].values
    except KeyError:
        colors = None
    if use_as_color != ["red", "green", "blue"] and colors is not None:
        import matplotlib.pyplot as plt

        s_m = plt.cm.ScalarMappable(cmap=cmap)
        colors = s_m.to_rgba(colors)[:, :-1] * 255
    elif colors is None:
        # default color orange
        colors = np.repeat([[255, 125, 0]], cloud.xyz.shape[0], axis=0)
    return colors.astype(np.uint8)
