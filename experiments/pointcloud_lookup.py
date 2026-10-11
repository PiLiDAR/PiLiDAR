import cv2
import numpy as np
import open3d as o3d


def angular_from_cartesian(cartesian_points):
    r = np.sqrt(np.sum(cartesian_points**2, axis=1)) + 1e-10  # hack: avoid division by zero
    theta = np.arccos(cartesian_points[:, 2] / r)
    phi = np.arctan2(cartesian_points[:, 1], cartesian_points[:, 0])
    angular_points = np.stack([theta, r, phi], axis=1)
    return angular_points

def get_sampling_coordinates(angular_points, img_shape, z_rotate=0):
    image_height, image_width = img_shape
 
    longitude = angular_points[:, 2] + np.deg2rad(90 + z_rotate)
    longitude = (longitude + 2 * np.pi) % (2 * np.pi)
    image_x = (2 * np.pi - longitude) / (2 * np.pi) * image_width
    image_x = np.round(image_x).astype(int)
    image_x = np.clip(image_x, 0, image_width - 1)

    latitude = np.pi / 2 - angular_points[:, 0]
    latitude = (latitude + np.pi / 2) % np.pi
    image_y = (1 - latitude / np.pi) * image_height
    image_y = np.round(image_y).astype(int)
    image_y = np.clip(image_y, 0, image_height - 1)

    return image_x, image_y

def angular_lookup(angular_points, pano, scale=1, degrees=False, z_rotate=0, as_float=True):
    if degrees:
        angular_points = np.deg2rad(angular_points)  # degrees to radians

    image_height, image_width, _ = pano.shape
    pano_RGB = cv2.cvtColor(pano, cv2.COLOR_BGR2RGB)

    if scale != 1:
        image_height = int(image_height * scale)
        image_width = int(image_height * 2)  # spherical map aspect ratio is 2:1
        pano_RGB = cv2.resize(pano_RGB, (image_width, image_height), interpolation=cv2.INTER_AREA)
    
    image_x, image_y = get_sampling_coordinates(angular_points, (image_height, image_width), z_rotate=z_rotate)
    colors = pano_RGB[image_y, image_x]

    if as_float:
        colors = colors.astype(np.float32) / 255
    return colors


if __name__ == "__main__":
    from lib.pointcloud import load_pointcloud, transform
    from lib.visualization import visualize


    pcd = load_pointcloud("export/scan_01.ply")
    map = cv2.imread("export/pano_01.jpg")
    scale = 0.5          # image size 50 % 
    z_rotate = -15.5     # degrees
    z_offset = 0.4       # offset in mm * -1

    pcd = transform(pcd, translate=(0, 0, z_offset))

    angular_points = angular_from_cartesian(np.asarray(pcd.points))
    colors = angular_lookup(angular_points, map, scale=scale, z_rotate=z_rotate)

    pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors))

    visualize([pcd], view="front", unlit=True)
    cv2.waitKey(0)
