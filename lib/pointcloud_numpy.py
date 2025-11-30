import os
import threading
import pickle
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree


def load_raw_scan(path):
    with open(path, "rb") as f:
        raw_scan = pickle.load(f)
    return raw_scan


def remove_NaN(array):
    return array[~np.isnan(array).any(axis=1)]


def remove_statistical_outliers(points: np.ndarray, k_neighbors: int = 20, std_ratio: float = 2.0) -> tuple:
    """
    Remove statistical outliers using k-nearest neighbors analysis (NumPy implementation).
    
    This function computes the mean distance from each point to its k nearest neighbors,
    then removes points whose mean distance exceeds mean + std_ratio * std_dev.
    
    Args:
        points: Nx3 or Nx4 numpy array of 3D points (optionally with intensity)
        k_neighbors: Number of nearest neighbors to consider (default: 20)
        std_ratio: Standard deviation multiplier for threshold (default: 2.0)
    
    Returns:
        Tuple of (filtered_points, inlier_mask)
        - filtered_points: Points that are not outliers
        - inlier_mask: Boolean mask indicating which points are inliers
    
    Example:
        >>> points = np.random.rand(1000, 3)
        >>> filtered, mask = remove_statistical_outliers(points, k_neighbors=20, std_ratio=2.0)
        >>> print(f"Removed {(~mask).sum()} outliers from {len(points)} points")
    """
    if points.shape[0] < k_neighbors:
        print(f"Warning: Point cloud has {points.shape[0]} points but k_neighbors={k_neighbors}. Skipping outlier removal.")
        return points, np.ones(points.shape[0], dtype=bool)
    
    # Extract XYZ coordinates (ignore intensity if present)
    coords = points[:, :3]
    
    # Build KD-tree for fast nearest neighbor search
    tree = cKDTree(coords)
    
    # Query k+1 nearest neighbors (includes the point itself)
    distances, _ = tree.query(coords, k=k_neighbors + 1)
    
    # Compute mean distance to k neighbors (exclude distance 0 to self)
    mean_distances = np.mean(distances[:, 1:], axis=1)
    
    # Compute global statistics
    global_mean = np.mean(mean_distances)
    global_std = np.std(mean_distances)
    
    # Threshold: mean + std_ratio * std
    threshold = global_mean + std_ratio * global_std
    
    # Inliers are points below threshold
    inlier_mask = mean_distances < threshold
    
    num_outliers = (~inlier_mask).sum()
    outlier_percent = (num_outliers / len(points)) * 100
    
    print(f"Statistical outlier removal: {num_outliers} outliers ({outlier_percent:.2f}%) removed")
    print(f"  k_neighbors={k_neighbors}, std_ratio={std_ratio}, threshold={threshold:.4f}")
    
    return points[inlier_mask], inlier_mask


def rotate_3D(points3d, rotation_degrees, translation_vector=(0, 0, 0), rotation_axis=(0, 0, 1)):
    rotation_axis = np.array(rotation_axis)
    rotation_radians = np.radians(rotation_degrees)
    rotation_vector = rotation_radians * rotation_axis
    rotation = R.from_rotvec(rotation_vector)
    rotation_matrix = rotation.as_matrix()

    pts = points3d[:, 0:3]
    pts_translated = pts + np.asarray(translation_vector)
    result_points = pts_translated @ rotation_matrix.T

    if points3d.shape[1] == 4:
        result_points = np.column_stack((result_points, points3d[:, 3]))
    return result_points


def merge_2D_points(raw_scan, z_step=1, ccw=False, position_offset=(0, 0, 0), angle_offset=0, up_vector=(0, 0, 1)):
    z_angles = raw_scan["z_angles"]
    cartesian_list = raw_scan["cartesian"]

    pointcloud = np.zeros((1, 4))
    z_angle = 0

    for i, points2d in enumerate(cartesian_list):
        points3d = np.insert(points2d, 1, values=0, axis=1)

        if z_angles is not None:
            z_angle = z_angles[i]
        else:
            z_angle = z_angle - z_step if ccw else z_angle + z_step

        points3d = rotate_3D(points3d, angle_offset, rotation_axis=np.array((0, 1, 0)))
        points3d = rotate_3D(points3d, -z_angle, translation_vector=position_offset, rotation_axis=np.array(up_vector))
        pointcloud = np.append(pointcloud, points3d, axis=0)

    pointcloud = remove_NaN(pointcloud)
    return pointcloud


def angular_from_cartesian(cartesian_points):
    r = np.sqrt(np.sum(cartesian_points ** 2, axis=1)) + 1e-10
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


def angular_lookup(angular_points, pano, scale=1, degrees=False, z_rotate=0):
    if degrees:
        angular_points = np.deg2rad(angular_points)

    image_height, image_width, _ = pano.shape
    pano_RGB = cv2.cvtColor(pano, cv2.COLOR_BGR2RGB)

    if scale != 1:
        image_height = int(image_height * scale)
        image_width = int(image_height * 2)
        pano_RGB = cv2.resize(pano_RGB, (image_width, image_height), interpolation=cv2.INTER_AREA)

    image_x, image_y = get_sampling_coordinates(angular_points, (image_height, image_width), z_rotate=z_rotate)
    colors = pano_RGB[image_y, image_x]
    return colors


def save_pointcloud_numpy(filepath, points, colors=None, intensities=None, ascii=True):
    directory, filename = os.path.split(filepath)
    os.makedirs(directory, exist_ok=True)

    n = points.shape[0]
    has_colors = colors is not None
    has_intensities = intensities is not None

    header = [
        "ply",
        "format ascii 1.0" if ascii else "format binary_little_endian 1.0",
        f"element vertex {n}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if has_intensities:
        header.append("property float intensity")
    if has_colors:
        header.extend(["property uchar red", "property uchar green", "property uchar blue"])
    header.append("end_header")

    with open(filepath, "wb") as f:
        f.write(("\n".join(header) + "\n").encode("ascii"))
        if ascii:
            if has_intensities and has_colors:
                data = np.column_stack([points.astype(np.float32), intensities.astype(np.float32).reshape(-1), colors.astype(np.uint8)])
                np.savetxt(f, data, fmt="%f %f %f %f %d %d %d")
            elif has_colors:
                data = np.column_stack([points.astype(np.float32), colors.astype(np.uint8)])
                np.savetxt(f, data, fmt="%f %f %f %d %d %d")
            elif has_intensities:
                data = np.column_stack([points.astype(np.float32), intensities.astype(np.float32).reshape(-1)])
                np.savetxt(f, data, fmt="%f %f %f %f")
            else:
                np.savetxt(f, points.astype(np.float32), fmt="%f %f %f")
        else:
            # Force ASCII to ensure maximal compatibility (CloudCompare)
            # Binary requires strict dtype packing per property; skipping for now.
            if has_intensities and has_colors:
                data = np.column_stack([points.astype(np.float32), intensities.astype(np.float32).reshape(-1), colors.astype(np.uint8)])
                np.savetxt(f, data, fmt="%f %f %f %f %d %d %d")
            elif has_colors:
                data = np.column_stack([points.astype(np.float32), colors.astype(np.uint8)])
                np.savetxt(f, data, fmt="%f %f %f %d %d %d")
            elif has_intensities:
                data = np.column_stack([points.astype(np.float32), intensities.astype(np.float32).reshape(-1)])
                np.savetxt(f, data, fmt="%f %f %f %f")
            else:
                np.savetxt(f, points.astype(np.float32), fmt="%f %f %f")


def save_pointcloud_threaded(points, output_path, colors=None, intensities=None, ascii=True):
    # Avoid threading to mitigate OpenBLAS munmap warnings on Pi
    save_pointcloud_numpy(output_path, points, colors, intensities, ascii)


def process_raw(config, save=True):
    if not os.path.exists(config.raw_path):
        raise FileNotFoundError(f"Raw scan file not found: {config.raw_path}")

    raw_scan = load_raw_scan(config.raw_path)

    # IMU data ignored in NumPy backend cleanup (no IMU used on Pi5 setup)

    print("Merging 2D points to 3D (NumPy backend)...")
    array_3D = merge_2D_points(
        raw_scan,
        position_offset=(0, 0, 0),  # Apply Y offset separately after inversion
        angle_offset=config.get("LIDAR", "LIDAR_OFFSET_ANGLE"),
        up_vector=(0, 0, 1),
    )
    print("Merge complete. Array shape:", array_3D.shape)
    # Diagnostics: pre-offset stats
    pts = array_3D[:, 0:3]
    finite_mask = np.isfinite(pts).all(axis=1)
    pts_finite = pts[finite_mask]
    pts_valid = pts_finite[np.all(np.abs(pts_finite) < 1e6, axis=1)]
    pre_lo = np.percentile(pts_valid, 0.1, axis=0)
    pre_hi = np.percentile(pts_valid, 99.9, axis=0)
    pre_centroid = np.mean(pts_valid, axis=0)
    print(f"Pre-offset bounds X[{pre_lo[0]:.3f},{pre_hi[0]:.3f}] Y[{pre_lo[1]:.3f},{pre_hi[1]:.3f}] Z[{pre_lo[2]:.3f},{pre_hi[2]:.3f}] centroid {pre_centroid}")
    print(f"Applied LIDAR_OFFSET_ANGLE={config.get('LIDAR','LIDAR_OFFSET_ANGLE')} deg, Y_OFFSET={config.get('3D','Y_OFFSET')} mm, Z_OFFSET={config.get('3D','Z_OFFSET')} mm, SCALE={config.get('3D','SCALE')}")

    max_points = 600000
    if array_3D.shape[0] > max_points:
        stride = int(np.ceil(array_3D.shape[0] / max_points))
        array_3D = array_3D[::stride]
        print(f"Downsampled to {array_3D.shape[0]} points with stride {stride}.")

    # Y and Z offset, invert Y and Z axes, and scale on numpy
    # Apply inverted offsets: positive offset shifts in negative direction after inversion
    array_3D[:, 1] -= config.get("3D", "Y_OFFSET")  # Subtract Y_OFFSET
    array_3D[:, 1] *= -1  # Invert Y axis (mirror Y coordinates)
    array_3D[:, 2] -= config.get("3D", "Z_OFFSET")  # Subtract Z_OFFSET
    array_3D[:, 2] *= -1  # Invert Z axis (points above scanner should be positive)
    scene_scale = config.get("3D", "SCALE")
    if scene_scale != 1:
        array_3D[:, 0:3] *= scene_scale
    # Diagnostics: post-offset stats
    pts2 = array_3D[:, 0:3]
    finite_mask2 = np.isfinite(pts2).all(axis=1)
    pts2_finite = pts2[finite_mask2]
    pts2_valid = pts2_finite[np.all(np.abs(pts2_finite) < 1e6, axis=1)]
    post_lo = np.percentile(pts2_valid, 0.1, axis=0)
    post_hi = np.percentile(pts2_valid, 99.9, axis=0)
    post_centroid = np.mean(pts2_valid, axis=0)
    print(f"Post-offset bounds X[{post_lo[0]:.3f},{post_hi[0]:.3f}] Y[{post_lo[1]:.3f},{post_hi[1]:.3f}] Z[{post_lo[2]:.3f},{post_hi[2]:.3f}] centroid {post_centroid}")

    # Filter out invalid points: remove NaN/Inf and extreme outliers (beyond ±10m after scale)
    pts_clean = array_3D[:, 0:3]
    valid_mask = np.isfinite(pts_clean).all(axis=1) & (np.abs(pts_clean) < 10).all(axis=1)
    array_3D_clean = array_3D[valid_mask]
    pts_removed = array_3D.shape[0] - array_3D_clean.shape[0]
    if pts_removed > 0:
        print(f"Filtered out {pts_removed} invalid/extreme points ({pts_removed/array_3D.shape[0]*100:.2f}%)")
    array_3D = array_3D_clean

    # Statistical outlier removal (NumPy backend)
    if config.get("FILTERING", "STATISTICAL_OUTLIER_REMOVAL", default=False):
        k_neighbors = config.get("FILTERING", "STATISTICAL_K_NEIGHBORS", default=20)
        std_ratio = config.get("FILTERING", "STATISTICAL_STD_RATIO", default=2.0)
        array_3D, _ = remove_statistical_outliers(array_3D, k_neighbors=k_neighbors, std_ratio=std_ratio)

    # Colors: pano texture mapping or intensity fallback
    colors_uint8 = None
    if config.get("ENABLE_VERTEXCOLOUR") and os.path.exists(config.pano_path):
        print(f"Mapping vertex colors from panorama ({config.pano_path})...")
        pano_bgr = cv2.imread(config.pano_path)
        if pano_bgr is not None:
            ang = angular_from_cartesian(array_3D[:, 0:3])
            colors_uint8 = angular_lookup(ang, pano_bgr, 
                                         scale=config.get("VERTEXCOLOUR", "SCALE"), 
                                         z_rotate=config.get("VERTEXCOLOUR", "Z_ROTATE"))
            print(f"Applied pano texture to {colors_uint8.shape[0]} points.")
        else:
            print("Warning: Could not load panorama image, falling back to intensity colors.")
    
    if colors_uint8 is None:
        # Fallback: green-to-red mapping from intensity (0..255)
        intensities = np.clip(array_3D[:, 3], 0, 255).astype(np.float32)
        norm = intensities / 255.0
        reds = (norm * 255).astype(np.uint8)
        greens = ((1.0 - norm) * 255).astype(np.uint8)
        blues = np.zeros_like(reds, dtype=np.uint8)
        colors_uint8 = np.stack([reds, greens, blues], axis=-1)

    if save:
        print(f"Saving PLY via NumPy backend (ASCII, {array_3D.shape[0]} valid points)...")
        save_pointcloud_numpy(
            filepath=config.pcd_path,
            points=array_3D[:, 0:3],
            colors=colors_uint8,
            intensities=None,
            ascii=True,
        )

    print("\nprocessing 3D (NumPy) completed.")
    return {"points": array_3D[:, 0:3], "colors": colors_uint8}
