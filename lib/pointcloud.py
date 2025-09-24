"""Point cloud utilities based on NumPy/Pandas/PyntCloud.

This module supersedes the previous Open3D backed implementation which caused
segmentation faults on the Raspberry Pi 5.  The public API remains largely
identical so the rest of PiLiDAR can continue to import the same functions.
"""

from __future__ import annotations

import os
import threading
import pickle
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import matplotlib
import numpy as np
import pandas as pd
import pye57
from pyntcloud import PyntCloud
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R


@dataclass
class PointCloudData:
    """Lightweight point cloud container with numpy storage."""

    points: np.ndarray
    colors: Optional[np.ndarray] = None
    intensities: Optional[np.ndarray] = None
    normals: Optional[np.ndarray] = None

    def __post_init__(self) -> None:
        self.points = np.asarray(self.points, dtype=np.float64)
        if self.colors is not None:
            self.colors = np.asarray(self.colors, dtype=np.float64)
            if self.colors.ndim == 1:
                self.colors = np.tile(self.colors[:, None], (1, 3))
        if self.intensities is not None:
            self.intensities = np.asarray(self.intensities, dtype=np.float64)
        if self.normals is not None:
            self.normals = np.asarray(self.normals, dtype=np.float64)

    def __len__(self) -> int:  # pragma: no cover - trivial
        return self.points.shape[0]

    def copy(self) -> "PointCloudData":
        return PointCloudData(
            points=self.points.copy(),
            colors=None if self.colors is None else self.colors.copy(),
            intensities=None if self.intensities is None else self.intensities.copy(),
            normals=None if self.normals is None else self.normals.copy(),
        )

    def select_by_index(self, indices: Sequence[int]) -> "PointCloudData":
        idx = np.asarray(indices, dtype=int)
        colors = None if self.colors is None else self.colors[idx]
        intensities = None if self.intensities is None else self.intensities[idx]
        normals = None if self.normals is None else self.normals[idx]
        return PointCloudData(self.points[idx], colors=colors, intensities=intensities, normals=normals)

    def with_colors(self, colors: np.ndarray) -> "PointCloudData":
        result = self.copy()
        result.colors = np.asarray(colors, dtype=np.float64)
        return result

    def with_intensities(self, intensities: np.ndarray) -> "PointCloudData":
        result = self.copy()
        result.intensities = np.asarray(intensities, dtype=np.float64)
        return result

    def to_dataframe(self) -> pd.DataFrame:
        df = pd.DataFrame(self.points, columns=["x", "y", "z"], dtype=np.float64)
        if self.colors is not None:
            rgb = np.clip(self.colors, 0.0, 1.0)
            df[["red", "green", "blue"]] = (rgb * 255.0).astype(np.float64)
        if self.intensities is not None:
            df["intensity"] = self.intensities.astype(np.float64)
        if self.normals is not None:
            df[["nx", "ny", "nz"]] = self.normals.astype(np.float64)
        return df

    def to_pyntcloud(self) -> PyntCloud:
        return PyntCloud(self.to_dataframe())


# ----------------------------------------------------------------------------
# Raw scan handling
# ----------------------------------------------------------------------------


def get_scan_dict(
    z_angles,
    angular_list=None,
    cartesian_list=None,
    packages=None,
    metadata=None,
    scan_id=None,
    device_id=None,
    sensor=None,
    hardware=None,
    location=None,
    author=None,
    stepper_log=None,
):
    header = {
        "scan_id": scan_id,
        "device_id": device_id,
        "sensor": sensor,
        "hardware": hardware,
        "location": location,
        "author": author,
    }
    if metadata:
        header.update(metadata)

    raw_scan = {
        "header": header,
        "z_angles": z_angles,
        "angular": angular_list,
        "cartesian": cartesian_list,
        "packages": packages,
    }
    if stepper_log is not None:
        raw_scan["stepper_log"] = stepper_log
    return raw_scan


def save_raw_scan(path: str, data: Dict[str, Any]) -> None:
    if isinstance(data, dict):
        with open(path, "wb") as handle:
            pickle.dump(data, handle)


def load_raw_scan(path: str) -> Dict[str, Any]:
    with open(path, "rb") as handle:
        return pickle.load(handle)


# ----------------------------------------------------------------------------
# Point cloud processing pipeline
# ----------------------------------------------------------------------------


def process_raw(config, save: bool = True) -> Dict[str, Optional[PointCloudData]]:
    """Convert recorded LiDAR data into point cloud representations."""

    if not os.path.exists(config.raw_path):
        raise FileNotFoundError(f"Raw LiDAR data not found at {config.raw_path}")

    raw_scan = load_raw_scan(config.raw_path)

    array_3d = merge_2D_points(
        raw_scan,
        position_offset=(0, config.get("3D", "Y_OFFSET"), 0),
        angle_offset=config.get("LIDAR", "LIDAR_OFFSET_ANGLE"),
        up_vector=(0, 0, 1),
    )

    base_cloud = PointCloudData(
        points=array_3d[:, :3],
        intensities=array_3d[:, 3] / 255.0,
    )

    base_cloud = transform(base_cloud, translate=(0, 0, config.get("3D", "Z_OFFSET")))
    scene_scale = config.get("3D", "SCALE")
    if scene_scale != 1:
        base_cloud = transform(base_cloud, scale=scene_scale)

    # Always compute the intensity-colored version (matching upstream fallback behaviour)
    intensity_cloud = colormap_pcd(base_cloud.copy(), gamma=1, cmap="viridis")
    if save:
        save_pointcloud_threaded(
            intensity_cloud,
            config.intensity_pcd_path,
            ply_ascii=config.get("3D", "ASCII"),
        )

    vertex_cloud: Optional[PointCloudData] = None
    pano_path = config.pano_path
    if config.get("ENABLE_VERTEXCOLOUR") and os.path.exists(pano_path):
        pano = cv2.imread(pano_path)
        if pano is None:
            print("Warnung: Panorama konnte nicht geladen werden.")
        else:
            colors = angular_lookup(
                angular_from_cartesian(base_cloud.points),
                pano,
                scale=config.get("VERTEXCOLOUR", "SCALE"),
                z_rotate=config.get("VERTEXCOLOUR", "Z_ROTATE"),
                as_float=True,
            )
            vertex_cloud = base_cloud.with_colors(colors)
            if save:
                save_pointcloud_threaded(
                    vertex_cloud,
                    config.vertex_pcd_path,
                    ply_ascii=config.get("3D", "ASCII"),
                )
    elif config.get("ENABLE_VERTEXCOLOUR"):
        print("Panorama-Färbung übersprungen (deaktiviert oder keine Bilddatei gefunden).")

    filtered_cloud: Optional[PointCloudData] = None
    if config.get("ENABLE_FILTERING"):
        low_res = downsample(intensity_cloud, voxel_size=config.get("FILTERING", "VOXEL_SIZE"))
        nb_points = config.get("FILTERING", "NB_POINTS")
        radius = config.get("FILTERING", "RADIUS")
        filtered_low = filter_outliers(low_res, nb_points=nb_points, radius=radius)
        filtered_cloud = filter_by_reference(intensity_cloud, filtered_low, radius=radius)
        if save:
            save_pointcloud_threaded(filtered_cloud, config.filtered_pcd_path, ply_ascii=config.get("3D", "ASCII"))

    print("\nprocessing 3D completed.")
    return {"intensity": intensity_cloud, "vertex": vertex_cloud, "filtered": filtered_cloud}


# ----------------------------------------------------------------------------
# Point cloud utilities
# ----------------------------------------------------------------------------


def remove_NaN(array: np.ndarray) -> np.ndarray:
    return array[~np.isnan(array).any(axis=1)]


def merge_2D_points(
    raw_scan: Dict[str, Any],
    z_step: float = 1,
    ccw: bool = False,
    position_offset: Tuple[float, float, float] = (0, 0, 0),
    angle_offset: float = 0,
    up_vector: Tuple[float, float, float] = (0, 0, 1),
) -> np.ndarray:
    z_angles = raw_scan["z_angles"]
    cartesian_list = raw_scan["cartesian"]

    assembled: List[np.ndarray] = []
    z_angle = 0.0

    for idx, points2d in enumerate(cartesian_list or []):
        if not isinstance(points2d, np.ndarray):
            points2d = np.asarray(points2d)
        if points2d.size == 0:
            continue

        # Always treat the last column as intensity and keep remaining columns
        # as geometric attributes.
        if points2d.shape[1] < 2:
            continue

        x_vals = points2d[:, 0].astype(np.float64)
        y_vals = points2d[:, 1].astype(np.float64)

        if points2d.shape[1] >= 4:
            z_base = points2d[:, 2].astype(np.float64)
            extras = points2d[:, 3:]
        else:
            z_base = np.zeros_like(x_vals)
            extras = points2d[:, 2:]

        intensity = extras[:, :1] if extras.size else np.empty((len(x_vals), 0))
        remaining = extras[:, 1:] if extras.size > 1 else np.empty((len(x_vals), 0))

        points3d = np.column_stack((x_vals, z_base, y_vals))

        if z_angles is not None:
            z_angle = z_angles[idx]
        else:
            z_angle = z_angle - z_step if ccw else z_angle + z_step

        rotated = rotate_3D(points3d, angle_offset, rotation_axis=(0, 1, 0))
        rotated = rotate_3D(rotated, -z_angle, translation_vector=position_offset, rotation_axis=up_vector)

        if intensity.size:
            rotated = np.column_stack((rotated, intensity))
        if remaining.size:
            rotated = np.column_stack((rotated, remaining))

        assembled.append(rotated)

    if not assembled:
        return np.zeros((0, 4))

    merged = np.concatenate(assembled, axis=0)
    return remove_NaN(merged)


def rotate_3D(
    points3d: np.ndarray,
    rotation_degrees: float,
    translation_vector: Tuple[float, float, float] = (0, 0, 0),
    rotation_axis: Tuple[float, float, float] = (0, 0, 1),
) -> np.ndarray:
    points3d = np.asarray(points3d, dtype=np.float64)
    translation_vector = np.asarray(translation_vector, dtype=np.float64)
    rotation_axis = np.asarray(rotation_axis, dtype=np.float64)
    rotation_axis = rotation_axis / (np.linalg.norm(rotation_axis) + 1e-12)

    rotation = R.from_rotvec(np.deg2rad(rotation_degrees) * rotation_axis)
    rotated = rotation.apply(points3d[:, :3] + translation_vector)

    if points3d.shape[1] > 3:
        rotated = np.column_stack((rotated, points3d[:, 3:]))
    return rotated


def pcd_from_np(
    array: np.ndarray,
    columns: str = "XYZI",
    estimate_normals: bool = True,
    colors: Optional[np.ndarray] = None,
    radius: float = 10,
    max_nn: int = 30,
) -> PointCloudData:
    if not isinstance(array, np.ndarray):
        array = np.asarray(array)
    if colors is not None and not isinstance(colors, np.ndarray):
        colors = np.asarray(colors)

    columns = columns.upper()
    zeros = np.zeros((array.shape[0], 1))

    if "XYZ" in columns:
        points = array[:, 0:3]
        color_index = 3
    else:
        color_index = 2
        if "XY" in columns:
            points = np.hstack((array[:, 0:2], zeros))
        elif "XZ" in columns:
            points = np.hstack((array[:, 0:1], zeros, array[:, 1:2]))
        elif "YZ" in columns:
            points = np.hstack((zeros, array[:, 0:2]))
        else:
            raise ValueError(f"Unsupported point cloud column definition: {columns}")

    intensities = None
    color_values = None

    if "I" in columns and array.shape[1] > color_index:
        intensities = array[:, color_index].astype(np.float64) / 255.0
        color_values = np.repeat(intensities[:, None], 3, axis=1)

    if "RGB" in columns and array.shape[1] >= color_index + 3:
        color_values = array[:, color_index : color_index + 3].astype(np.float64) / 255.0

    if colors is not None:
        color_values = colors.astype(np.float64)

    pcd = PointCloudData(points=points, colors=color_values, intensities=intensities)

    if estimate_normals and len(pcd) > 3:
        estimate_point_normals(pcd, radius=radius, max_nn=max_nn)

    return pcd


def estimate_point_normals(
    pcd: PointCloudData,
    radius: float = 1.0,
    max_nn: int = 30,
    center: Tuple[float, float, float] = (0, 0, 0),
) -> np.ndarray:
    points = pcd.points
    if len(points) == 0:
        pcd.normals = np.zeros((0, 3))
        return pcd.normals

    tree = cKDTree(points)
    normals = np.zeros_like(points)
    center_vec = np.asarray(center, dtype=np.float64)

    for idx, point in enumerate(points):
        neighbours = tree.query_ball_point(point, radius)
        if len(neighbours) < 3:
            normals[idx] = np.array([0.0, 0.0, 1.0])
            continue

        neighbours = neighbours[:max_nn]
        neighbourhood = points[neighbours]
        covariance = np.cov(neighbourhood.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        normal = eigenvectors[:, np.argmin(eigenvalues)]
        if np.dot(normal, point - center_vec) > 0:
            normal = -normal
        normals[idx] = normal / (np.linalg.norm(normal) + 1e-12)

    pcd.normals = normals
    return normals


def transform(
    pcd: PointCloudData,
    transformation: Optional[np.ndarray] = None,
    translate: Optional[Tuple[float, float, float]] = None,
    scale: Optional[float] = None,
    euler_rotate_deg: Optional[Tuple[float, float, float]] = None,
    pivot: Tuple[float, float, float] = (0, 0, 0),
) -> PointCloudData:
    points = pcd.points.copy()

    if transformation is not None:
        transform_matrix = np.asarray(transformation, dtype=np.float64)
        homog = np.column_stack((points, np.ones(points.shape[0])))
        points = (transform_matrix @ homog.T).T[:, :3]

    if translate is not None:
        points += np.asarray(translate, dtype=np.float64)

    if euler_rotate_deg is not None:
        rotation = R.from_euler("xyz", euler_rotate_deg, degrees=True)
        pivot_vec = np.asarray(pivot, dtype=np.float64)
        points = rotation.apply(points - pivot_vec) + pivot_vec

    if scale is not None:
        pivot_vec = np.asarray(pivot, dtype=np.float64)
        points = (points - pivot_vec) * scale + pivot_vec

    result = pcd.copy()
    result.points = points
    return result


def get_transform_vectors(transform_M: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    translation = transform_M[:3, 3]
    rotation_M = np.array(transform_M[:3, :3])
    r = R.from_matrix(rotation_M)
    euler_angles = r.as_euler("xyz", degrees=True)
    return translation, euler_angles


def colormap_pcd(pcd: PointCloudData, cmap: str = "viridis", gamma: float = 2.2) -> PointCloudData:
    if pcd.intensities is not None and len(pcd.intensities) > 0:
        channel = pcd.intensities
    elif pcd.colors is not None and len(pcd.colors) > 0:
        channel = pcd.colors[:, 0]
    else:
        channel = np.zeros(len(pcd))

    channel = channel.astype(np.float64)
    channel -= channel.min()
    if channel.max() > 0:
        channel /= channel.max()

    if gamma != 1:
        channel = np.power(channel, gamma)

    colors = matplotlib.colormaps[cmap](channel)[:, :3]
    return pcd.with_colors(colors)


def downsample(pcd: PointCloudData, voxel_size: float = 0.02) -> PointCloudData:
    if len(pcd) == 0 or voxel_size <= 0:
        return pcd.copy()

    voxel_indices = np.floor(pcd.points / voxel_size).astype(np.int64)
    _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
    return pcd.select_by_index(sorted(unique_idx))


def filter_outliers(pcd: PointCloudData, nb_points: int = 20, radius: float = 0.5) -> PointCloudData:
    if len(pcd) == 0:
        return pcd.copy()

    tree = cKDTree(pcd.points)
    keep: list[int] = []
    for idx, point in enumerate(pcd.points):
        if len(tree.query_ball_point(point, radius)) >= nb_points:
            keep.append(idx)
    return pcd.select_by_index(keep)


def filter_by_reference(original: PointCloudData, reference: PointCloudData, radius: float = 0.02) -> PointCloudData:
    if len(original) == 0 or len(reference) == 0:
        return original.copy()

    tree = cKDTree(reference.points)
    keep: list[int] = []
    for idx, point in enumerate(original.points):
        if tree.query_ball_point(point, radius):
            keep.append(idx)
    return original.select_by_index(keep)


# ----------------------------------------------------------------------------
# Panorama helpers
# ----------------------------------------------------------------------------


def angular_from_cartesian(cartesian_points: np.ndarray) -> np.ndarray:
    cartesian_points = np.asarray(cartesian_points, dtype=np.float64)
    r = np.linalg.norm(cartesian_points, axis=1) + 1e-10
    theta = np.arccos(cartesian_points[:, 2] / r)
    phi = np.arctan2(cartesian_points[:, 1], cartesian_points[:, 0])
    return np.stack([theta, r, phi], axis=1)


def get_sampling_coordinates(
    angular_points: np.ndarray,
    img_shape: Tuple[int, int],
    z_rotate: float = 0,
) -> Tuple[np.ndarray, np.ndarray]:
    image_height, image_width = img_shape

    longitude = angular_points[:, 2] + np.deg2rad(90 + z_rotate)
    longitude = (longitude + 2 * np.pi) % (2 * np.pi)
    image_x = (2 * np.pi - longitude) / (2 * np.pi) * image_width
    image_x = np.clip(np.round(image_x).astype(int), 0, image_width - 1)

    latitude = np.pi / 2 - angular_points[:, 0]
    latitude = (latitude + np.pi / 2) % np.pi
    image_y = (1 - latitude / np.pi) * image_height
    image_y = np.clip(np.round(image_y).astype(int), 0, image_height - 1)

    return image_x, image_y


def angular_lookup(
    angular_points: np.ndarray,
    pano: np.ndarray,
    scale: float = 1.0,
    degrees: bool = False,
    z_rotate: float = 0,
    as_float: bool = False,
) -> np.ndarray:
    if degrees:
        angular_points = np.deg2rad(angular_points)

    image_height, image_width = pano.shape[:2]
    pano_rgb = cv2.cvtColor(pano, cv2.COLOR_BGR2RGB)

    if scale != 1:
        image_height = int(image_height * scale)
        image_width = int(image_height * 2)
        pano_rgb = cv2.resize(pano_rgb, (image_width, image_height), interpolation=cv2.INTER_AREA)

    image_x, image_y = get_sampling_coordinates(angular_points, (image_height, image_width), z_rotate=z_rotate)
    colors = pano_rgb[image_y, image_x]

    if as_float:
        colors = colors.astype(np.float32) / 255.0
    return colors


def get_lidar_pano(pcd: PointCloudData, image_width: int, image_height: int) -> np.ndarray:
    luminance = pcd.colors[:, 0] if pcd.colors is not None else pcd.intensities
    if luminance is None:
        luminance = np.zeros(len(pcd))

    angular_points = angular_from_cartesian(pcd.points)
    image_x, image_y = get_sampling_coordinates(angular_points, (image_height, image_width))

    panorama = np.zeros((image_height, image_width), dtype=np.uint8)
    for idx in range(len(image_x)):
        x, y = image_x[idx], image_y[idx]
        if 0 <= x < image_width and 0 <= y < image_height:
            panorama[y, x] = int(np.clip(luminance[idx] * 255, 0, 255))
    return cv2.medianBlur(panorama, 3)


# ----------------------------------------------------------------------------
# File IO helpers
# ----------------------------------------------------------------------------


def load_pointcloud(
    filepath: str,
    columns: str = "XYZI",
    csv_delimiter: str = ",",
    as_array: bool = False,
) -> PointCloudData | np.ndarray:
    ext = os.path.splitext(filepath)[1].lower()

    if ext in {".pcd", ".ply", ".npz", ".las"}:
        cloud = PyntCloud.from_file(filepath)
        df = cloud.points
        points = df[["x", "y", "z"]].to_numpy(dtype=np.float64)
        colors = None
        if {"red", "green", "blue"}.issubset(df.columns):
            colors = df[["red", "green", "blue"]].to_numpy(dtype=np.float64) / 255.0
        intensities = df["intensity"].to_numpy(dtype=np.float64) if "intensity" in df.columns else None
        result = PointCloudData(points, colors=colors, intensities=intensities)
        return result if not as_array else result.points

    if ext == ".e57":
        e57 = pye57.E57(filepath, mode="r")
        data = e57.read_scan_raw()
        e57.close()
        points = np.column_stack([data["cartesianX"], data["cartesianY"], data["cartesianZ"]])
        colors = np.column_stack([data["colorRed"], data["colorGreen"], data["colorBlue"]]) / 255.0
        result = PointCloudData(points, colors=colors)
        return result if not as_array else result.points

    if ext == ".csv":
        array = np.loadtxt(filepath, delimiter=csv_delimiter)
        return array if as_array else pcd_from_np(array, columns=columns)

    if ext == ".npy":
        array = np.load(filepath)
        return array if as_array else pcd_from_np(array, columns=columns)

    raise ValueError(f"Unsupported file type: {ext}")


def save_pointcloud(
    pcd: PointCloudData | np.ndarray,
    filepath: str,
    ply_ascii: bool = True,
    ply_compression: bool = False,
    csv_delimiter: str = ",",
) -> None:
    directory, _ = os.path.split(filepath)
    if directory:
        os.makedirs(directory, exist_ok=True)

    ext = os.path.splitext(filepath)[1].lower()

    if isinstance(pcd, np.ndarray):
        array = pcd
    else:
        array = None

    if ext == ".ply" or ext == ".pcd":
        cloud = pcd if isinstance(pcd, PointCloudData) else PointCloudData(pcd)
        cloud.to_pyntcloud().to_file(filepath)
        return

    if ext == ".csv":
        if array is None:
            array = PointCloudData(pcd.points if isinstance(pcd, PointCloudData) else pcd).points
        np.savetxt(filepath, array, delimiter=csv_delimiter)
        return

    if ext == ".e57":
        cloud = pcd if isinstance(pcd, PointCloudData) else PointCloudData(pcd)
        e57 = pye57.E57(filepath, mode="w")
        points = cloud.points
        colors = (cloud.colors if cloud.colors is not None else np.zeros_like(points)) * 255.0
        data_raw = {
            "cartesianX": points[:, 0],
            "cartesianY": points[:, 1],
            "cartesianZ": points[:, 2],
            "colorRed": colors[:, 0],
            "colorGreen": colors[:, 1],
            "colorBlue": colors[:, 2],
        }
        e57.write_scan_raw(data_raw)
        e57.close()
        return

    raise ValueError(f"Unsupported file type: {ext}")


def save_pointcloud_threaded(
    pcd: PointCloudData | np.ndarray,
    output_path: str,
    ply_ascii: bool = True,
    ply_compression: bool = False,
    csv_delimiter: str = ",",
) -> None:
    thread = threading.Thread(
        target=save_pointcloud,
        args=(pcd, output_path, ply_ascii, ply_compression, csv_delimiter),
        daemon=False,
    )
    thread.start()
    thread.join()


# ----------------------------------------------------------------------------
# Legacy script support
# ----------------------------------------------------------------------------


if __name__ == "__main__":
    from config import Config

    scan_id = "240824-1230"

    config = Config()
    config.init(scan_id=scan_id)
    config.set(False, "ENABLE_VERTEXCOLOUR")

    pano = cv2.imread(config.pano_path)
    intensity = load_pointcloud(config.intensity_pcd_path)

    lidar_pano = get_lidar_pano(intensity, image_width=2048, image_height=1024)
    cv2.imwrite(os.path.join(config.scan_dir, f"{config.scan_id}_lidar.jpg"), lidar_pano)
    cv2.imshow("Pano from Lidar data", lidar_pano)
    cv2.waitKey(0)
