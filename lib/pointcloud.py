import numpy as np
import open3d as o3d
import pye57
import copy
import cv2
import json
import matplotlib
import threading
import os
from scipy.spatial.transform import Rotation as R
import pickle

try:
    from lib.imu_orientation import (Orientation, leveling_rotation, mounting_matrix, reference_gravity)
except Exception:
    try:
        from imu_orientation import (Orientation, leveling_rotation, mounting_matrix, reference_gravity)
    except Exception:
        Orientation = None


def get_scan_dict(z_angles, angular_list=None, cartesian_list=None,
                  scan_id=None, device_id=None, sensor=None, hardware=None,
                  location=None, author=None, header=None):
    if header is None:
        header = {
            # "creation_date": None,
            # "modification_date": None,
            "scan_id": scan_id,
            "device_id": device_id,
            "sensor": sensor,
            "hardware": hardware,
            "location": location,
            "author": author,
        }
    raw_scan = {
        "header": header,
        "z_angles": z_angles,
        "angular": angular_list,
        "cartesian": cartesian_list
    }
    return raw_scan

def save_raw_scan(path, data):
    """Persist a raw scan as an **open** container next to ``path``.

    Writes ``<stem>.npz`` (arrays) + ``<stem>.json`` (metadata) — replacing the
    Python-locked pickle. The ``path`` argument may be the legacy ``*.pkl`` name
    (``config.raw_path``) or any path; the extension is replaced by ``.npz`` so
    the on-disk schema is always ``<scan_id>_lidar.npz`` + ``.json``. The original
    ``.pkl`` (if present) is left for backward compatibility.
    """
    if not isinstance(data, dict):
        return
    base = os.path.splitext(path)[0]

    cartesian = data.get("cartesian") or []
    quats = data.get("quaternions")
    arrays = {
        "z_angles": np.asarray(data.get("z_angles") or [], dtype=float),
        "cartesian": np.asarray(_stack_cartesian(cartesian)),
    }
    if data.get("angular") is not None:
        arrays["angular"] = np.asarray(data["angular"])
    if quats is not None:
        # DMP quaternions are int16 (unit norm ~16384); keep that dtype rather
        # than letting np.asarray promote the Python ints to int64
        arrays["quaternions"] = np.asarray(quats, dtype=np.int16)

    np.savez_compressed(base + ".npz", **arrays)
    _write_metadata(base + ".json", data, cartesian, arrays)


def _stack_cartesian(cartesian):
    """Stack the per-plane 2D fans into one ``(n_points, n_cols + 1)`` array.

    Each fan's points live in columns 1..n (column 0 is the slice index), so the
    per-plane point counts are recoverable and the ragged list is lossless as a
    single rectangular float array.
    """
    rows = []
    for i, fan in enumerate(cartesian):
        fan = np.asarray(fan, dtype=float)
        if fan.size == 0:
            continue
        rows.append(np.column_stack([np.full(len(fan), i, dtype=float), fan]))
    if not rows:
        return np.empty((0, 1))
    return np.vstack(rows)


def _unstack_cartesian(arr):
    """Inverse of :func:`_stack_cartesian` -> list of per-plane 2D fans."""
    arr = np.asarray(arr)
    if arr.size == 0:
        return []
    idx = arr[:, 0].astype(int)
    return [arr[idx == i, 1:] for i in range(idx.max() + 1)]


def _write_metadata(path, data, cartesian, arrays):
    meta = dict(data.get("header") or {})
    meta["format"] = "pilidar-raw"
    meta["version"] = 1

    # the array schema actually written (dtype + shape), so the sidecar never
    # claims something the .npz does not contain
    schema = {name: f"{arr.dtype} {list(arr.shape)}" for name, arr in arrays.items()}
    if "cartesian" in schema:
        schema["cartesian"] += "  (col 0 = plane index; rows = n_points)"
    meta["arrays"] = schema

    meta["n_planes"] = len(cartesian)
    meta["n_points"] = int(sum(len(np.asarray(c)) for c in cartesian))
    if "z_angles" in data and data["z_angles"] is not None and len(data["z_angles"]):
        za = np.asarray(data["z_angles"], dtype=float)
        meta["angle_min_deg"] = float(za.min())
        meta["angle_max_deg"] = float(za.max())
    if "quaternions" in arrays:
        meta["has_quaternions"] = True
        meta["n_quaternions"] = int(len(arrays["quaternions"]))
        # the IMU also samples the dropped warm-up plane -> one extra leading quat
        meta["warmup_quaternion_extra"] = int(len(arrays["quaternions"]) - len(cartesian))
        # IMU body->gravity orientation at stepper 0 (rotation axis = body +Y)
        try:
            from lib.imu_orientation import scanner_orientation
            rot = scanner_orientation(arrays["quaternions"], data.get("z_angles"))
            if rot is not None:
                meta["scanner_orientation_xyzw"] = [float(v) for v in rot.as_quat()]
        except Exception:
            pass
    else:
        meta["has_quaternions"] = False
    with open(path, "w") as f:
        json.dump(meta, f, indent=2, default=str)


def load_raw_scan(path):
    """Load a raw scan saved by :func:`save_raw_scan` (open NPZ) or legacy pickle.

    Prefers the open container (``<stem>.npz`` + ``.json``); falls back to the
    ``.pkl`` so existing scans keep loading.
    """
    base = os.path.splitext(path)[0]
    npz_path = base + ".npz"
    if os.path.exists(npz_path):
        with np.load(npz_path, allow_pickle=False) as npz:
            arrays = {k: npz[k] for k in npz.files}
        header = {}
        json_path = base + ".json"
        if os.path.exists(json_path):
            try:
                with open(json_path) as f:
                    meta = json.load(f)
                header = {k: v for k, v in meta.items() if k not in ("arrays",)}
            except (OSError, ValueError):
                header = {}
        raw_scan = {
            "header": header,
            "z_angles": arrays.get("z_angles", np.empty(0)).tolist(),
            "angular": arrays.get("angular"),
            "cartesian": _unstack_cartesian(arrays["cartesian"]) if "cartesian" in arrays else [],
        }
        if "quaternions" in arrays:
            raw_scan["quaternions"] = arrays["quaternions"]
        return raw_scan

    # legacy pickle
    with open(path, "rb") as f:
        raw_scan = pickle.load(f)
    return raw_scan


def convert_raw_scan(pkl_path, force=False):
    """Convert a legacy pickled raw scan to the open container (NPZ + JSON).

    Returns ``True`` if converted, ``False`` if an ``.npz`` already exists (and
    ``force`` is not set). The original ``.pkl`` is left in place.
    """
    npz_path = os.path.splitext(pkl_path)[0] + ".npz"
    if os.path.exists(npz_path) and not force:
        return False
    raw = load_raw_scan(pkl_path)
    save_raw_scan(pkl_path, raw)
    return True

def lidar_mount(config):
    """The LiDAR mount ``(position, rotation)`` in scanner space.

    ``position`` = LiDAR origin (mm) in the scanner frame (X left/right, Y forward,
    Z up); ``rotation`` = mount euler (xyz, deg) — the roll about Y (the disc
    normal) is the middle component.
    """
    cal = config.get("3D", "CALIBRATION")
    return tuple(cal["POSITION"]), tuple(cal["ROTATION"])


def process_raw(config, save=True):

    def get_cartesian_list(filepaths, columns="XZI", csv_delimiter=","):
        cartesian_list = []
        for filepath in filepaths:
            points2d = load_pointcloud(filepath, columns, csv_delimiter, as_array=True)
            cartesian_list.append(points2d)
        return cartesian_list

    def postprocess_3D(config, pcd, save=True, imu_up=None):
        # scale pointcloud (mm -> m); the LiDAR mount offsets are already applied
        # during assembly (see merge_2D_points / lidar_mount)
        scene_scale = config.get("3D","SCALE")                                              # mm -> 0.001 m
        if scene_scale !=1:
            pcd = transform(pcd, scale=scene_scale)


        # ANGULAR LOOKUP ("TEXTURING") — uses the SAME projection as the LiDAR pano
        if config.get("ENABLE_VERTEXCOLOUR") and os.path.exists(config.pano_path):
            center, rotation = projection_params(config)
            projected = project_points(np.asarray(pcd.points), center, rotation)
            colors = angular_lookup(angular_from_cartesian(projected),  # angular_points
                                    cv2.imread(config.pano_path),  # pano
                                    scale=config.get("VERTEXCOLOUR","SCALE"))

            pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors))

        else:  # colorize pointcloud by mapping intensities to colormap
            pcd = colormap_pcd(pcd, gamma=1, cmap="viridis")

        # LEVEL THE CLOUD (one rotation at the end, after coloring: the panorama is
        # referenced to the device frame, so level only once the colours are set)
        if imu_up is not None:
            leveling = np.eye(4)
            leveling[:3, :3] = leveling_rotation(imu_up).as_matrix()
            pcd = transform(pcd, transformation=leveling)
            print("\nleveled point cloud to gravity.")

        if save:
            save_pointcloud_threaded(pcd, config.pcd_path, ply_ascii=config.get("3D","ASCII"))


        # FILTER OUTLIER POINTS
        if config.get("ENABLE_FILTERING"):
            low_pcd = downsample(pcd, voxel_size=config.get("FILTERING", "VOXEL_SIZE"))

            nb_points = config.get("FILTERING", "NB_POINTS")
            radius = config.get("FILTERING", "RADIUS")
            filtered_low_pcd = filter_outliers(low_pcd, nb_points=nb_points, radius=radius)

            pcd = filter_by_reference(pcd, filtered_low_pcd, radius=radius)
            if save:
                save_pointcloud_threaded(pcd, config.filtered_pcd_path, ply_ascii=config.get("3D","ASCII"))

        return pcd

    # load raw data dict from pickle file (or the open NPZ container)
    imu_up = None
    raw_base = os.path.splitext(config.raw_path)[0]
    if os.path.exists(raw_base + ".npz") or os.path.exists(config.raw_path):
        # print("lidar.pkl file found:", config.raw_path)
        raw_scan = load_raw_scan(config.raw_path)


        # IMU data -> gravity direction in the cloud frame (for leveling at the end).
        # Use gravity at the sweep's reference attitude, NOT the sweep mean: the
        # mean lies inside the 180 deg gravity half-cone and under-reports the tilt.
        if "quaternions" in raw_scan and config.get("IMU", "LEVEL"):
            g_ref = reference_gravity(raw_scan["quaternions"], z_angles=raw_scan.get("z_angles"))
            imu_up = mounting_matrix(config) @ g_ref
            print(f"\nIMU: {len(raw_scan['quaternions'])} samples; "
                  f"reference gravity in cloud frame = {np.round(imu_up, 3).tolist()}")

        # assemble the 3D cloud. When FAN_SPLIT is enabled this splits each fan
        # into its left/right radial halves and applies the stored half offset so
        # the two halves coincide (removes the "double walls" artifact); otherwise
        # it is the legacy full-fan merge. See lib/fan_split.py.
        from lib.fan_split import assemble_aligned
        array_3D = assemble_aligned(config, raw_scan)


    elif os.path.exists(config.lidar_dir):
        from lib.file_utils import angles_from_filenames

        print(f"Loading raw slices from directory: {config.lidar_dir}")
        filepaths, z_angles = angles_from_filenames(config.lidar_dir, name="plane", ext="npy")
        print(f"{len(filepaths)} files found (min: {min(z_angles):.2f}°, max: {max(z_angles):.2f}°).")

        print("processing 3D planes...")
        cartesian_list = get_cartesian_list(filepaths)

        raw_scan = get_scan_dict(z_angles, cartesian_list=cartesian_list)
        if save:
            save_raw_scan(config.raw_path, raw_scan)

        # fan-split-aware assembly (see lib/fan_split.py); falls back to the
        # legacy full-fan merge when FAN_SPLIT is disabled or no offset is stored.
        from lib.fan_split import assemble_aligned
        array_3D = assemble_aligned(config, raw_scan)
    else:
        raise FileNotFoundError(f"Neither {config.raw_path} nor {config.lidar_dir} found.")


    normal_radius = config.get("3D","NORMAL_RADIUS")  # radius for normal estimation in mm
    pcd = pcd_from_np(array_3D, estimate_normals=True, max_nn=50, radius=normal_radius)
    print("\n2D->3D merge completed.")

    # LiDAR intensity panorama(s) — the two fan halves (each sees every surface
    # from its own side, so rendering them together shows spurious brightness
    # steps) + the merged legacy view, sharing the colouring projection so they
    # line up with the stitched colour panorama.
    if save:
        save_lidar_pano(config, raw_scan)

    pcd = postprocess_3D(config, pcd, save=save, imu_up=imu_up)
    print("\nprocessing 3D completed.")
    return pcd


#------------------------------------------------------------------------------------------------
# Open3D Tensor Geometry

def create_pcd_tensor(points, normals=None, colors=None, intensities=None, distances=None, write_ascii=False):
    # Create a PointCloud Tensor object
    pcdt = o3d.t.geometry.PointCloud()

    dtype_vectors = o3d.core.Dtype.Float64  # Double precision for positions and normals
    dtype_colors = o3d.core.Dtype.UInt8     # Unsigned 8-bit integer for colors

    pcdt.point.positions = o3d.core.Tensor(points, dtype=dtype_vectors)

    # all dtypes are double precision except colors
    if normals is not None:
        pcdt.point.normals = o3d.core.Tensor(normals, dtype=dtype_vectors)
    if colors is not None:
        pcdt.point.colors = o3d.core.Tensor(colors, dtype=dtype_colors)
    if intensities is not None:
        pcdt.point.intensities = o3d.core.Tensor(intensities, dtype=dtype_vectors)
    if distances is not None:
        pcdt.point.distance = o3d.core.Tensor(distances, dtype=dtype_vectors)
    return pcdt

# def visualize(pcd):
#     # Convert tensor pointcloud to legacy and visualize it
#     legacy_pcd = pcd.to_legacy() if isinstance(pcd, o3d.t.geometry.PointCloud) else pcd
#     o3d.visualization.draw([legacy_pcd], show_skybox=False)

# def save_pointcloud(pcd, filename, ply_ascii=False):
#     if isinstance(pcd, o3d.t.geometry.Geometry):
#         o3d.t.io.write_point_cloud(filename, pcd, write_ascii=ply_ascii)
#     else:
#         o3d.io.write_point_cloud(filename, pcd, write_ascii=ply_ascii)


#------------------------------------------------------------------------------------------------
# filtering

def downsample(pcd, voxel_size=0.02):
    downsampled_pcd = pcd.voxel_down_sample(voxel_size)
    return downsampled_pcd

def print_stats(pcd, txt=""):
    bbox = pcd.get_axis_aligned_bounding_box()
    bbox_extent = tuple(round(value, 2) for value in bbox.get_extent())
    print(f"{txt} points: {len(pcd.points)}, bbox_extend: {bbox_extent}")

def filter_outliers(pcd, nb_points=20, radius=0.5):
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
    filtered_pcd = pcd.select_by_index(ind)
    return filtered_pcd

def filter_by_reference(original_pcd, reference_pcd, radius=0.02):
    original_points = np.asarray(original_pcd.points)

    kdtree = o3d.geometry.KDTreeFlann(reference_pcd)

    indices_to_keep = set()
    for i, point in enumerate(original_points):
        [_, idx, _] = kdtree.search_radius_vector_3d(point, radius)
        if len(idx) > 0:
            indices_to_keep.add(i)

    filtered_pcd = original_pcd.select_by_index(list(indices_to_keep))
    return filtered_pcd


#------------------------------------------------------------------------------------------------

def projection_params(config):
    """Return ``(center, rotation)`` for the equirectangular projection.

    ONE place defines the projection used by both the LiDAR intensity panorama
    (calibration) and the vertex colouring, so they can never disagree.

    - ``VERTEXCOLOUR.CENTER_MM`` = [x, y, z] projection origin in the final cloud
      frame (**millimetres**, the same unit as ``3D.CALIBRATION.POSITION``); default
      the origin. Converted to metres internally (the projection runs on the
      meter-scaled cloud).
    - ``VERTEXCOLOUR.ROTATE_DEG`` = [rx, ry, rz] euler (xyz, deg) applied after the
      centre subtraction. This is the single rotation knob (the legacy yaw-only
      ``Z_ROTATE`` was folded into ``ROTATE_DEG[2]`` and removed).

    Rotation meanings for the camera panorama (camera faces **+Y**, up +Z):
      * about **X** — **pitch** (moves the horizon = a vertical shift)
      * about **Y** — **roll** about the camera's facing axis (rotates the view)
      * about **Z** — **yaw / pan** (horizontal shift)

    Returns ``(np.ndarray(3), scipy Rotation)``.
    """
    scale = config.get("3D", "SCALE")  # mm -> m
    center_mm = np.asarray(config.get("VERTEXCOLOUR", "CENTER_MM") or [0.0, 0.0, 0.0], dtype=float)
    center = center_mm * scale  # mm -> m (projection runs on the meter-scaled cloud)
    rotate = config.get("VERTEXCOLOUR", "ROTATE_DEG") or [0.0, 0.0, 0.0]
    return center, R.from_euler("xyz", rotate, degrees=True)


def project_points(points, center, rotation):
    """Translate to the projection centre and rotate into the camera frame."""
    return rotation.apply(np.asarray(points, dtype=float) - center)


def _split_raw_half(raw_scan, positive):
    """Filter a raw scan to the ``x >= 0`` (right) or ``x < 0`` (left) fan half.

    The in-plane radial coordinate ``x`` lives only in the raw 2D fan; after the
    revolve it is mixed into the world horizontal, so the split must be done here
    (before assembly), not on the final cloud.
    """
    fans = []
    for fan in raw_scan["cartesian"]:
        f = np.asarray(fan)
        c = f[:, 0]
        fans.append(f[c >= 0] if positive else f[c < 0])
    return {"z_angles": raw_scan["z_angles"], "cartesian": fans}


def save_lidar_pano(config, raw_scan, width=2048, height=1024, legacy=True):
    """Save the LiDAR **intensity** panorama(s) for a raw scan.

    A sweep wider than 180° sees every surface through **both** in-plane radial
    halves of the fan, so the data holds **two** 360° panoramas at a small
    parallax offset. Rendered together they show spurious brightness steps where
    one half stands closer to the surface than the other, so the halves are
    written **separately** by default:

    * ``<scan_id>_lidar_left.jpg`` / ``_lidar_right.jpg`` — the two fan halves
    * ``<scan_id>_lidar.jpg`` — the merged view (kept for compatibility)

    Each half is assembled on its own (the split is by the sign of the raw
    in-plane radial coordinate, which the revolve would otherwise blur) and
    projected with the shared :func:`projection_params`, so the result lines up
    with the stitched colour panorama. Fed the RAW laser intensity.
    """
    mount = lidar_mount(config)
    scale = config.get("3D", "SCALE")
    center, rotation = projection_params(config)
    out = {}
    for name, positive in (("left", False), ("right", True)):
        p3 = merge_2D_points(_split_raw_half(raw_scan, positive), mount=mount)
        pts = np.asarray(p3[:, :3], dtype=float) * scale
        out[name] = render_lidar_pano(project_points(pts, center, rotation),
                                      p3[:, 3], width, height, center=(0, 0, 0))
        cv2.imwrite(os.path.join(config.scan_dir, f"{config.scan_id}_lidar_{name}.jpg"), out[name])

    p3 = merge_2D_points(raw_scan, mount=mount)
    pts = np.asarray(p3[:, :3], dtype=float) * scale
    out["merged"] = render_lidar_pano(project_points(pts, center, rotation),
                                      p3[:, 3], width, height, center=(0, 0, 0))
    if legacy:
        cv2.imwrite(os.path.join(config.scan_dir, f"{config.scan_id}_lidar.jpg"), out["merged"])
    return out


def get_lidar_pano(pcd, image_width, image_height, intensity=None, center=(0, 0, 0), z_rotate=0):
    """Render a grayscale panorama from a point cloud, coloured by laser intensity.

    ``intensity`` (per point, 0-255 or 0-1) is preferred; when omitted it falls
    back to the red channel of the cloud colours — but a truly independent LiDAR
    panorama must pass the raw intensity, otherwise a colourised cloud makes this
    circular (it would re-project the colour panorama it is trying to align to).

    ``center`` (x, y, z) is the projection origin — the effective camera nodal
    point; it is the main knob the panoramic calibration tunes. ``z_rotate`` (deg)
    rotates the equirectangular map horizontally.
    """
    if intensity is None:
        intensity = np.asarray(pcd.colors)[:, 0]
    return render_lidar_pano(np.asarray(pcd.points), intensity, image_width, image_height,
                             center=center, z_rotate=z_rotate)


def _as_uint8_intensity(intensity):
    """Coerce intensities to uint8, accepting either 0-1 or 0-255 ranges."""
    intensity = np.asarray(intensity, dtype=float)
    if intensity.size and intensity.max() <= 1.0 + 1e-6:
        intensity = intensity * 255.0
    return np.clip(intensity, 0, 255).astype(np.uint8)


def render_lidar_pano(points, intensity, image_width, image_height, center=(0, 0, 0), z_rotate=0):
    """Grayscale equirectangular panorama from 3D ``points`` + per-point ``intensity``.

    Vectorised (a single ``img[y, x] = value`` scatter) so it stays fast on the
    ~2 M-point clouds. Unobserved pixels remain black.
    """
    angular_points = angular_from_cartesian(points, center=center)
    image_x, image_y = get_sampling_coordinates(angular_points, (image_height, image_width), z_rotate=z_rotate)
    panorama = np.zeros((image_height, image_width), dtype=np.uint8)
    panorama[image_y, image_x] = _as_uint8_intensity(intensity)
    return cv2.medianBlur(panorama, 3)


# colorize pointcloud using matplotlib colormap
def colormap_pcd(pcd, cmap="viridis", gamma=2.2):
    r = np.asarray(pcd.colors)[:, 0]
    r_norm = (r - r.min()) / (r.max() - r.min())
    r_corrected = r_norm**(gamma)
    colors = matplotlib.colormaps[cmap](r_corrected)    # Map the normalized color channel to the color map
    if colors.shape[1] == 4:
        colors = colors[:, :3]                          # Remove the alpha channel if present
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

# load point cloud from file (3D: pcd, ply, e57 | 2D: csv, npy), return as pcd object or numpy table
# columns parameter: "XYZ" for 3D, "XZ" for 2D vertical, "I" for intensity or "RGB" for color
def load_pointcloud(filepath, columns="XYZI", csv_delimiter=",", as_tensor=False, as_array=False):
    ext = os.path.splitext(filepath)[1][1:]

    if ext == "pcd" or ext == "ply":
        if as_tensor:
            pcd = o3d.t.io.read_point_cloud(filepath)
        else:
            pcd = o3d.io.read_point_cloud(filepath)

        if as_array:
            return np.column_stack((np.asarray(pcd.points), np.asarray(pcd.colors) * 255))

    elif ext == "e57":
        # Create an E57 object with read mode and read data
        e57 = pye57.E57(filepath, mode='r')
        data = e57.read_scan_raw()
        e57.close()

        # Create a numpy array with the point cloud data
        points = np.column_stack([data["cartesianX"], data["cartesianY"], data["cartesianZ"]])
        colors = np.column_stack([data["colorRed"],   data["colorGreen"], data["colorBlue"]])

        if as_array:
            return np.column_stack((np.asarray(points), np.asarray(colors)))

        # Normalize the color values to the range [0, 1]
        colors = colors / 255

        # Create an open3d point cloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

    elif ext == "csv":
        array = np.loadtxt(filepath, delimiter=csv_delimiter)
        if as_array:
            return array

        pcd = pcd_from_np(array, columns=columns)

    elif ext == 'npy':
        array = np.load(filepath)
        if as_array:
            return array

        pcd = pcd_from_np(array, columns=columns)

    else:
        raise ValueError("Unsupported file type: " + ext)

    return pcd

# export point cloud to file (pcd, ply, e57, csv)
def save_pointcloud(pcd, filepath, ply_ascii=False, ply_compression=True, csv_delimiter=","):
    # Create the directory if it does not exist
    directory, filename = os.path.split(filepath)
    os.makedirs(directory, exist_ok=True)

    # Get the file extension
    ext = os.path.splitext(filename)[1][1:]

    if ext == "pcd":
        if isinstance(pcd, o3d.t.geometry.Geometry):
            o3d.t.io.write_point_cloud(filepath, pcd)
        else:
            o3d.io.write_point_cloud(filename=filepath, pointcloud=pcd)

    elif ext == "ply":
        if isinstance(pcd, o3d.t.geometry.Geometry):
            o3d.t.io.write_point_cloud(filepath, pcd, write_ascii=ply_ascii, compressed=ply_compression)
        else:
            o3d.io.write_point_cloud(filename=filepath, pointcloud=pcd, write_ascii=ply_ascii, compressed=ply_compression)

    # TODO: add support for exporting intensity and RGB colors
    elif ext == "csv":
        if not isinstance(pcd, np.ndarray):
            array = np.asarray(pcd.points)
        np.savetxt(filepath, array, delimiter=csv_delimiter)

    elif ext == "e57":
        # if a single point cloud is provided, convert it to a list
        if isinstance(pcd, list):
            pcd_list = pcd
        elif isinstance(pcd, o3d.geometry.PointCloud):
            pcd_list = [pcd]

        # Create an E57 object with write mode
        e57 = pye57.E57(filepath, mode='w')

        # Write each point cloud to the E57 file as a separate scan
        for pcd in pcd_list:
            # Convert open3d point cloud to numpy array
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors) * 255

            # Create a dictionary with keys for each coordinate and color
            data_raw = {
                "cartesianX": points[:, 0],
                "cartesianY": points[:, 1],
                "cartesianZ": points[:, 2],
                "colorRed"  : colors[:, 0],
                "colorGreen": colors[:, 1],
                "colorBlue" : colors[:, 2]}

            # Write the point cloud data to the E57 file
            e57.write_scan_raw(data_raw)
        e57.close()

    print("\nexport completed.")

def save_pointcloud_threaded(pcd, output_path, ply_ascii=False, ply_compression=True, csv_delimiter=","):
    export_thread = threading.Thread(target=save_pointcloud, args=(pcd, output_path, ply_ascii, ply_compression, csv_delimiter))
    export_thread.start()

# Remove rows with NaN values from a numpy array
def remove_NaN(array):
    return array[~np.isnan(array).any(axis=1)]

# estimate normals for a point cloud
def estimate_point_normals(pcd, radius=1, max_nn=30, center=(0,0,0)):
    KD_search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
    pcd.estimate_normals(search_param=KD_search_param)
    pcd.orient_normals_towards_camera_location(camera_location=center)
    return pcd

def merge_2D_points(raw_scan, z_step=1, ccw=False, mount=None,
                    position_offset=(0, 0, 0), angle_offset=0, up_vector=(0, 0, 1)):
    """Assemble the 3D cloud from the per-plane 2D fans.

    The scanner's object frame has **+Z = up (rotation axis)**, **+Y = forward**
    (the camera look direction) and X = left/right. Each fan is a vertical plane
    whose normal is ±Y; the LiDAR is mounted on the mount axis (Y) at a position
    offset, rolled about Y by ``rotation[1]``.

    ``mount`` is ``(position, rotation)`` in scanner space: the LiDAR origin
    (mm) and its mount rotation (euler xyz, deg). Defaults use ``position_offset``
    (a pure Y translation) and ``angle_offset`` (a roll about Y) for callers that
    pass the raw values.
    """
    z_angles = raw_scan["z_angles"]
    cartesian_list = raw_scan["cartesian"]

    if mount is not None:
        position_offset, angle_offset = mount

    # Mount orientation about the LiDAR origin. A scalar keeps the older
    # convention (roll about Y = the disc normal); a 3-vector is a full euler-xyz
    # rotation (deg), so a LiDAR unit that sits off-square (pitch, roll AND yaw)
    # can be described in scanner space.
    if np.ndim(angle_offset) == 0:
        mount_rotation = R.from_rotvec(np.radians(float(angle_offset)) * np.array([0.0, 1.0, 0.0]))
    else:
        mount_rotation = R.from_euler("xyz", angle_offset, degrees=True)

    # accumulate 3D slices in a list for fast single concatenation
    pointcloud_chunks = []
    z_angle = 0

    for i, points2d in enumerate(cartesian_list):
        # 2D (x, y) -> 3D (x, 0, y): the 2D length (y) becomes the height (Z),
        # standing the fan upright about X; the plane lies in X-Z, its normal is Y.
        points3d = np.insert(points2d, 1, values=0, axis=1)

        # Use the corresponding angle from the list if provided, otherwise use the fixed angle increment
        if z_angles is not None:
            z_angle = z_angles[i]
        else:
            if ccw:
                z_angle -= z_step
            else:
                z_angle += z_step

        # LiDAR mount orientation (pitch/roll/yaw about the LiDAR origin)
        if points3d.shape[1] == 4:
            points3d = np.column_stack((mount_rotation.apply(points3d[:, :3]), points3d[:, 3]))
        else:
            points3d = mount_rotation.apply(points3d[:, :3])

        # revolve around the Z-axis (up_vector) by its angle from filename
        points3d = rotate_3D(points3d, -z_angle, translation_vector=position_offset, rotation_axis=np.array(up_vector))
        pointcloud_chunks.append(points3d)

    if pointcloud_chunks:
        pointcloud = np.vstack(pointcloud_chunks)
    else:
        pointcloud = np.empty((0, 4))

    # Remove rows with NaN values
    pointcloud = remove_NaN(pointcloud)
    return pointcloud

# convert numpy array to open3d point cloud
# supports 2D and 3D points, intensity and RGB colors or pcd objects (pcd.points and pcd.colors)
def pcd_from_np(array, columns="XYZI", estimate_normals=True, colors=None, radius=10, max_nn=30):
    pcd = o3d.geometry.PointCloud()

    # Convert the pointcloud and colors to numpy arrays if they are not already
    # TODO: merged from another version -> check if it's still valid
    if not isinstance(array, np.ndarray):
        array = np.asarray(array)
    if colors is not None and not isinstance(colors, np.ndarray):
        colors = np.asarray(colors)

    columns = columns.upper()
    zeros = np.zeros((array.shape[0], 1))

    # 3D points
    if "XYZ" in columns:
        points = array[:, 0:3]
        pcd.points = o3d.utility.Vector3dVector(points)
        color_i = 3

     # 2D points
    else:
        color_i = 2
        if "XY" in columns:
            pcd.points = o3d.utility.Vector3dVector(np.hstack((array[:, 0:2], zeros)))
        elif "XZ" in columns:
            pcd.points = o3d.utility.Vector3dVector(np.hstack((array[:, 0:1], zeros, array[:, 1:2])))
        elif "YZ" in columns:
            pcd.points = o3d.utility.Vector3dVector(np.hstack((zeros, array[:, 0:2])))
        else:
            raise ValueError("Unsupported point cloud type: " + type(array))

    if estimate_normals:
        pcd = estimate_point_normals(pcd, radius=radius, max_nn=max_nn)

    # colors
    if "I" in columns:  # ext == "XYZI" -> array.shape[1] == 4:
        intensities = array[:, color_i] / 255  # normalize intensity values

        # convert intensity to RGBA color
        pcd.colors = o3d.utility.Vector3dVector(np.column_stack([intensities, intensities, intensities]))

    elif "RGB" in columns:
        pcd.colors = o3d.utility.Vector3dVector(array[:, color_i:color_i+3] / 255)

    elif colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


#------------------------------------------------------------------------------------------------
# Transformation functions

def rotate_3D(points3d, rotation_degrees, translation_vector=(0,0,0), rotation_axis=(0,0,1)):
    rotation_axis = np.array(rotation_axis)

    rotation_radians = np.radians(rotation_degrees)
    rotation_vector = rotation_radians * rotation_axis
    rotation = R.from_rotvec(rotation_vector)
    rotation_matrix = rotation.as_matrix()
    pcd = o3d.geometry.PointCloud()

    pcd.points = o3d.utility.Vector3dVector(points3d[:, 0:3])  # column 4 are intensities

    if points3d.shape[1] == 4:
        intensities = points3d[:, 3]
        colors = np.stack([intensities]*3, axis=-1)
        pcd.colors = o3d.utility.Vector3dVector(colors)

    # Perform translation before rotation, adjusting Y based on translation and initial Z
    pcd.translate(translation_vector)
    # pcd.points[:, 1] += pcd.points[:, 2] * translation_vector[1]  # Update Y based on translation and Z

    # Rotate the point cloud using the rotation matrix
    pcd.rotate(rotation_matrix, center=(0, 0, 0))

    # Convert the point cloud back to a NumPy array
    result_points = np.asarray(pcd.points)

    # Reattach intensity column if provided
    if points3d.shape[1] == 4:
        result_points = np.column_stack((result_points, points3d[:, 3]))

    return result_points

# def rotate_3D_np(points3d, rotation_axis, rotation_degrees):
#     """ Rotate a vector v about axis by taking the component of v perpendicular to axis,
#     rotating it theta in the plane perpendicular to axis,
#     then add the component of v parallel to axis.

#     Let a be a unit vector along an axis axis. Then a = axis/norm(axis).
#     Let A = I x a, the cross product of a with an identity matrix I.
#     Then exp(theta,A) is the rotation matrix.
#     Finally, dotting the rotation matrix with the vector will rotate the vector.

#     https://www.kite.com/python/answers/how-to-rotate-a-3d-vector-about-an-axis-in-python
#     https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
#     """
#     rotation_radians = np.radians(rotation_degrees)
#     rotation_vector = rotation_radians * rotation_axis
#     rotation = R.from_rotvec(rotation_vector)

#     # rotation_matrix = rotation.as_matrix()
#     # return np.dot(points3d[:, 0:3], rotation_matrix.T)

#     result_points = points3d.copy()
#     if points3d.shape[1] == 4:
#         # remove intensity column
#         points3d = np.delete(points3d, -1, axis=1)
#     # apply rotation to each point
#     for i, point in enumerate(points3d):
#         result_points[i][0:3] = rotation.apply(point)
#     return result_points

# extract translation and rotation vectors from transformation matrix
def get_transform_vectors(transform_M):
    # Extract translation (top-right 3x1 sub-matrix)
    translation = transform_M[:3, 3]

    # Extract rotation (top-left 3x3 sub-matrix), make a copy to avoid read only error
    rotation_M = np.array(transform_M[:3, :3])
    # Convert rotation matrix to Euler angles
    r = R.from_matrix(rotation_M)
    euler_angles = r.as_euler('xyz', degrees=True)

    return translation, euler_angles

# apply translation and rotation to open3d point cloud
def transform(pcd, transformation=None, translate=None, scale=None, euler_rotate_deg=None, pivot=(0,0,0)):
    pcd_temp = copy.deepcopy(pcd)

    if transformation is not None:
        pcd_temp.transform(transformation)

    if translate is not None:
        pcd_temp.translate(translate)

    if euler_rotate_deg is not None:
        euler_rotate_rad = np.deg2rad(euler_rotate_deg)
        rotation_matrix = pcd_temp.get_rotation_matrix_from_xyz(euler_rotate_rad)
        pcd_temp.rotate(rotation_matrix, center=pivot)

    if scale is not None:
        pcd_temp.scale(scale, center=pivot)

    return pcd_temp


#------------------------------------------------------------------------------------------------
# pointcloud Lookup

def angular_from_cartesian(cartesian_points, center=(0, 0, 0)):
    """Spherical coordinates ``(theta, r, phi)`` of each point about ``center``.

    ``center`` is the projection origin (the effective fisheye/equirect nodal
    point). Moving it changes both the direction to each point and their ordering,
    which is exactly what panoramic extrinsic calibration solves for.
    """
    points = np.asarray(cartesian_points, dtype=float)[:, :3] - np.asarray(center, dtype=float)
    r = np.sqrt(np.sum(points**2, axis=1)) + 1e-10  # hack: avoid division by zero
    theta = np.arccos(np.clip(points[:, 2] / r, -1.0, 1.0))
    phi = np.arctan2(points[:, 1], points[:, 0])
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
    '''
    regenerate LiDAR intensity panoramas (`<scan_id>_lidar_left.jpg` /
    `_lidar_right.jpg` for the two fan halves + the merged `<scan_id>_lidar.jpg`)
    from the raw scan, using the SAME projection as the vertex colouring.
    '''

    from config import Config

    scan_id = "261012-0037"

    config = Config()
    config.init(scan_id=scan_id)

    raw_scan = load_raw_scan(config.raw_path)
    # fan-frame mm; the mount offsets and projection are applied inside
    save_lidar_pano(config, raw_scan)               # raw laser intensity
    # cv2.imshow("Pano from Lidar data", lidar_pano); cv2.waitKey(0)
