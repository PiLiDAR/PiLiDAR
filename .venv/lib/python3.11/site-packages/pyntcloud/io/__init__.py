from pyntcloud.io.open3d import from_open3d, to_open3d
from pyntcloud.io.pyvista import from_pyvista, to_pyvista
from .ascii import read_ascii, write_ascii
from .bin import read_bin, write_bin
from .las import read_las
from .npz import read_npz, write_npz
from .obj import read_obj, write_obj
from .ply import read_ply, write_ply
from .off import read_off
from .pcd import read_pcd

FROM_FILE = {
    "ASC": read_ascii,
    "BIN": read_bin,
    "CSV": read_ascii,
    "LAS": read_las,
    "LAZ": read_las,
    "NPZ": read_npz,
    "OBJ": read_obj,
    "OFF": read_off,
    "PCD": read_pcd,
    "PLY": read_ply,
    "PTS": read_ascii,
    "TXT": read_ascii,
    "XYZ": read_ascii,
}
FROM_INSTANCE = {
    "PYVISTA": from_pyvista,
    "OPEN3D": from_open3d
}

TO_FILE = {
    "ASC": write_ascii,
    "BIN": write_bin,
    "CSV": write_ascii,
    "NPZ": write_npz,
    "OBJ": write_obj,
    "PLY": write_ply,
    "PTS": write_ascii,
    "TXT": write_ascii,
    "XYZ": write_ascii,
}
TO_INSTANCE = {
    "PYVISTA": to_pyvista,
    "OPEN3D": to_open3d
}
