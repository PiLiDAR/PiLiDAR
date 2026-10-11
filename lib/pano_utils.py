"""
http://paulbourke.net/dome/dualfish2sphere/diagram.pdf

Hugin CLI parameters from StereoPi.sh :
    pto_gen --projection=2 --fov=360 -o ./tmp/project.pto $1 $2
    pto_template --output=./tmp/project.pto --template=template.pto ./tmp/project.pto
    hugin_executor --stitching --prefix=$RESULT_NAME ./tmp/project.pto

additional parameters (used by hugin_refine):
    pto_lensstack -o project1.pto --new-lens i1 project.pto
    cpfind -o project.pto --multirow project.pto
    cpclean -o project.pto project.pto
    linefind -o project.pto project.pto
    # refine rotations + shared distortion; lock translations (rotational pano);
    # anchor image 0 fully. No TrX/TrY/TrZ -> no parallax.
    pto_var -o project.pto --opt r,p,y,Ra,Rb,Rc,Rd,Re,a,b,c,d,e,!r0,!p0,!y0 project.pto
    autooptimiser -n -o project.pto project.pto
    pano_modify  --projection=1 --fov=AUTO --center --canvas=AUTO --crop=AUTO -o project.pto project.pto

Hugin tools are invoked via _run_hugin(), which prefers a system-wide install
on PATH and falls back to the Flatpak build (net.sourceforge.Hugin).
"""

import math
import re
import shutil
import numpy as np
import subprocess
import os


# Hugin tools may be installed system-wide (hugin-tools) or via Flatpak.
# Detect once and build the correct command prefix for each tool.
_HUGIN_FLATPAK_APP = "net.sourceforge.Hugin"


def _hugin_prefix(tool):
    """Return the command prefix to invoke a Hugin CLI tool.

    Prefers a system-wide install on PATH; falls back to the Flatpak build
    (``flatpak run --user --command=<tool> net.sourceforge.Hugin``).
    """
    if shutil.which(tool):
        return [tool]
    if shutil.which("flatpak"):
        return ["flatpak", "run", "--user", f"--command={tool}", _HUGIN_FLATPAK_APP]
    raise RuntimeError(
        f"Hugin tool '{tool}' not found on PATH and no Flatpak install detected. "
        "Install hugin-tools or the net.sourceforge.Hugin Flatpak."
    )


def _run_hugin(tool, *args, check=True):
    """Run a Hugin CLI tool with the detected prefix and return the CompletedProcess."""
    cmd = _hugin_prefix(tool) + list(args)
    result = subprocess.run(cmd, capture_output=True, text=True)
    if check and result.returncode != 0:
        tail = (result.stderr.strip().splitlines() or [""])[-5:]
        raise RuntimeError(f"{tool} failed (rc={result.returncode}):\n" + "\n".join(tail))
    return result


def _ev_of(path):
    """Extract the EV offset from a bracket filename like ``image_89.99_-4.jpg`` -> -4."""
    stem = os.path.basename(path).rsplit('.', 1)[0]
    return int(stem.rsplit('_', 1)[-1])


def _template_ev_order(template_path):
    """Return the per-angle EV offset sequence encoded in a Hugin template.

    ``pto_template`` maps template image *i* to project image *i* by position, so
    the project's ``imglist`` must follow the same within-angle EV order as the
    template (e.g. ``[0, 2, -2, 4, -4]``). The first repeated EV marks the end of
    the first angle group, so we return everything before it.
    """
    evs = []
    with open(template_path) as f:
        for line in f:
            if line.startswith('i '):
                m = re.search(r'n"([^"]+)"', line)
                if m:
                    try:
                        evs.append(_ev_of(m.group(1)))
                    except (ValueError, IndexError):
                        pass
    if not evs:
        return None
    seen = set()
    for i, ev in enumerate(evs):
        if ev in seen:
            return evs[:i]
        seen.add(ev)
    return evs


def _reorder_imglist(imglist, ev_order):
    """Reorder ``imglist`` so each angle group follows ``ev_order``.

    ``imglist`` is grouped by angle (contiguous blocks of ``len(ev_order)`` images
    in capture order). Within each block we re-sort by the template's EV sequence
    so positional template mapping lands priors on the correct frames.
    """
    m = len(ev_order)
    reordered = []
    for start in range(0, len(imglist), m):
        group = imglist[start:start + m]
        ev_to_img = {}
        for p in group:
            try:
                ev_to_img.setdefault(_ev_of(p), p)
            except (ValueError, IndexError):
                reordered.append(p)  # unparseable EV: keep as-is
        for ev in ev_order:
            if ev in ev_to_img:
                reordered.append(ev_to_img.pop(ev))
        reordered.extend(ev_to_img.values())  # any EVs not in the template order
    return reordered


def hugin_modify(pto_path, new_project_path, width=6800):
    # Load the project file(pto_path)
    with open(pto_path, 'r') as file:
        lines = file.readlines()

    # search for first line that starts with 'p ' -> 'p f2 w6000 h3000 v360 ...'
    for i, line in enumerate(lines):

        if line.startswith("p "):
            # replace w and h arguments
            words = line.split(" ")
            for j, word in enumerate(words):
                if word.startswith("w"):
                    words[j] = f"w{width}"
                elif word.startswith("h"):
                    words[j] = f"h{width//2}"

            lines[i] = " ".join(words)
            break

    # Save the project file at new_project_path
    with open(new_project_path, 'w') as file:
        file.writelines(lines)


def hugin_stitch(config, nice=19):
    pto_path = config.pto_path
    output_path  = os.path.join(config.scan_dir, f'{config.scan_id}.jpg')

    # Match the template's per-angle EV order so positional priors land on the
    # correct frames (capture order is [-4,-2,0,2,4]; templates are [0,2,-2,4,-4]).
    ev_order = _template_ev_order(config.template_path)
    if ev_order:
        config.imglist = _reorder_imglist(config.imglist, ev_order)

    # create Hugin project
    _run_hugin('pto_gen', '--projection=2', '--fov=360', '-o', pto_path, *config.imglist)  # *list -> unpack

    # apply template
    _run_hugin('pto_template', f'--output={pto_path}', f'--template={config.template_path}', pto_path)

    # modify resolution of the temporary project file
    hugin_modify(pto_path, pto_path, width=config.get("PANO", "PANO_WIDTH"))

    # start stitching as non-blocking thread with low priority
    if config.get("ENABLE_PANO"):
        cmd_string = ['nice', '-n', str(nice), *_hugin_prefix('hugin_executor'), '--stitching', f'--prefix={output_path}', pto_path]
        process = subprocess.Popen(cmd_string)

        # Wait for the process to finish
        process.wait()

        # check returncode if stitching was successful
        if process.returncode != 0:
            raise Exception(f"Command failed with return code {process.returncode}")

    return output_path


def _count_control_points(pto_path):
    """Count control-point lines (``c ...``) in a Hugin project file."""
    count = 0
    with open(pto_path, 'r') as f:
        for line in f:
            if line.startswith("c "):
                count += 1
    return count


def hugin_refine(config, nice=19, min_control_points=5):
    """Stitch with feature matching + constrained optimization.

    Pipeline: pto_gen -> pto_template -> cpfind -> cpclean -> linefind
              -> pto_var -> autooptimiser -> hugin_executor.

    The template provides coarse priors (yaw/pitch/roll + lens distortion);
    control points then let Hugin refine the per-mount extrinsics and shared
    distortion. If too few control points are found (e.g. a textureless scene),
    optimization is skipped and the template priors are used as-is.
    """
    pto_path = config.pto_path
    output_path = os.path.join(config.scan_dir, f'{config.scan_id}.jpg')

    # Match the template's per-angle EV order so positional priors land on the
    # correct frames (capture order is [-4,-2,0,2,4]; templates are [0,2,-2,4,-4]).
    ev_order = _template_ev_order(config.template_path)
    if ev_order:
        config.imglist = _reorder_imglist(config.imglist, ev_order)

    # Create Hugin project
    _run_hugin('pto_gen', '--projection=2', '--fov=360', '-o', pto_path, *config.imglist)

    # Apply template (coarse priors)
    _run_hugin('pto_template', f'--output={pto_path}', f'--template={config.template_path}', pto_path)

    # Add + clean control points
    _run_hugin('cpfind', '-o', pto_path, '--multirow', pto_path)
    _run_hugin('cpclean', '-o', pto_path, pto_path)

    n_cp = _count_control_points(pto_path)
    print(f"[hugin_refine] {n_cp} control points after cpclean")

    if n_cp >= min_control_points:
        # Find line features (verticals) to help constrain roll
        _run_hugin('linefind', '-o', pto_path, pto_path)

        # Constrained optimization: refine rotations + shared lens distortion.
        # - No TrX/TrY/TrZ: a 360° pano rotates about a common optical center,
        #   so translations stay locked (avoids parallax).
        # - Image 0 fully anchored (!r0,!p0,!y0) as the coordinate reference.
        # - Distortion (Ra..Re,a..e) optimized on all images so they converge
        #   consistently instead of one frame carrying it alone.
        _run_hugin('pto_var', '-o', pto_path, '--opt',
                   'r,p,y,Ra,Rb,Rc,Rd,Re,a,b,c,d,e,!r0,!p0,!y0', pto_path)
        _run_hugin('autooptimiser', '-n', '-o', pto_path, pto_path)
    else:
        print(f"[hugin_refine] WARNING: only {n_cp} control points "
              f"(< {min_control_points}); skipping optimization, using template priors.")

    # Modify resolution of the temporary project file
    hugin_modify(pto_path, pto_path, width=config.get("PANO", "PANO_WIDTH"))

    # Start stitching as non-blocking thread with low priority
    if config.get("ENABLE_PANO"):
        cmd_string = ['nice', '-n', str(nice), *_hugin_prefix('hugin_executor'), '--stitching', f'--prefix={output_path}', pto_path]
        process = subprocess.Popen(cmd_string)

        # Wait for the process to finish
        process.wait()

        # Check return code if stitching was successful
        if process.returncode != 0:
            raise Exception(f"Command failed with return code {process.returncode}")

    return output_path


def sample_color(img, uv, normalize_color=False):
    longitude, latitude = uv

    if isinstance(longitude, float):
        h, w, c = img.shape
        longitude = int(longitude * w)
        latitude = int(latitude * h)

    # print(longitude, latitude)
    color = img[latitude, longitude]

    if normalize_color:  # convert uint8 to float
        color = color / 255

    return tuple(color)


def vector_to_longlat(point3d):
    Px, Py, Pz = point3d
    longitude = math.atan2(-Py, Px)
    latitude = math.atan2(Pz, math.sqrt(Px**2 + Py**2))
    return longitude, latitude  # CW


def longlat_to_vector(longlat):
    longitude, latitude = longlat
    Px = math.cos(latitude) * math.cos(longitude)
    Py = math.cos(latitude) * math.sin(longitude)
    Pz = math.sin(latitude)
    return Px, Py, Pz


def longlat_to_spherical(longlat):  # horizontal, vertical
    longitude, latitude = longlat
    u = longitude / math.pi / 2
    v = 2 * latitude / math.pi
    return u, v


def spherical_to_longlat(uv):
    u, v = uv
    longitude = u * math.pi
    latitude = v * math.pi / 2
    return longitude, latitude


def deg2rad(rad_list):
    # deg_list = [math.degrees(item) for item in rad_list]  # list comprehension
    return tuple(map(math.degrees, rad_list))


def rad2deg(deg_list):
    return tuple(map(math.radians, deg_list))


def mod(angles, mod=math.pi*2):
    angles = list(map(math.fmod, angles, [mod] * len(angles)))
    return angles


def polar2cartesian(angles, distances, offset_angle=math.pi/2):
    """
    https://stackoverflow.com/questions/20924085/python-conversion-between-coordinates

    converts list of polar coordinates into cartesian (x/y)
    :param angles: list of radians
    :param distances: list of distances
    :param offset_angle: offset radian
    :return: two lists of x and y coordinates
    """
    angles = list(np.array(angles) + offset_angle)
    x_list = distances * -np.cos(angles)
    y_list = distances * np.sin(angles)
    return x_list, y_list


def cartesian2polar(x_list, y_list, offset_angle=math.pi/2):
    distances = np.sqrt(x_list**2 + y_list**2)
    angles = -np.arctan2(y_list, x_list)
    angles = list(np.array(angles) + offset_angle)
    return angles, distances


if __name__ == "__main__":
    from file_utils import list_files
    from config import Config

    config = Config()
    config.init(scan_id="240915-2326")


    # Get all jpg files in the directory
    files = list_files(config.img_dir, filter="_0.jpg")
    print(len(files), "images found.")

    config.imglist = files[0:4]
    print("imglist:", config.imglist)

    config.template_path = f'{config.base_dir}/hugin/template_4.pto'

    hugin_stitch(config)


    # files = list_files(config.img_dir)
    # print(len(files), "images found.")
    # print("imglist:", config.imglist)

    # config.template_path = f'{config.base_dir}/hugin/template_4_AEB1-2.pto'
    # hugin_refine(config)
