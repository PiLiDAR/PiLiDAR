"""
Experimental: run the Hugin feature-matching + optimization (refine) pipeline
on an existing scan and compare against the ORIGINAL template-only stitch.

Usage:
    python3 refine_experiment.py [scan_id] [stage]

stages:
    all      (default) full pipeline: pto_gen -> template -> cpfind -> cpclean
             -> linefind -> pto_var -> autooptimiser -> hugin_executor
    cps      stop after cpclean (inspect control points / timing)
    stitch   assume project.pto already optimized; just stitch

Hugin tools are invoked through Flatpak (net.sourceforge.Hugin).
"""
import os
import sys
import subprocess

HUGIN_APP = "net.sourceforge.Hugin"
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # repo root (parent of experiments/)
TEMPLATE = os.path.join(PROJECT_ROOT, "hugin", "template_4_AEB2-2.pto")


def hugin(tool, *args, check=True):
    cmd = ["flatpak", "run", "--user", f"--command={tool}", HUGIN_APP, *args]
    print(f"\n>>> {tool} {' '.join(args)}", flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True)
    tail = (r.stderr.strip().splitlines() or [""])[-3:]
    for line in tail:
        print(f"    {line}")
    if check and r.returncode != 0:
        raise RuntimeError(f"{tool} failed (rc={r.returncode})")
    return r


def build_imglist(img_dir):
    """Order images by angle then EV, matching the template's positional layout."""
    import glob
    found = glob.glob(os.path.join(img_dir, "image_*.jpg"))
    # sort angles NUMERICALLY (0.0, 89.99, 179.97, 269.99) to match template yaw priors
    angle_set = sorted({os.path.basename(p).split("_")[1] for p in found},
                       key=float)
    evs = [-4, -2, 0, 2, 4]
    imglist = []
    for a in angle_set:
        for e in evs:
            p = os.path.join(img_dir, f"image_{a}_{e}.jpg")
            if os.path.exists(p):
                imglist.append(p)
    return imglist


def main():
    scan_id = sys.argv[1] if len(sys.argv) > 1 else "240827-1322"
    stage = sys.argv[2] if len(sys.argv) > 2 else "all"

    base = os.path.join(PROJECT_ROOT, "scans", scan_id)
    img_dir = os.path.join(base, "img")
    work = os.path.join(base, "tmp_refine")
    os.makedirs(work, exist_ok=True)
    pto = os.path.join(work, "project.pto")

    imglist = build_imglist(img_dir)
    print(f"scan={scan_id}  images={len(imglist)}")

    if stage in ("all", "cps"):
        hugin("pto_gen", "--projection=2", "--fov=360", "-o", pto, *imglist)
        hugin("pto_template", f"--output={pto}", f"--template={TEMPLATE}", pto)
        hugin("cpfind", "-o", pto, "--multirow", pto)
        hugin("cpclean", "-o", pto, pto)
        if stage == "cps":
            print("\n[cps] stopping after cpclean. project at:", pto)
            return

    # line features (verticals) help constrain roll
    hugin("linefind", "-o", pto, pto)

    # Constrained optimization: refine rotations + shared lens distortion,
    # anchor image 0 (no translations -> rotational pano, no parallax).
    hugin("pto_var", "-o", pto, "--opt",
          "r,p,y,Ra,Rb,Rc,Rd,Re,a,b,c,d,e,!r0,!p0,!y0", pto)
    hugin("autooptimiser", "-n", "-o", pto, pto)

    out_prefix = os.path.join(base, f"{scan_id}_blended_fused_REFINED")
    hugin("hugin_executor", "--stitching", f"--prefix={out_prefix}", pto)
    print("\nDONE. Output prefix:", out_prefix)


if __name__ == "__main__":
    main()
