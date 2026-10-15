"""Generate a self-contained HTML page to pick RGB<->LiDAR panorama point pairs.

Works where a cv2 GUI cannot (e.g. WSL / headless): open the page in any browser,
click a feature in the RGB panorama then its match in the LiDAR panorama. Pairs are
listed with their pixel coordinates in the ORIGINAL panoramas, ready to paste back.

    ~/venv/bin/python experiments/pano_picker.py 261015-1839
    # then open the printed file:// path in a browser
"""
import argparse
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>Pano point picker — __SCAN__</title>
<style>
  body {{ font: 14px/1.4 system-ui, sans-serif; margin: 12px; background:#111; color:#eee; }}
  h3 {{ margin: 4px 0; font-size: 15px; }}
  #wrap {{ display: flex; gap: 12px; flex-wrap: wrap; }}
  .pane {{ flex: 1 1 480px; min-width: 320px; }}
  canvas {{ width: 100%; height: auto; background:#000; cursor: crosshair; display:block; }}
  .active {{ outline: 2px solid #4ea1ff; }}
  code, textarea {{ font-family: ui-monospace, monospace; }}
  textarea {{ width: 100%; height: 120px; background:#000; color:#0f0; border:1px solid #444; }}
  table {{ border-collapse: collapse; margin: 8px 0; }}
  td, th {{ border:1px solid #444; padding: 2px 8px; }}
</style></head>
<body>
<h2>Pano point picker — scan __SCAN__</h2>
<p>Click a feature in the <b>RGB</b> panorama, then its match in the <b>LiDAR</b>
panorama. Repeat for <b>3+</b> well-spread pairs (different azimuth <i>and</i>
elevation). Avoid the floor right under the scanner (parallax).
Keys: <code>u</code> undo, <code>c</code> clear.</p>
<div id="wrap">
  <div class="pane"><h3>RGB panorama (camera) &nbsp;<span id="ws">…</span></h3>
    <canvas id="cR"></canvas></div>
  <div class="pane"><h3>LiDAR intensity &nbsp;<span id="wl">…</span></h3>
    <canvas id="cL"></canvas></div>
</div>
<h3>Pairs <span id="n">0</span></h3>
<table id="tbl"><tr><th>#</th><th>x_rgb</th><th>y_rgb</th><th>x_lidar</th><th>y_lidar</th></tr></table>
<p>Paste this back:</p>
<textarea id="out" readonly></textarea>
<p id="res"></p>
<script>
const SRC = {{ rgb: "file://__RGB__", lidar: "file://__LID__" }};
const img = {{}}, cv = {{ R: document.getElementById('cR'), L: document.getElementById('cL') }};
let pairs = [], pending = null, which = 'R';
function fit(c, im) {{ c.width = im.naturalWidth; c.height = im.naturalHeight;
  c.getContext('2d').drawImage(im, 0, 0); }}
function redraw() {{
  for (const k of ['R','L']) {{
    const c = cv[k], im = img[k]; if (!im) continue; fit(c, im);
    const ctx = c.getContext('2d'); const idx = (k==='R')?0:1;
    ctx.lineWidth = Math.max(2, im.naturalWidth/400); ctx.font = ctx.lineWidth*6 + 'px sans-serif';
    pairs.forEach((p, i) => {{ const [x,y] = p[idx];
      ctx.strokeStyle = (idx===0)?'#ff4040':'#40ff40'; ctx.fillStyle = ctx.strokeStyle;
      ctx.beginPath(); ctx.arc(x, y, ctx.lineWidth*3, 0, 7); ctx.stroke();
      ctx.fillText(i, x + ctx.lineWidth*4, y - ctx.lineWidth*2); }});
    if (pending && ((k==='R')===true) === (which==='R') && pending.x != null) {{
      ctx.strokeStyle = '#ffd040'; ctx.beginPath();
      ctx.arc(pending.x, pending.y, ctx.lineWidth*4, 0, 7); ctx.stroke(); }}
  }}
  const wR = document.getElementById('ws'); if (img.R) wR.textContent = img.R.naturalWidth+'x'+img.R.naturalHeight;
  const wL = document.getElementById('wl'); if (img.L) wL.textContent = img.L.naturalWidth+'x'+img.L.naturalHeight;
  document.getElementById('n').textContent = pairs.length;
}}
function click(k, e) {{
  const c = cv[k], r = c.getBoundingClientRect();
  const x = Math.round((e.clientX - r.left) / r.width * c.width);
  const y = Math.round((e.clientY - r.top) / r.height * c.height);
  if (which === 'R') {{ if (k !== 'R') return; pending = {{x, y}}; which = 'L';
    document.getElementById('cL').classList.add('active');
    document.getElementById('cR').classList.remove('active'); }}
  else {{ if (k !== 'L') return; pairs.push([pending, {{x, y}}]); pending = null; which = 'R';
    document.getElementById('cR').classList.add('active');
    document.getElementById('cL').classList.remove('active'); }}
  redraw(); render();
}}
function render() {{
  const t = document.getElementById('tbl'); t.innerHTML =
    '<tr><th>#</th><th>x_rgb</th><th>y_rgb</th><th>x_lidar</th><th>y_lidar</th></tr>';
  pairs.forEach((p, i) => {{ const tr = t.insertRow();
    [i, p[0].x, p[0].y, p[1].x, p[1].y].forEach(v => tr.insertCell().textContent = v); }});
  document.getElementById('out').value =
    pairs.map(p => [p[0].x, p[0].y, p[1].x, p[1].y].join(' ')).join('\\n');
}}
for (const k of ['R','L']) {{ img[k] = new Image(); img[k].onload = redraw;
  img[k].src = (k==='R') ? SRC.rgb : SRC.lidar; }}
cv.R.addEventListener('click', e => click('R', e));
cv.L.addEventListener('click', e => click('L', e));
cv.R.classList.add('active');
document.addEventListener('keydown', e => {{
  if (e.key === 'u') {{ if (pending) {{ pending = null; which='R'; }}
    else pairs.pop(); redraw(); render(); }}
  else if (e.key === 'c') {{ pairs = []; pending = null; which='R'; redraw(); render(); }}
}});
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    ap.add_argument("--rgb", default=None)
    ap.add_argument("--lidar", default=None)
    args = ap.parse_args()
    scan = os.path.join(ROOT, "scans", args.scan_id)
    rgb = os.path.abspath(args.rgb or os.path.join(scan, f"{args.scan_id}_blended_fused.jpg"))
    lid = os.path.abspath(args.lidar or os.path.join(scan, f"{args.scan_id}_lidar.jpg"))
    if not os.path.exists(rgb) or not os.path.exists(lid):
        raise SystemExit(f"missing {rgb} or {lid}")
    html = HTML.replace("__SCAN__", args.scan_id).replace("__RGB__", rgb).replace("__LID__", lid)
    out = os.path.join(scan, "tmp", "pano_picker.html")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write(html)
    print(f"wrote {out}")
    print("open:  file://" + out)


if __name__ == "__main__":
    main()