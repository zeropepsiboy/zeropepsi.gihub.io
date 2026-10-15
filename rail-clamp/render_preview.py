import sys, os, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import vtk
from vtk.util.numpy_support import vtk_to_numpy
COL = {"sleeper": "#a0a09a", "rail": "#8c6148", "joint_bar_R": "#5a6673", "joint_bar_L": "#5a6673", "pad": "#f2e6b3",
       "cylinder": "#e68c1a", "lock_collar": "#cc2619", "plunger": "#d0d0d8", "yoke": "#1a73cc"}
def load(p):
    r = vtk.vtkSTLReader(); r.SetFileName(p); r.Update()
    pd = r.GetOutput(); pts = vtk_to_numpy(pd.GetPoints().GetData())
    ids = vtk_to_numpy(pd.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    return pts[ids]
def shade(tri, c):
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-9
    l = np.array([0.4, -0.5, 0.75]); l /= np.linalg.norm(l); k = 0.45 + 0.55 * np.clip(n @ l, 0, 1)
    base = np.array(matplotlib.colors.to_rgb(c)); return np.clip(base * k[:, None], 0, 1)
name = sys.argv[1]; d = os.path.join("out", name)
views = [(22, -62, "isometric"), (0, -90, "front (along rail)"), (0, 0, "side")]
fig = plt.figure(figsize=(15, 5.4))
for i, (el, az, t) in enumerate(views):
    ax = fig.add_subplot(1, 3, i + 1, projection="3d")
    for part, c in COL.items():
        tri = load(os.path.join(d, part + ".stl"))
        if part == "rail": tri = tri  # full
        ax.add_collection3d(Poly3DCollection(tri, facecolors=shade(tri, c), edgecolor="none"))
    ax.set_xlim(-220, 220); ax.set_ylim(-220, 220); ax.set_zlim(-60, 380); ax.set_box_aspect((440, 440, 440))
    ax.view_init(el, az); ax.set_axis_off(); ax.set_title(t)
fig.suptitle(f"Rail-sleeper clamp - {name}  (gap 18 mm, open state)"); fig.tight_layout()
fig.savefig(f"preview_{name}.png", dpi=110)
