"""Wedging: re-simulates the scene's RBD solver with different glue strengths and reports how much
of the geometry moved, the kind of batch job usually scripted around a simulation.

    hdc python examples/scripts/batch_wedge.py examples/destruction.hdg
"""
import hd

rbd = next((n for n in hd.nodes() if n.type() == "rbdsolver"), None)
if rbd is None:
    raise SystemExit("this scene has no RBD Solver")

# the fractured wall is merged first, so its points come first in the solver's output
wall = next((n for n in hd.nodes() if n.type() == "voronoifracture"), None)
count = wall.geometry().numPoints() if wall else None

frame = 48
for strength in (0.5, 3.0, 20.0):
    rbd.parm("gluestrength").set(strength)
    hd.setFrame(1)
    rest = rbd.geometry().pointPositions()[:count]
    hd.setFrame(frame)
    now = rbd.geometry().pointPositions()[:count]
    fell = sum(1 for a, b in zip(rest, now) if a[1] - b[1] > 0.05)
    print("glue %5.1f m/s: %3d%% of the wall dropped more than 5 cm by frame %d" % (strength, 100 * fell // max(1, len(now)), frame))
