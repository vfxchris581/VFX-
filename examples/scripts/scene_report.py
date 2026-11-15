"""Prints every node of the open scene with its cook time, size and any errors.

    hdc python examples/scripts/scene_report.py examples/destruction.hdg
"""
import hd

rows = []
for node in hd.nodes():
    err = node.errors()
    geo = node.geometry()
    rows.append((node.name(), node.typeLabel(), geo.numPoints(), geo.numPrims(), node.cookTime(), err))

width = max([len(r[0]) for r in rows] + [4])
print("%-*s  %-20s %10s %10s %9s" % (width, "node", "type", "points", "prims", "cook ms"))
for name, label, pts, prims, ms, err in rows:
    print("%-*s  %-20s %10d %10d %9.2f%s" % (width, name, label, pts, prims, ms, "  ERROR: " + err if err else ""))
print("frame %g, range %s, %d nodes" % (hd.frame(), hd.frameRange(), len(rows)))
