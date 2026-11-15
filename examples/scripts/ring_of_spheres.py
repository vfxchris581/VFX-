"""Builds a ring of coloured spheres with the hd module.

Run it from the app (File > Run Python Script...), the Python Shell, or:
    hdc python examples/scripts/ring_of_spheres.py --save ring.hdg
"""
import math
import hd

hd.clearScene()
count = 12

# points on a circle, coloured around the hue wheel, by a Python SOP
points = hd.createNode("python", "ring_points")
points.parm("code").set(
    "import colorsys, math\n"
    "n = %d\n"
    "geo.addPoints([(2 * math.cos(i * 2 * math.pi / n), 0.5, 2 * math.sin(i * 2 * math.pi / n)) for i in range(n)])\n"
    "geo.setAttribValues('Cd', [colorsys.hsv_to_rgb(i / n, 0.7, 0.9) for i in range(n)])\n"
    "geo.setAttribValues('pscale', [0.25 + 0.1 * math.sin(i) for i in range(n)])\n" % count)

ball = hd.createNode("sphere", "ball")
ball.parm("rows").set(16)
ball.parm("cols").set(24)

copies = hd.createNode("copytopoints", "ring")
copies.setInput(0, ball)
copies.setInput(1, points)
copies.parm("pack").set(1)   # share one sphere between all copies

spin = hd.createNode("xform", "spin")
spin.setInput(0, copies)
spin.parm("r").setExpression("$F * 3", 1)   # one turn every 120 frames
spin.setDisplayFlag()

geo = spin.geometry()
print("built", geo, "from", len(hd.nodes()), "nodes")
