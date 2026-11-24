"""Quay 08, a seismically damaged wholesale fish market, in metres (Z up)."""

import math
import random

from geometry import Part, collection, lettering

rng = random.Random(808)
H = 7.6


def wall(p, start, end, height, mat="limewash", thickness=0.28, base=0):
    x, y = start
    u, v = end
    p.box(
        ((x + u) / 2, (y + v) / 2, base + height / 2),
        (abs(u - x) + thickness, abs(v - y) + thickness, height),
        mat,
    )


def rail(p, a, b, base, colour="iron"):
    x, y = a
    u, v = b
    divisions = math.ceil(math.hypot(u - x, v - y) / 1.6)
    for k in range(divisions + 1):
        t = k / divisions
        px, py = x + (u - x) * t, y + (v - y) * t
        p.rod((px, py, base), (px, py, base + 1.02), 0.036, colour)
    for h in (0.49, 1.02):
        p.rod((x, y, base + h), (u, v, base + h), 0.043, colour)


def sign(col, text, x, y, z, width, size, bg="deep"):
    p = Part(text + " | enamel", col)
    p.box((x, y, z), (width, 0.072, size * 1.68), bg)
    p.box((x, y - 0.038, z - size * 0.69), (width - 0.13, 0.015, 0.023), "ochre", solid=False)
    for dx in (-width / 2 + 0.11, width / 2 - 0.11):
        p.rod((x + dx, y - 0.045, z), (x + dx, y - 0.055, z), 0.029, "steel", solid=False)
    p.finish()
    lettering(col, text, (x, y - 0.052, z + 0.014), size)


def architecture():
    floor = collection("01  /  terrazzo and drainage")
    p = Part("Raft slab and tiled work floors", floor)
    # The generated flight world places its ground plane at the basement datum.
    deck(p, (-18.25, 18.25, -13.25, 13.25), -0.19, 0.38, "concrete", [WEST_WELL, DOCK_WELL])
    for x, y, sx, sy in [(-13.6, 0, 8.3, 25.6), (13.7, 0, 8, 25.6), (0, -2.5, 17.9, 19.8)]:
        deck(
            p,
            (x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2),
            0.003,
            0.006,
            "tile",
            [WEST_WELL, DOCK_WELL],
            False,
        )
        for i in range(1, int(sx)):
            u = x - sx / 2 + i
            deck(
                p,
                (u - 0.007, u + 0.007, y - sy / 2, y + sy / 2),
                0.007,
                0.004,
                "concrete",
                [WEST_WELL, DOCK_WELL],
                False,
            )
        for i in range(1, int(sy)):
            v = y - sy / 2 + i
            deck(
                p,
                (x - sx / 2, x + sx / 2, v - 0.007, v + 0.007),
                0.007,
                0.004,
                "concrete",
                [WEST_WELL, DOCK_WELL],
                False,
            )
    for x in (-8.15, 8.15):
        p.box((x, 0, 0.014), (0.18, 24, 0.018), "coal", solid=False)
        for i in range(144):
            p.box((x, -11.9 + i / 6, 0.027), (0.18, 0.045, 0.012), "steel", solid=False)
    for x in (-2.2, 2.2):
        for y in range(-11, 8, 2):
            p.box((x, y, 0.015), (0.09, 1.05, 0.012), "ochre", solid=False)
    p.finish()

    walls = collection("02  /  masonry and loading bays")
    p = Part("North elevation | brick piers and clerestory lights", walls)
    for x in (-18, 18):
        wall(p, (x, -13), (x, 13), H)
        p.box((x * 0.992, 0, 0.64), (0.16, 25.9, 1.26), "teal")
    wall(p, (-18, 13), (18, 13), H)
    for x in range(-16, 18, 4):
        p.box((x, 12.82, 5.58), (3.05, 0.06, 1.95), "coal")
        for dx in (-0.95, 0, 0.95):
            p.box((x + dx, 12.76, 5.6), (0.84, 0.045, 1.74), "slate", solid=False)
        p.box((x, 12.71, 5.7), (3, 0.035, 0.065), "steel", solid=False)
    # Side rooms each have multiple entries. Lintels retain flight headroom.
    for x in (-9.4, 9.4):
        for y1, y2 in [(-13, -8.1), (-5, -0.2), (3.8, 7.5)]:
            wall(p, (x, y1), (x, y2), 3.38)
            p.box((x, (y1 + y2) / 2, 0.62), (0.34, y2 - y1, 1.24), "tile")
        for y1, y2 in [(-8.1, -5), (-0.2, 3.8)]:
            wall(p, (x, y1), (x, y2), 0.34, "iron", base=3.05)
    for x1, x2 in [(-18, -16), (-12, -7.7), (-4.4, 2.2), (5.9, 11.4), (15.4, 18)]:
        wall(p, (x1, 7.65), (x2, 7.65), 3.38)
        p.box(((x1 + x2) / 2, 7.47, 0.68), (x2 - x1, 0.08, 1.36), "teal")
    for x1, x2 in [(-18, -17.15), (-12.65, -9.4), (9.4, 12.8), (17.78, 18)]:
        wall(p, (x1, -1.3), (x2, -1.3), 3.1, "ceramic")
    p.finish()

    front = collection("03  /  street facade • cutaway")
    p = Part("Street doors and entrance transom", front)
    for x1, x2 in [(-18, -2.4), (2.4, 18)]:
        wall(p, (x1, -13), (x2, -13), 5.2)
        p.box(((x1 + x2) / 2, -12.8, 0.65), (x2 - x1, 0.12, 1.3), "deep")
    p.box((0, -13, 4.9), (5.2, 0.45, 0.62), "iron")
    p.finish()
    sign(front, "QUAY  08     /     FISH EXCHANGE", 0, -13.24, 4.9, 8.5, 0.44)

    frame = collection("04  /  riveted portal frames")
    for y in (-10.6, -4.1, 2.4, 8.85):
        p = Part(f"Riveted bent at {y:g} m", frame)
        for x in (-8.72, 8.72):
            p.box((x, y, 0.24), (0.74, 0.78, 0.48), "concrete")
            p.box((x, y, 5.56), (0.21, 0.37, 10.20), "iron")
            for dx in (-0.16, 0.16):
                p.box((x + dx, y, 5.56), (0.075, 0.45, 10.20), "teal")
            for z in (0.3, 0.65, 2.8, 5.1, 7.5, 9.1, 10.45):
                p.box((x, y - 0.21, z), (0.39, 0.05, 0.23), "iron", solid=False)
                for dx in (-0.12, 0.12):
                    p.rod((x + dx, y - 0.24, z), (x + dx, y - 0.26, z), 0.027, "steel", solid=False)
            for side in (-1, 1):
                p.beam((x, y, 8.7), (x + side * 1.65, y, 10.51), 0.12, 0.16, "iron")
        for side in (-1, 1):
            p.beam((side * 9.2, y, 10.60), (0, y, 11.90), 0.17, 0.21, "teal")
        p.box((0, y, 9.63), (18, 0.20, 0.20), "oxide")
        for i in range(12):
            x = -8.7 + i * 1.45
            p.beam(
                (x, y, 9.68 if i % 2 == 0 else 11.80 - abs(x) * 0.138),
                (x + 1.45, y, 11.80 - abs(x + 1.45) * 0.138 if i % 2 == 0 else 9.68),
                0.085,
                0.09,
                "iron",
            )
        p.finish()

    roof = collection("05  /  roof sheets • cutaway")
    p = Part("Service roofs and ruptured sawtooth canopy", roof)
    for x in (-13.75, 13.75):
        p.box((x, 0, 7.58), (8.6, 26.3, 0.18), "slate")
    # A continuous high lantern rises above both low service-wing roofs.
    # Banks of broken fanlights alternate with boarded bays along the eaves.
    for side in (-1, 1):
        x = side * 9.04
        for z, h in [(7.79, 0.42), (9.05, 0.13), (10.47, 0.28)]:
            p.box((x, 0, z), (0.19, 26.0, h), "iron")
        for j in range(24):
            y = -12.5 + j * 1.08
            p.box((x, y, 9.11), (0.13, 0.09, 2.75), "teal")
            if j % 6 in (1, 4):
                p.box((x, y + 0.5, 8.61), (0.07, 0.89, 1.04), "slate")
            elif j % 6 == 3:
                p.beam((x, y + 0.08, 7.99), (x, y + 0.91, 9.01), 0.095, 0.046, "wood")
    for y in (-12.68, 12.70):
        for j in range(17):
            x = -8 + j
            top = 11.78 - abs(x) * 0.14
            p.box((x, y, 8.66), (0.94, 0.13, 1.87), "tile")
            p.box((x - 0.48, y, (9.60 + top) / 2), (0.09, 0.16, top - 9.60), "iron")
            if j % 5 in (0, 2):
                p.box((x, y, (9.60 + top) / 2), (0.89, 0.065, top - 9.64), "slate")
    for x in (-7, -4, -1, 1, 4, 7):
        z = 11.78 - abs(x) * 0.14
        p.box((x, 0, z), (0.095, 26.0, 0.13), "iron")
    # North half of the central roof failed. Broken sheet ends remain on the purlins.
    for side in (-1, 1):
        for i in range(24):
            y = -12.5 + i * 1.1
            length = rng.uniform(1.8, 4.8) if -2 < y < 8 and side == 1 else rng.uniform(4.8, 6.8)
            x = side * (8.9 - length / 2)
            z = 10.65 + (8.9 - abs(x)) * 0.14
            p.box((x, y, z), (length, 1.09, 0.08), "slate", (0, side * 0.14, 0))
            for dy in (-0.38, 0, 0.38):
                p.box((x, y + dy, z + 0.055), (length, 0.052, 0.07), "iron", (0, side * 0.14, 0), False)
    p.finish()
    return front


def galleries():
    col = collection("06  /  inspection galleries and stairs")
    p = Part("North gallery and east return", col)
    for x, y, sx, sy in [(0, 10.4, 35.6, 5.1), (13.6, 2.35, 8.1, 11.0)]:
        deck(
            p,
            (x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2),
            3.46,
            0.28,
            "concrete",
            [DOCK_WELL, EAST_STAIR],
        )
        for dx in (-sx / 2 + 0.13, sx / 2 - 0.13):
            p.box((x + dx, y, 3.22), (0.18, sy, 0.3), "iron")
    p.finish()
    p = Part("Cross-hall bridge and stairs", col)
    # The elevated bridge meets the south lip of the shaft and the north
    # door of the tally office. The east gallery branches at the shaft head.
    ax, ay, bx, by = 0, -5.3, 7.0, 8.52
    angle = math.atan2(by - ay, bx - ax)
    length = math.hypot(bx - ax, by - ay)
    p.box(((ax + bx) / 2, (ay + by) / 2, 3.46), (length, 2.6, 0.28), "concrete", (0, 0, angle))
    for side in (-1, 1):
        dx, dy = -math.sin(angle) * 1.26 * side, math.cos(angle) * 1.26 * side
        rail(p, (ax + dx, ay + dy), (bx + dx, by + dy), 3.6)
        p.beam((ax + dx, ay + dy, 3.32), (bx + dx, by + dy, 3.32), 0.15, 0.25, "iron")
    for x0, x1 in [(-17.8, -15.4), (-10.6, 5.1), (9.2, 9.6)]:
        rail(p, (x0, 7.88), (x1, 7.88), 3.6)
    for a, b in [((9.62, -3.1), (9.62, 7.8)), ((9.7, -3.07), (14.85, -3.07))]:
        rail(p, a, b, 3.6)
    # West stair has a landing at bridge level, then connects to the north return.
    p.box((-10.85, 1.55, 3.46), (2.75, 2.2, 0.28), "concrete")
    p.box((-11.55, 5.20, 3.46), (1.65, 5.2, 0.28), "concrete")
    rail(p, (-12.3, 2.6), (-12.3, 7.73), 3.6)
    for x, start_y, end_y in [(-10.85, -5.1, 0.46), (-14.15, 2.25, 7.85), (16.3, -3.4, 3.6)]:
        for i in range(20):
            step = (end_y - start_y) / 20
            top = (i + 1) * 0.18
            p.box((x, start_y + (i + 0.5) * step, top - 0.07), (2.02, step + 0.01, 0.14), "concrete")
            p.box((x, start_y + i * step + 0.018, top + 0.004), (1.95, 0.035, 0.014), "ochre", solid=False)
        for dx in (-1.02, 1.02):
            p.beam((x + dx, start_y, 0.03), (x + dx, end_y, 3.49), 0.13, 0.17, "iron")
            p.rod((x + dx, start_y, 0.98), (x + dx, end_y, 4.57), 0.047, "teal")
            for i in range(5):
                t = i / 4
                y = start_y + (end_y - start_y) * t
                p.rod((x + dx, y, 0.12 + t * 3.46), (x + dx, y, 0.98 + t * 3.59), 0.035, "iron")
    p.finish()
    sign(col, "AUCTION   /   HALL 08", -0.65, 7.46, 3.15, 5.6, 0.37)
    sign(col, "COLD STORE", -13.95, 7.45, 3.21, 3.6, 0.27)
    sign(col, "PROCESSING   /   EAST STAIR", 12.0, -3.16, 3.48, 4.1, 0.25)


def crate(p, x, y, z, angle=0, colour="teal", w=0.9, d=0.62):
    # Vented fish boxes, with a solid bottom and open top.
    c, s = math.cos(angle), math.sin(angle)

    def bar(dx, dy, dz, sx, sy, sz, mat):
        p.box((x + c * dx - s * dy, y + s * dx + c * dy, z + dz), (sx, sy, sz), mat, (0, 0, angle))

    bar(0, 0, 0.045, w, d, 0.085, colour)
    for side in (-1, 1):
        for h in (0.13, 0.26, 0.36):
            bar(0, side * (d / 2 - 0.025), h, w, 0.05, 0.055, colour)
            bar(side * (w / 2 - 0.025), 0, h, 0.05, d, 0.055, colour)
        for dx in (-w * 0.43, 0, w * 0.43):
            bar(dx, side * (d / 2 - 0.03), 0.21, 0.048, 0.06, 0.33, colour)


def pallet(p, x, y, z=0):
    for dx in (-0.63, 0, 0.63):
        p.box((x + dx, y, z + 0.09), (0.16, 1.08, 0.18), "umber")
    for i in range(6):
        p.box((x - 0.68 + i * 0.27, y, z + 0.205), (0.21, 1.2, 0.07), "wood")


def fish(p, x, y, z, angle):
    c, s = math.cos(angle), math.sin(angle)

    def at(u, v, h):
        return (x + c * u - s * v, y + s * u + c * v, z + h)

    # Spindle, paired flank highlights and forked tail.
    points = [
        at(-0.22, 0, 0.04),
        at(-0.07, -0.067, 0.035),
        at(0.09, -0.055, 0.038),
        at(0.22, 0, 0.04),
        at(0.09, 0.055, 0.038),
        at(-0.07, 0.067, 0.035),
        at(-0.015, 0, 0.091),
        at(-0.015, 0, 0),
    ]
    p.poly(points, [(i, (i + 1) % 6, 6) for i in range(6)], "fish", False)
    p.poly(points, [(i, 7, (i + 1) % 6) for i in range(6)], "belly", False)
    p.poly(
        [at(0.17, 0, 0.04), at(0.30, -0.065, 0.052), at(0.27, 0, 0.048), at(0.30, 0.07, 0.05)],
        [(0, 1, 2), (0, 2, 3)],
        "fish",
        False,
    )


def auction():
    col = collection("07  /  wet auction hall")
    for i, (x, y) in enumerate([(-4.8, -3), (4.8, -3), (-4.8, 4.4), (4.8, 3.72)]):
        p = Part(f"Auction island {i + 1:02} | iced trays and dial scale", col)
        for dx in (-1.75, 1.75):
            for dy in (-0.48, 0.48):
                p.box((x + dx, y + dy, 0.47), (0.16, 0.16, 0.94), "iron")
        p.box((x, y, 0.92), (4.15, 1.52, 0.24), "ceramic")
        p.box((x, y - 0.72, 0.65), (4.16, 0.12, 0.45), "teal")
        for k in (-1.37, 0, 1.37):
            p.box((x + k, y, 1.06), (1.28, 1.26, 0.06), "ice")
            for dx in (-0.65, 0.65):
                p.box((x + k + dx, y, 1.095), (0.05, 1.35, 0.12), "steel")
            for n in range(5):
                fish(p, x + k + rng.uniform(-0.41, 0.41), y - 0.48 + n * 0.22, 1.10, rng.uniform(-0.4, 0.4))
        for dy in (-0.72, 0.72):
            p.box((x, y + dy, 1.10), (4.15, 0.055, 0.17), "steel")
        # Elevated scale at one end.
        sx = x - 1.88
        p.rod((sx, y + 0.46, 1.05), (sx, y + 0.46, 1.77), 0.045, "iron")
        p.rod((sx, y + 0.51, 1.72), (sx, y + 0.37, 1.72), 0.23, "teal", 24)
        p.rod((sx, y + 0.365, 1.72), (sx, y + 0.352, 1.72), 0.19, "ivory", 24, False)
        p.beam((sx, y + 0.34, 1.72), (sx + 0.095, y + 0.34, 1.82), 0.014, 0.012, "oxide", False)
        p.finish()
        sign(col, f"LOT   0{i + 1}", x, y - 0.81, 0.76, 1.05, 0.145)
    p = Part("Auctioneer's booth | glazed ticket windows", col)
    p.box((0, -7.55, 0.69), (4.9, 1.55, 1.38), "teal")
    p.box((0, -7.55, 1.40), (5.06, 1.7, 0.11), "wood")
    for x in (-2.42, -0.82, 0.82, 2.42):
        p.box((x, -7.85, 2.0), (0.075, 0.08, 1.18), "iron")
    p.box((0, -7.85, 2.61), (4.97, 0.14, 0.22), "deep")
    p.finish()
    sign(col, "TICKETS    /    CRIEE", 0, -8.37, 1.02, 2.96, 0.23)
    # A fallen display board blocks the diagonal view into the north-east lot.
    p = Part("Dislodged lot board", col)
    p.box((5.45, 2.94, 1.3), (3.85, 0.23, 2.54), "deep", (0.08, 0, -0.10))
    p.finish()


def machinery():
    col = collection("08  /  cold chain and packing rooms")
    for i, (x, y, z) in enumerate([(-16.0, -9.9, 0), (-15.7, 5.0, 0), (2.4, 11.65, 3.6), (14.15, 1.1, 3.6)]):
        p = Part(f"Refrigeration plant {i + 1:02} | fans, ducts and valves", col)
        p.box((x, y, z + 0.16), (2.86, 1.38, 0.32), "concrete")
        p.box((x, y, z + 1.27), (2.7, 1.14, 1.92), "ceramic")
        for dx in (-0.62, 0.62):
            fy = y - 0.60
            p.rod((x + dx, fy + 0.015, z + 1.41), (x + dx, fy - 0.05, z + 1.41), 0.46, "coal", 24)
            for n in range(6):
                a = n * math.pi / 3
                p.beam(
                    (x + dx, fy - 0.074, z + 1.41),
                    (x + dx + 0.31 * math.cos(a), fy - 0.074, z + 1.41 + 0.31 * math.sin(a)),
                    0.096,
                    0.055,
                    "steel",
                    False,
                )
            for j in range(-3, 4):
                reach = math.sqrt(0.42**2 - (j * 0.103) ** 2)
                p.rod(
                    (x + dx - reach, fy - 0.108, z + 1.41 + j * 0.103),
                    (x + dx + reach, fy - 0.108, z + 1.41 + j * 0.103),
                    0.012,
                    "iron",
                    6,
                    False,
                )
        for dx in (-0.9, 0.55):
            p.rod((x + dx, y + 0.29, z + 2.12), (x + dx, y + 0.29, z + 2.52), 0.095, "copper")
            p.rod((x + dx, y + 0.29, z + 2.52), (x + dx + 0.62, y + 0.29, z + 2.52), 0.095, "copper")
        p.finish()
    for i, (x, y, levels) in enumerate(
        [
            (-15, -12.0, 3),
            (-12.45, -8.57, 4),
            (-16.4, 3.25, 3),
            (14, -5.05, 4),
            (12, -9, 2),
            (15.7, 4.3, 3),
            (-4.6, 10, 3),
            (16.55, 6.67, 4),
            (12.5, -0.7, 2),
        ]
    ):
        p = Part(f"Fish-box pallet {i + 1:02}", col)
        pallet(p, x, y)
        for j in range(levels):
            for dx in (-0.46, 0.46):
                crate(p, x + dx, y, 0.24 + j * 0.38, colour=("teal", "slate", "clay")[(i + j) % 3])
        p.finish()
    # Packing benches, splashbacks, overhead service run.
    for x, y in [(15, -7.65), (-12.8, -10.7), (11.85, 3.90), (3.9, 10.4)]:
        p = Part("Stainless packing bench", col)
        p.box((x, y, 1.04), (3.1, 1.12, 0.12), "steel")
        p.box((x, y + 0.51, 1.34), (3.1, 0.065, 0.59), "ceramic")
        for dx in (-1.36, 1.36):
            for dy in (-0.42, 0.42):
                p.box((x + dx, y + dy, 0.50), (0.085, 0.085, 1.0), "iron")
        p.finish()
    for x in (-16.9, 17.39):
        p = Part("Brine supply and return", col)
        for dx, z in [(0, 3.95), (0.29, 4.26)]:
            p.rod((x + dx, -12, z), (x + dx, 12.5, z), 0.105, "copper", 12)
            for y in range(-11, 13, 2):
                p.rod((x + dx, y, z), (x + dx, y + 0.07, z), 0.135, "iron", 12)
        p.finish()
    for label, x, y, z in [
        ("01   /   ICE PLANT", -14.85, -5.97, 3.14),
        ("03   /   DISPATCH", 13.75, 7.45, 3.22),
        ("BRINE  /  PUMP  B", -8.0, 12.77, 5.9),
    ]:
        sign(col, label, x, y, z, 4.2, 0.275)


def damage():
    col = collection("09  /  fracture field and fallen canopy")
    p = Part("Sheared canopy | folded steel, torn edges and rebar", col)
    # Sheets lean against the south-east column, breaking one aisle.
    for i in range(6):
        y = -4.1 + i * 0.24
        p.beam((5.7, y, 0.15), (8.55, y + 0.60, 2.58), 0.24, 0.084, "slate")
    for dy in (0, 1.2):
        p.beam((5.55, -4.12 + dy, 0.12), (8.68, -3.45 + dy, 2.66), 0.13, 0.16, "oxide")
    p.finish()
    for i, (x, y, r) in enumerate(
        [
            (7.3, -4.9, 2.4),
            (-1.7, 5.5, 2.1),
            (-11.6, 0.35, 1.25),
            (9.7, 8.2, 1.45),
            (-5.9, -9.3, 1.4),
            (3.2, 11.1, 1.45),
        ]
    ):
        p = Part(f"Spall fan {i + 1:02} | aggregate and reinforcement", col)
        for n in range(59):
            a = rng.random() * math.tau
            dist = rng.random() ** 0.72 * r
            xx, yy = x + math.cos(a) * dist, y + math.sin(a) * dist * 0.68
            s = rng.uniform(0.074, 0.36) * (1.3 - dist / r * 0.55)
            if n in (0, 1):
                s = 0.66
            count = rng.randint(5, 8)
            ring = [
                (
                    math.cos(k * math.tau / count) * s * rng.uniform(0.72, 1.18),
                    math.sin(k * math.tau / count) * s * rng.uniform(0.44, 0.89),
                )
                for k in range(count)
            ]
            pts = [(xx + u, yy + v, 0.012) for u, v in ring]
            pts += [(xx + u * 0.76 + 0.04, yy + v * 0.79, s * rng.uniform(0.46, 0.94)) for u, v in ring]
            p.poly(
                pts,
                [tuple(reversed(range(count)))]
                + [(0 + count, k + count, k + 1 + count) for k in range(1, count - 1)]
                + [(k, (k + 1) % count, (k + 1) % count + count, k + count) for k in range(count)],
                rng.choice(["spall", "concrete", "clay"]),
            )
        for j in range(4):
            dx = j * 0.15
            p.rod((x + dx, y, 0.13), (x + dx + 0.36, y + 0.21, 0.34), 0.019, "oxide")
            p.rod(
                (x + dx + 0.36, y + 0.21, 0.34), (x + dx + 0.86, y + 0.49, 0.39 + dx * 0.23), 0.019, "oxide"
            )
        p.finish()
    # Floor fissures branch from the same impact zones as the fallen material.
    p = Part("Hairline cracks and wash water", col)
    for x, y in [(7, -5), (-2, 5), (-14, -1.3), (-6, -9), (4, 11)]:
        for j in range(4):
            a = rng.random() * math.tau
            for k in range(5):
                dx = math.cos(a) * rng.uniform(0.28, 0.65)
                dy = math.sin(a) * rng.uniform(0.28, 0.65)
                w = 0.016 * (1 - k / 6)
                p.poly(
                    [
                        (x - w, y, 0.030),
                        (x + w, y, 0.030),
                        (x + dx + w, y + dy, 0.030),
                        (x + dx - w, y + dy, 0.030),
                    ],
                    [(0, 1, 2, 3)],
                    "umber",
                    False,
                )
                x, y = x + dx, y + dy
                a += rng.uniform(-0.6, 0.6)
    for x, y in [(-6.9, -4.55), (5, -0.7), (-15, -8.5), (14.8, 5.8)]:
        ring = [
            (
                x + math.cos(i * math.tau / 24) * rng.uniform(0.91, 1.44),
                y + math.sin(i * math.tau / 24) * rng.uniform(0.41, 0.69),
                0.018,
            )
            for i in range(24)
        ]
        p.poly(ring, [tuple(range(24))], "water", False)
    p.finish()


def identity():
    col = collection("10  /  exchange clock and fittings")
    sign(col, "NORTH QUAY     /     FISH EXCHANGE", 0, 12.65, 6.31, 13.5, 0.57)
    p = Part("Clock over the north gallery", col)
    for y, r, mat in [(12.55, 0.79, "deep"), (12.42, 0.68, "ivory")]:
        p.rod((0, y, 4.95), (0, y - 0.12, 4.95), r, mat, 48)
    for i in range(12):
        a = math.tau * i / 12
        p.beam(
            (0.51 * math.sin(a), 12.28, 4.95 + 0.51 * math.cos(a)),
            (0.60 * math.sin(a), 12.28, 4.95 + 0.60 * math.cos(a)),
            0.032,
            0.024,
            "ink",
            False,
        )
    for x, z in [(-0.35, 5.30), (0.21, 5.12)]:
        p.beam((0, 12.25, 4.95), (x, 12.25, z), 0.059, 0.032, "ink", False)
    p.finish()
    for x in (-5.1, 5.1):
        p = Part("Enamel pendant reflectors", col)
        for y in (-10.6, -4.1, 2.4, 8.85):
            offset = 0.33 if x > 0 and y == -4.1 else 0
            p.rod((x, y, 9.63), (x + offset, y, 5.9), 0.018, "coal")
            p.rod((x + offset, y, 5.71), (x + offset, y, 5.91), 0.33, "teal", 24, True, 0.075)
            p.rod((x + offset, y, 5.698), (x + offset, y, 5.71), 0.29, "beacon", 24, False)
        p.finish()
    for x, y in [(-4.2, -6.25), (6.25, 7.12), (-14.7, 9.1), (15.1, -3.7)]:
        p = Part("Loaded box trolley", col)
        for dx in (-0.45, 0.45):
            for dy in (-0.27, 0.27):
                p.rod((x + dx - 0.06, y + dy, 0.14), (x + dx + 0.06, y + dy, 0.14), 0.13, "coal", 12)
        p.box((x, y, 0.31), (1.12, 0.75, 0.095), "ochre")
        for dy in (-0.31, 0.31):
            p.rod((x - 0.49, y + dy, 0.32), (x - 0.56, y + dy, 1.06), 0.031, "iron")
        p.rod((x - 0.56, y - 0.31, 1.06), (x - 0.56, y + 0.31, 1.06), 0.035, "iron")
        crate(p, x + 0.10, y, 0.36, 0.08, "clay")
        p.finish()
    for x, y, z in [(-2.8, 7.49, 1.30), (1.35, 12.85, 4.35), (11.0, -1.45, 1.65)]:
        p = Part("Delaminated render exposing brick courses", col)
        for row in range(6):
            span = 1 + min(row, 5 - row)
            for j in range(span + 1):
                p.box(
                    (x + (j - span / 2) * 0.34, y - 0.012, z + row * 0.137),
                    (0.321, 0.046, 0.115),
                    rng.choice(["clay", "umber", "oxide"]),
                    solid=False,
                )
        for side in (-1, 1):
            xx, zz = x + side * 0.30, z + 0.72
            for k in range(6):
                nx, nz = xx + side * rng.uniform(0.05, 0.19), zz + rng.uniform(0.10, 0.24)
                p.beam((xx, y - 0.047, zz), (nx, y - 0.047, nz), 0.023, 0.017, "umber", False)
                xx, zz = nx, nz
        p.finish()


WEST_WELL = (-16.8, -13.0, -5.8, 2.05)
DOCK_WELL = (5.1, 8.9, 8.6, 12.25)
EAST_STAIR = (14.95, 17.65, -3.0, 3.6)


def deck(p, rect, z, thickness, mat, holes, solid=True):
    """Rectangular slab strips with real through-openings in both meshes."""
    pieces = [rect]
    for hx, hu, hy, hv in holes:
        rest = []
        for x, u, y, v in pieces:
            a, b = max(x, hx), min(u, hu)
            c, d = max(y, hy), min(v, hv)
            if b <= a or d <= c:
                rest.append((x, u, y, v))
                continue
            rest.extend([(x, a, y, v), (b, u, y, v), (a, b, y, c), (a, b, d, v)])
        pieces = [r for r in rest if r[1] - r[0] > 0.0001 and r[3] - r[2] > 0.0001]
    for x, u, y, v in pieces:
        p.box(((x + u) / 2, (y + v) / 2, z), (u - x, v - y, thickness), mat, solid=solid)


def incline(p, a, b, width, mat, thickness=0.12):
    dx, dy, dz = (v - u for u, v in zip(a, b))
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    center = tuple((u + v) / 2 for u, v in zip(a, b))
    p.box(center, (math.dist(a, b), width, thickness), mat, (0, pitch, yaw))


def service_levels():
    col = collection("10b  /  subgrade ice works and loading well")
    p = Part("Subgrade L corridor | insulated envelope", col)
    # Basement is an L: the floor above is continuous except at the two wells.
    for a, b in [
        ((-17.8, -12.95), (-17.8, 12.9)),
        ((-11.4, -6.5), (-11.4, 7.6)),
        ((-11.4, -6.5), (17.7, -6.5)),
        ((17.7, -12.95), (17.7, -6.5)),
        ((17.7, 7.6), (17.7, 12.9)),
        ((-17.8, 12.9), (17.7, 12.9)),
    ]:
        wall(p, a, b, 3.56, "concrete", base=-3.6)
    section = collection("03b  /  subgrade retaining wall • cutaway")
    skin = Part("Retaining walls on the section plane", section)
    for a, b in [((-17.8, -12.95), (17.7, -12.95)), ((-11.4, 7.6), (17.7, 7.6))]:
        wall(skin, a, b, 3.56, "concrete", base=-3.6)
    skin.finish()
    for r in [(-17.85, -11.38, -12.95, 12.9), (-11.38, 17.75, -12.95, -6.5), (-11.38, 17.75, 7.6, 12.9)]:
        deck(p, r, -3.68, 0.16, "tile", [])
    for x in (-16.84, -12.9):
        p.box((x, -5.80, 1.57), (0.17, 0.23, 3.14), "iron")
    p.box((-14.85, -5.80, 3.21), (4.14, 0.24, 0.26), "teal")
    incline(p, (-14.85, -5.80, -0.07), (-14.85, 1.90, -3.60), 3.72, "concrete", 0.15)
    for x in (-16.65, -13.05):
        p.rod((x, -5.8, 0.73), (x, 1.9, -2.80), 0.045, "ochre")
        for j in range(7):
            t = j / 6
            z = -3.53 * t
            p.rod((x, -5.8 + 7.7 * t, z), (x, -5.8 + 7.7 * t, z + 0.73), 0.038, "iron")
    for x, z in [(-17.38, -1.45), (-17.38, -2.02)]:
        p.rod((x, -5.8, z), (x, 12.3, z), 0.13, "copper", 12)
        p.rod((x, 12.3, z), (16.5, 12.3, z), 0.13, "copper", 12)
        for y in range(-5, 12, 2):
            p.rod((x, y, z), (x, y + 0.09, z), 0.17, "oxide", 12)
    for y in (-0.8, 4.6, 9.0):
        p.box((-16.95, y, -2.89), (1.2, 1.42, 1.42), "teal")
        p.rod((-16.55, y, -2.38), (-15.98, y, -2.38), 0.32, "iron", 18)
    p.finish()
    for x1, x2, y1, y2 in [DOCK_WELL]:
        p = Part("Loading shaft | bumper kerbs and caged ladder", col)
        for y in (y1, y2):
            p.box(((x1 + x2) / 2, y, 0.09), (x2 - x1, 0.12, 0.18), "ochre")
        for x in (x1 + 0.30, x1 + 0.94):
            p.rod((x, y2 - 0.26, -3.46), (x, y2 - 0.26, 0.81), 0.038, "oxide")
        for j in range(13):
            p.rod(
                (x1 + 0.30, y2 - 0.26, -3.30 + j * 0.32),
                (x1 + 0.94, y2 - 0.26, -3.30 + j * 0.32),
                0.028,
                "iron",
            )
        p.finish()
    col = collection("10c  /  tally office, beacon and ice chute")
    p = Part("Elevated tally suite and east access gallery", col)
    deck(p, (-2.8, 2.8, -8.45, -5.15), 3.46, 0.28, "concrete", [])
    # The bridge enters from the north; the only other open side feeds the chute.
    wall(p, (2.8, -8.45), (2.8, -5.2), 2.8, "tile", base=3.6)
    for x, u in [(-2.8, -1.55), (1.55, 2.8)]:
        wall(p, (x, -5.2), (u, -5.2), 2.8, "tile", base=3.6)
    wall(p, (-2.8, -8.45), (2.8, -8.45), 0.88, "teal", base=3.6)
    wall(p, (-2.8, -8.45), (2.8, -8.45), 0.31, "deep", base=6.10)
    for j in range(21):
        x = -2.8 + j * 0.28
        p.box((x, -8.45, 5.31), (0.039, 0.115, 1.69), "steel")
    p.box((0, -6.80, 6.44), (5.86, 3.44, 0.16), "iron")
    for x in (-2.80, 2.80):
        for y in (-8.28, -5.35):
            p.box((x, y, 5.0), (0.16, 0.23, 2.8), "iron")
    # Warm prismatic beacon directly inside the viewing slit.
    p.rod((0.14, -8.24, 5.65), (0.14, -8.24, 5.77), 0.24, "iron", 24)
    p.rod((0.14, -8.24, 5.77), (0.14, -8.24, 6.02), 0.18, "beacon", 24)
    p.rod((0.14, -8.24, 6.02), (0.14, -8.24, 6.10), 0.21, "iron", 24)
    p.finish()
    p = Part("Broken ice-return chute | sloped bed and folded sides", col)
    a = (-2.82, -6.45, 3.53)
    b = (-4.42, -10.05, 0.20)
    incline(p, a, b, 2.55, "steel")
    angle = math.atan2(b[1] - a[1], b[0] - a[0])
    for side in (-1, 1):
        dx, dy = -math.sin(angle) * 1.26 * side, math.cos(angle) * 1.26 * side
        incline(
            p, (a[0] + dx, a[1] + dy, a[2] + 0.35), (b[0] + dx, b[1] + dy, b[2] + 0.35), 0.11, "deep", 0.69
        )
    p.finish()
    sign(col, "WEIGH OFFICE     /     08", 0, -8.62, 4.12, 4.3, 0.28)


def failed_bay():
    col = collection("09b  /  failed eastern roof bay")
    p = Part("Buckled roof lattice grounded across the east sale aisle", col)
    # The bottom chord has folded; portions of the attached corrugated sheet
    # remain hooked on the upper chord. The west aisle still passes the wreck.
    sections = [
        (2.75, -0.12, 0.07),
        (3.92, -0.77, 0.8),
        (5.18, -1.55, 1.44),
        (6.45, -2.34, 2.67),
        (7.52, -3.27, 3.72),
        (8.65, -4.08, 5.14),
    ]
    for i, (x, y, z) in enumerate(sections):
        p.beam((x, y - 0.47, z), (x, y + 0.47, z), 0.1, 0.12, "oxide")
        for side in (-1, 1):
            p.beam((x, y + side * 0.47, z), (x + 0.03, y, z + 0.71), 0.082, 0.085, "iron")
            if i:
                u, v, w = sections[i - 1]
                p.beam((u, v + side * 0.47, w), (x, y + side * 0.47, z), 0.1, 0.12, "oxide")
                p.beam((u, v + side * 0.47, w), (x, y, z + 0.71), 0.067, 0.07, "iron")
        if i:
            u, v, w = sections[i - 1]
            p.beam((u, v, w + 0.71), (x, y, z + 0.71), 0.11, 0.12, "teal")
    p.finish()
    p = Part("Draped electric conduit and detached lamp", col)
    for j in range(3):
        pts = [
            (
                8.66 - 6.3 * i / 18,
                -4.1 + (3.9 + j * 0.32) * i / 18,
                6.87 - (2.9 + j * 0.28) * i / 18 - 2.4 * math.sin(math.pi * i / 18),
            )
            for i in range(19)
        ]
        for a, b in zip(pts, pts[1:]):
            p.rod(a, b, 0.024, "coal", 6)
    p.rod((2.35, 0.13, 3.41), (2.35, 0.13, 3.90), 0.025, "coal")
    p.rod((2.35, 0.13, 3.40), (2.51, 0.11, 3.21), 0.08, "teal", 20, True, 0.33)
    p.finish()
    # Two cold-store panels provide different occluders on the auction floor and mezzanine.
    p = Part("Shutter jammed above the west lot", col)
    for x in (-6.38, -3.0):
        p.box((x, -3.0, 1.68), (0.095, 0.14, 3.36), "iron")
    for j in range(18):
        p.box((-4.69, -3.0, 1.05 + j * 0.123), (3.34, 0.10, 0.128), "slate", (0.09, 0, -0.008))
    p.finish()
    p = Part("Insulated locker bank | gallery equipment shelter", col)
    p.box((-12.78, 8.82, 4.64), (3.82, 0.76, 2.08), "tile")
    for j in range(5):
        x = -14.23 + j * 0.72
        p.box((x, 8.408, 4.65), (0.68, 0.07, 1.89), "deep")
        for k in range(4):
            p.box((x, 8.365, 5.23 + k * 0.046), (0.32, 0.022, 0.015), "steel", solid=False)
        p.box((x + 0.20, 8.345, 4.61), (0.038, 0.05, 0.19), "ochre", solid=False)
    p.finish()


def loading_gear():
    col = collection("10d  /  dock hoist and ice tanks")
    p = Part("North dock | hoist tower, guides and suspended pallet", col)
    for x in (5.0, 9.0):
        for y in (8.49, 12.32):
            p.box((x, y, 2.11), (0.27, 0.34, 11.40), "iron")
            p.box((x, y, 7.7), (0.40, 0.43, 0.22), "oxide")
        p.beam((x, 8.49, 7.79), (x, 12.32, 7.79), 0.22, 0.31, "teal")
    for y in (8.49, 12.32):
        p.beam((5, y, 7.8), (9, y, 7.8), 0.32, 0.38, "oxide")
    p.rod((7.0, 10.4, 7.54), (8.43, 10.4, 7.54), 0.27, "iron", 20)
    for x in (7.35, 8.21):
        for y in (10.25, 10.55):
            p.rod((x, y, 7.47), (x, y, 6.72), 0.023, "coal")
    p.box((7.80, 10.4, 6.67), (1.28, 0.98, 0.14), "wood")
    for i in range(4):
        p.box((7.8, 10.0, 6.8 + i * 0.145), (1.08, 0.09, 0.10), "wood")
        p.box((7.8, 10.8, 6.8 + i * 0.145), (1.08, 0.09, 0.10), "wood")
    p.finish()
    sign(col, "NORTH  LIFT     /     02", 7.0, 8.30, 7.52, 3.75, 0.27)
    p = Part("Plate-ice machine and brine vessels", col)
    for x, y in [
        (-8.9, 11.83),
        (-3.2, 11.83),
        (2.6, 11.83),
        (11.45, 11.83),
        (-8.8, -10.4),
        (-2.8, -10.4),
        (4.8, -10.4),
        (13.55, -10.4),
    ]:
        p.rod((x, y, -3.55), (x, y, -1.51), 0.68, "steel", 32)
        p.rod((x, y, -1.51), (x, y, -1.31), 0.68, "ceramic", 32, True, 0.32)
        for z in (-3.25, -2.8, -2.0):
            p.rod((x, y, z), (x, y, z + 0.048), 0.70, "iron", 32)
        p.rod((x, y, -1.31), (x, y, -0.86), 0.12, "steel", 12)
        p.rod((x, y, -0.86), (x + 0.88, y, -0.86), 0.12, "steel", 12)
        p.rod((x, y - 0.61, -2.64), (x, y - 1.02, -2.64), 0.063, "oxide", 12)
        for i in range(10):
            a, b = i * math.tau / 10, (i + 1) * math.tau / 10
            p.rod(
                (x + 0.18 * math.cos(a), y - 1.04, -2.64 + 0.18 * math.sin(a)),
                (x + 0.18 * math.cos(b), y - 1.04, -2.64 + 0.18 * math.sin(b)),
                0.022,
                "clay",
            )
    p.finish()
    p = Part("East stairwell | masonry returns and landing strings", col)
    for y in (-3.4, 3.6):
        p.box((17.56, y, 2.98), (0.25, 0.34, 5.96), "concrete")
    # The open face looks into processing. The shaft has independent upper
    # and lower mouths at opposite ends of its flight of stairs.
    for j in range(5):
        y = -3.1 + j * 1.45
        p.box((17.56, y, 4.49), (0.14, 1.22, 1.56), "slate")
        p.box((17.44, y, 4.49), (0.08, 1.28, 0.063), "steel", solid=False)
    p.finish()


def finishes():
    col = collection("10e  /  glazing, spalled brickwork and loose stock")
    p = Part("Interwar factory windows | rendered piers with exposed brick arrises", col)
    for side in (-1, 1):
        x = side * 17.83
        for bay in range(6):
            y = -10.65 + bay * 4.21
            p.box((x, y, 4.93), (0.12, 3.48, 2.62), "concrete")
            for j in range(4):
                for k in range(2):
                    yy = y - 1.20 + j * 0.80
                    zz = 4.35 + k * 1.14
                    p.box(
                        (x - side * 0.084, yy, zz),
                        (0.043, 0.735, 1.045),
                        "deep" if (bay * 7 + j + k) % 9 == 0 else "slate",
                        solid=False,
                    )
            p.box((x - side * 0.12, y, 3.56), (0.28, 3.62, 0.16), "ceramic")
            if bay % 2 == 0:
                for row in range(11):
                    for j in range(1 + (row % 5 != 0)):
                        yy = y - 1.76 + (j - 0.5) * 0.32 + (row % 2) * 0.14
                        p.box(
                            (x - side * 0.06, yy, 1.95 + row * 0.135),
                            (0.13, 0.306, 0.12),
                            rng.choice(["clay", "umber", "oxide"]),
                            solid=False,
                        )
    p.finish()
    for x, y in [(2.2, -0.9), (-5.8, -4.9), (10.8, 5.85), (-11.9, -6.65)]:
        p = Part("Spilled auction stock", col)
        crate(p, x, y, 0.01, 0.47, "slate")
        for i in range(12):
            fish(p, x + rng.uniform(0.44, 1.35), y + rng.uniform(-0.78, 0.8), 0.023, rng.random() * math.tau)
        p.finish()
    p = Part("Ice bins and ticket office awning", col)
    for x, y in [(-10.5, 8.47), (-4.8, 8.47), (0.7, 8.47), (-6.1, -10.9), (0.85, -10.9), (9.7, -10.9)]:
        for dx in (-0.85, 0.85):
            p.box((x + dx, y, -2.99), (0.10, 1.1, 1.20), "teal")
        for dy in (-0.5, 0.5):
            p.box((x, y + dy, -2.99), (1.72, 0.12, 1.20), "teal")
        p.box((x, y, -2.55), (1.7, 1.04, 0.12), "ice")
        for i in range(18):
            p.box(
                (x + rng.uniform(-0.70, 0.70), y + rng.uniform(-0.40, 0.40), -2.42),
                (0.21, 0.16, 0.15),
                "ice",
                (0.12, rng.random() * 0.48, rng.random() * math.pi),
                False,
            )
    for x in (-2.38, 2.38):
        p.box((x, -8.08, 2.26), (0.084, 0.12, 1.7), "iron")
    for j in range(30):
        p.box(
            (-2.65 + j * 0.18, -7.62, 2.93),
            (0.184, 1.93, 0.055),
            "oxide" if j % 4 == 0 else "teal",
            (0.10, 0, 0),
        )
    p.finish()
    for x, y in [(-14.3, 1.65), (-14.2, 8.2), (-6.9, 10.6), (2.6, 10.6), (12.6, 10.7)]:
        p = Part("Bulkhead lamp and conduit", col)
        p.rod((x, y, -0.57), (x, y, -0.40), 0.19, "iron", 16)
        p.rod((x, y, -0.61), (x, y, -0.57), 0.14, "beacon", 16)
        p.finish()


def scale_details(front_collection):
    col = collection("10f  /  fire doors, scales and fittings")
    # Full-height south gable above the loading transom. Narrow industrial
    # panes and steel mullions bridge the low eaves to the clerestory.
    gable = Part("Street infill and glazed transom", front_collection)
    for x in range(-8, 9):
        gable.box((x, -12.98, 5.57), (0.99, 0.25, 0.74), "limewash")
        gable.box((x, -12.98, 7.41), (0.99, 0.24, 0.71), "limewash")
        gable.box((x - 0.48, -12.98, 6.50), (0.085, 0.21, 1.2), "iron")
        gable.box((x, -12.98, 6.53), (0.90, 0.06, 1.04), "slate")
        gable.box((x, -12.9, 6.47), (0.94, 0.05, 0.055), "steel", solid=False)
    gable.finish()
    for x, y in [(-3.25, 7.45), (4.65, -12.81)]:
        # Near wall and far wall have handles on the hall-facing side.
        front = -1 if y > 0 else 1
        p = Part("Personnel fire door • 0.96 × 2.10 m", col)
        p.box((x, y, 1.08), (1.12, 0.12, 2.16), "iron")
        p.box((x, y + front * 0.081, 1.05), (0.96, 0.055, 2.10), "teal")
        for radius, mat, offset in [(0.183, "steel", 0.120), (0.147, "coal", 0.140)]:
            p.rod(
                (x, y + front * offset, 1.58), (x, y + front * (offset + 0.012), 1.58), radius, mat, 28, False
            )
        p.box((x + 0.29, y + front * 0.143, 1.0), (0.18, 0.066, 0.036), "steel")
        p.box((x, y + front * 0.08, 2.32), (0.42, 0.11, 0.14), "deep")
        p.box((x, y + front * 0.15, 2.32), (0.28, 0.022, 0.064), "ivory", solid=False)
        p.finish()
    p = Part("Inspection desk, pigeonholes and shift ledger", col)
    p.box((2.08, -6.83, 4.50), (0.70, 1.65, 0.091), "wood")
    for y in (-7.49, -6.17):
        p.box((2.08, y, 4.02), (0.62, 0.09, 0.85), "iron")
    for j in range(4):
        p.box((2.45, -6.80, 4.7 + j * 0.20), (0.30, 1.29, 0.032), "wood")
    p.box((1.96, -6.55, 4.58), (0.28, 0.35, 0.061), "ivory", (0, 0, 0.18), False)
    p.finish()
    # Worn, enamelled weigh scales and a pressure extinguisher on the piers.
    for x, y in [(-8.72, -4.1), (8.72, 2.4), (-8.72, 8.85)]:
        p = Part("Fire extinguisher and station number", col)
        sx = x + 0.30
        p.rod((sx, y, 0.54), (sx, y, 1.04), 0.093, "oxide", 16)
        p.rod((sx, y, 1.04), (sx, y, 1.10), 0.092, "oxide", 16, True, 0.045)
        p.box((sx, y, 1.125), (0.14, 0.055, 0.048), "coal")
        p.box((sx, y - 0.088, 0.83), (0.091, 0.017, 0.18), "ivory", solid=False)
        p.finish()
