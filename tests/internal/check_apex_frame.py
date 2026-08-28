# Copyright (C) 2023-2026 CascadiaVoxel LLC
#
#    nanoPRC is free software: you can redistribute it and/or modify it under
#    the terms of the GNU Affero General Public License as published by the
#    Free Software Foundation, either version 3 of the License, or (at your
#    option) any later version.
#
#    nanoPRC is distributed in the hope that it will be useful, but WITHOUT
#    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#    FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public
#    License for more details.
#
#    You should have received a copy of the GNU Affero General Public License
#    along with nanoPRC. If not, see <https://www.gnu.org/licenses/>.

"""INTERNAL DEVELOPMENT TOOL -- not part of the permanent test suite, not
registered with CTest.

WHAT: Verifies numerically, over random triangles, the algebraic relationship
between the apex frame that ISO/CD 14739-1.4:2026 7.8.9.3 specifies and the
frame nanoPRC's prc_compute_triangle_basis actually builds.

WHY: 7.8.9.3 gives the apex coordinate system as

    O       = (V1 + V0) * 0.5
    (1) X   = (V1 - V0) / |V1 - V0|
    (2) Z   = (V3 - O) ^ X
    (3) Y   = Z ^ X

nanoPRC's decoder instead computes X = V0 - V1 (the negated axis) and then
applies a conditional orientation flip that has no counterpart anywhere in
the clause:

    w = O - V3;  if (dot(Y, w) > 0) { Z = -Z; Y = -Y; }

These are not two independent divergences. Given the negated X the predicate
is algebraically true for every non-degenerate configuration, so the flip is
an unconditional negation in disguise, and the composed result is exactly the
spec frame rotated 180 degrees about Z:

    nanoPRC frame == (-X_spec, -Y_spec, +Z_spec)

Both facts are asserted here rather than argued. The spec's operand order is
not open to interpretation -- 7.8.9.3 states "In the Formula (1), V0 and V1
are taken so that V0 has a treatment index less than V1" -- so a reader that
implements (1)-(3) literally does not arrive at the frame conforming writers
emit. Independent implementations have been observed to negate it too.

See tests/internal/census_orient_flip.c for the companion measurement on real
files, which reports the flip firing on 99.9544% of 31.3M apex-frame
computations across a 310-file corpus; the residual is the null-axis case the
clause hands to MakeOrthoRep, where the assumption Y = Z ^ X does not hold.

HOW: python tests/internal/check_apex_frame.py [trials]
Exit 0 if both properties hold over every trial, 1 otherwise. Pure standard
library; no build directory or corpus needed.
"""

import math
import random
import sys


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def unit(a):
    n = math.sqrt(dot(a, a))
    return [c / n for c in a]


def midpoint(a, b):
    return [(a[i] + b[i]) * 0.5 for i in range(3)]


def main(argv):
    trials = int(argv[1]) if len(argv) > 1 else 200000
    random.seed(7)

    flip_fired = 0
    max_dev = 0.0

    for _ in range(trials):
        v0 = [random.uniform(-10, 10) for _ in range(3)]
        v1 = [random.uniform(-10, 10) for _ in range(3)]
        v3 = [random.uniform(-10, 10) for _ in range(3)]

        origin = midpoint(v0, v1)
        z_temp = sub(v3, origin)

        # 7.8.9.3 Formulas (1)-(3), exactly as printed.
        xs = unit(sub(v1, v0))
        zs = unit(cross(z_temp, xs))
        ys = unit(cross(zs, xs))

        # nanoPRC prc_compute_triangle_basis.
        xn = unit(sub(v0, v1))
        zn = unit(cross(z_temp, xn))
        yn = unit(cross(zn, xn))
        w = sub(origin, v3)
        if dot(yn, w) > 0.0:
            flip_fired += 1
            zn = [-c for c in zn]
            yn = [-c for c in yn]

        dev = max(max(abs(xn[i] + xs[i]) for i in range(3)),
                  max(abs(yn[i] + ys[i]) for i in range(3)),
                  max(abs(zn[i] - zs[i]) for i in range(3)))
        max_dev = max(max_dev, dev)

    print("trials                                  : %d" % trials)
    print("flip predicate fired                    : %d (%.4f%%)"
          % (flip_fired, 100.0 * flip_fired / trials))
    print("max deviation from (-Xs, -Ys, +Zs)      : %.3e" % max_dev)

    ok = True
    if flip_fired != trials:
        print("FAIL: the flip predicate is expected to fire unconditionally on "
              "non-degenerate input")
        ok = False
    # Pure sign flips, so this is exact in IEEE double -- not merely small.
    if max_dev != 0.0:
        print("FAIL: composed frame is not exactly the spec frame rotated 180 "
              "degrees about Z")
        ok = False

    if ok:
        print("OK: flip is unconditional, and the composed frame is exactly "
              "(-Xs, -Ys, +Zs)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
