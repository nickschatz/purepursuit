import math
import matplotlib.pyplot as plot

import mathlib

if __name__ == "__main__":
    P1 = mathlib.QuinticPolynomial(1, 1, 1, 1, 1, 0)
    appr = P1.get_local_quadratic_approximation(0)

    xs = list(map(lambda x: x/100, range(-1*100, 1*100)))
    y1s = list(map(P1.compute, xs))
    y2s = list(map(appr.compute, xs))

    plot.plot(xs, y1s)
    plot.plot(xs, y2s)
    plot.show()