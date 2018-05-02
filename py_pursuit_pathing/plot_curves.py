import matplotlib.pyplot as plot

from py_pursuit_pathing import mathlib

if __name__ == "__main__":
    P1 = mathlib.Polynomial([-0.3800000000000009, 2.687005768508887, 0.0, 0.0])

    xs = list(map(lambda x: x/100, range(-0*100, 5*100)))
    y1s = list(map(P1.compute, xs))

    plot.plot(xs, y1s)
    plot.show()