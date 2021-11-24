from sys import argv
from numpy import genfromtxt

import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx


def plot_points(pts, plot, number, i):
    jet = plt.get_cmap('jet')
    cNorm = colors.Normalize(vmin=0, vmax=number - 1)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
    colorVal = scalarMap.to_rgba(i)
    x, y, z = pts.T
    plot.scatter(x, y, z, color=colorVal, s=1)


def vis():
    # a = genfromtxt('/home/pietrek/Downloads/b.csv', delimiter=',')
    a = genfromtxt(
        f'/home/pietrek/Downloads/test/{argv[1]}.csv', delimiter=',')

    plt.figure()
    ax = plt.subplot(111, projection='3d')
    plot_points(a, ax, 1, 0)

    plt.show()


vis()
