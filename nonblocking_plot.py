from matplotlib.pylab import *

def show():
    ax = gca()
    fig = gcf()

    fig.canvas.draw()
    fig.canvas.flush_events()
    fig.show()

