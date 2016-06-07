from matplotlib.pylab import *
""" To use as a replacement for matplotlib, replace:

    from matplotlib import pylab as plt
    
with:

    import nonblocking_plot as plt
    
Then use as you would with matplotlib's plt.
"""    

def show():
    ax = gca()
    fig = gcf()

    fig.canvas.draw()
    fig.canvas.flush_events()
    fig.show()

