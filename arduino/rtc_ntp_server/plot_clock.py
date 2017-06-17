fn = 'clock.dat'

from pylab import *
from numpy import *

lines = open(fn).readlines()
dat = array([map(float, line.split()) for line in lines])
plot(diff(dat[:,0]))
plot(diff(dat[:,1]))
plot(dat[:,2])
show()
