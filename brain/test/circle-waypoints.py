#generate circle setpoint_leader waypoints robodrone mission
from math import *

r=5
wn=3
z=5.0
c=0
csv=True
while c < 2*3.14/(wn*0.05):
    theta=wn*c*0.05
    x=r*sin(theta)
    y=r*cos(theta)
    if (csv):
        print x,";",y,";",z
    else:
        print "- frame: 4"
        print "  command: 16"
        print "  is_current: true"
        print "  autocontinue: true"
        print "  param1: 0.0"
        print "  param2: 0.0"
        print "  param3: 0.0"
        print "  param4: 0.0"
        print "  x_lat:",round(x,2)
        print "  y_long:",round(y,2)
        print "  z_alt:",round(z,2)
    c +=1

