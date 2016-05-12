#!/usr/bin/python
import os
filenames = os.listdir('.')
d = {}
for f in filenames:
    if ( '.txt' in f):
        r = open(f, "r")
        e = 0
        for l in r:
            e += float(l)
        r.close()
        #print "e%s: %f\n" %(f, e)
        d[f] = e

low = min(d, key=d.get)
print low
print d[low]
