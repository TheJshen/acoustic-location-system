#!/usr/bin/python
import os
filenames = os.listdir('.')
for f in filenames:
    if ( 'orderB' in f):
        r = open(f, "r")
        e = 0
        for l in r:
            e += float(l)
        r.close()
        print "e%s: %f\n" %(f, e)
