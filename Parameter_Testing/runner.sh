#!/bin/bash

for f1 in {34000..35001..1000}
   do
        for f2 in {30000..40000..1000}
            do
                for f3 in {30000..40000..1000}
                    do
                        if [ $f1 -ne $f2 ] && [ $f1 -ne $f3 ] && [ $f2 -ne $f3 ]; 
                        then
                            python locations.py $f1 $f2 $f3 11 1 1 5
                            python locations.py $f1 $f2 $f3 11 95 1 5
                            python locations.py $f1 $f2 $f3 11 1 95 5
                            python locations.py $f1 $f2 $f3 11 -95 1 5
                            python locations.py $f1 $f2 $f3 11 1 -95 5

                            python locations.py $f1 $f2 $f3 11 1 1 50
                            python locations.py $f1 $f2 $f3 11 50 1 50
                            python locations.py $f1 $f2 $f3 11 1 50 50
                            python locations.py $f1 $f2 $f3 11 -50 1 50
                            python locations.py $f1 $f2 $f3 11 1 -50 50

                            python locations.py $f1 $f2 $f3 11 1 1 100
                        fi
                    
                    done
            done
        
    done

