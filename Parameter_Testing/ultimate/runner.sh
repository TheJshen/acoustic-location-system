#!/bin/bash

for f1 in {30000..40001..5000}
   do
        for f2 in {30000..40001..5000}
            do
                for f3 in {30000..40001..5000}
                    do
                        if [ $f1 -ne $f2 ] && [ $f1 -ne $f3 ] && [ $f2 -ne $f3 ]; 
                        then    
                            
                            for NC in {1..21..5}
                                do
                                    for TPS in {1..21..5}
                                        do
                                            for FO in {58..66..1}
                                                do  
                                                    python all.py $f1 $f2 $f3 $NC $TPS $FO 1 1 5
                                                    python all.py $f1 $f2 $f3 $NC $TPS $FO 95 1 5
                                                    python all.py $f1 $f2 $f3 $NC $TPS $FO 1 95 5
                                                    python all.py $f1 $f2 $f3 $NC $TPS $FO -95 1 5
                                                    python all.py $f1 $f2 $f3 $NC $TPS $FO 1 -95 5
    
                                                    python all.py $f1 $f2 $f3 $NC $TPS $FO 1 1 50
                                                    python all.py $f1 $f2 $f3 $NC $TPS $FO 50 1 50
                                                    python all.py $f1 $f2 $f3 $NC $TPS $FO 1 50 50
                                                    python all.py $f1 $f2 $f3 $NC $TPS $FO-50 1 50
                                                    python all.py $f1 $f2 $f3 $NC $TPS $FO 1 -50 50
            
                                                    python all.py $f1 $f2 $f3 $NC $TPS $FO 1 1 100
                                                    
                                                done
                                        done
                            done
                        fi
                    
                    done
            done
        
    done


python readError.py 
