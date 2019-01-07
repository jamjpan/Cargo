#!/usr/bin/env bash
EXE=./rspgen_c

cd ../

for m in 1 5 10 15 20 25 30 35 40 45 50
do
    for rn in mny
    do
        echo Building rs-m"$m"k-$rn...
        $EXE -m $((m*1000)) -i data/$rn.rnet -f data/$rn.dat -g data/$rn.gtree out/rs-m"$m"k-$rn &> /dev/null
    done
done
cd script/

echo All done!

