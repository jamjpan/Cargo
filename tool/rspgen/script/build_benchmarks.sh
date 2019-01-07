#!/usr/bin/env bash
EXE=./rspgen_c

cd ../
echo Building rs-rs...
$EXE -t 0 out/rs-rs &> /dev/null

for m in 1 5 10 15 20 25 30 35 40 45 50
do
    for c in 1 3 6 9
    do
        echo Building rs-m"$m"k-c$c...
        $EXE -m $((m*1000)) -c $c out/rs-m"$m"k-c$c &> /dev/null
    done
    for x in 0.5 1.0 2.0 4.0
    do
        echo Building rs-m"$m"k-x$x...
        $EXE -m $((m*1000)) -x $x out/rs-m"$m"k-x$x &> /dev/null
    done
    for d in 2 4 6 8
    do
        echo Building rs-m"$m"k-d$d...
        $EXE -m $((m*1000)) -d $d out/rs-m"$m"k-d$d &> /dev/null
    done
done
cd script/

echo All done!

