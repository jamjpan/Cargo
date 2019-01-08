#!/usr/bin/env bash
#
# build_all.sh
# Script for batch-generating Cargo problem instances via rspgen_c
# James J. Pan <jamesjpan@outlook.com> Last Modified: Jan 7 2019
#
EXE=./rspgen_c
#
## NOTE ABOUT USAGE
# The program rspgen_c requires three input files. These must be specially
# formatted. One of the files is a "trips" file, a collection of raw
# taxi/ridesharing trips, where each row in the file is a trip, and each
# trip is expressed using nine columns.
#
# "trips" file format (extension *.dat):
#
#     Col 1: (trip_id)      , the ID of the trip
#     Col 2: (yyyy/mm/dd)   , date the pickup occurred
#     Col 3: (hh:mm:ss)     , time the pickup occurred
#     Col 4: (lng)          , the longitude of the pickup
#     Col 5: (lat)          , the latitude of the pickup
#     Col 6: (yyyy/mm/dd)   , date the dropoff occurred
#     Col 7: (hh:mm:ss)     , time the dropoff occurred
#     Col 8: (lng)          , the longitude of the dropoff
#     Col 9: (lat)          , the latitude of the dropoff
#
# The other two files are to do with the road network. One of these is an
# "rnet" file. This file lists rows, where each row corresponds to an edge in
# the road network. The format for each row is:
#
# "rnet" file format (extension *.rnet):
#
#     Col 1: (edge id)      , a 0-indexed ID for the edge
#     Col 2: (node_1)       , the ID of node_1 from edges file
#     Col 3: (node_2)       , the ID of node_2 from edges file
#     Col 4: (lng)          , longitude of node_1
#     Col 5: (lat)          , latitude of node_1
#     Col 6: (lng)          , longitude of node_2
#     Col 7: (lat)          , latitude of node_2
#
# The last file is a "gtree" file. This file contains a G-tree spatial index
# for the road network. To obtain one, use tool/gtreebuilder. This particular
# tool requires an "edges" file, easily obtainable from an rnet. Execute
# gtreebuilder with no arguments to see the help text.
#
#
## NOTE ABOUT EXECUTION SPEED
# The full set of problem instances contains 8448 *.instance files.  On my
# machine (i5), it takes about 24 hours to generate the set.  The time can be
# shortened by generating the instances in parallel using xargs:
#
# ./(this_file) | xargs -n(# of args) -P(# of processes) ./rspgen_c
#
# The above command executes ./rspgen_c with the arguments to the left of
# the pipe. Thus this file must be _modified_ to only output the arguments.
# The output of the file should be a string, for example "-m 1000 -c 3".
#
# The bottleneck to execution is that rspgen_c loads all the input files at
# every run. To speed up the program, maybe a "batch mode" could be introduced
# that would load these just once into memory, then re-use them from run to
# run. As another speed-up, the mode could also remember frequent calculations.

cd ../

ROAD=(bj5 cd1 mny)
VEHL=(1 5 10 15 20 25 30 35 40 45 50)
CAPA=(1 3 6 9)
VELO=(10 15 20 25)
SCAL=(0.5 1.0 2.0 4.0)
DELA=(2 4 6 8)

for road in ${ROAD[@]}
do
    for vehl in ${VEHL[@]}
    do
        for capa in ${CAPA[@]}
        do
            for velo in ${VELO[@]}
            do
                for scal in ${SCAL[@]}
                do
                    for dela in ${DELA[@]}
                    do
                        echo road=$road m=$((vehl*1000)) c=$capa d=$dela s=$velo x=$scal
                        $EXE                \
                        -m $((vehl*1000))   \
                        -c $capa            \
                        -d $dela            \
                        -s $velo            \
                        -x $scal            \
                        -i data/$road.rnet  \
                        -f data/$road.dat   \
                        -g data/$road.gtree \
                        out/rs-"$road"-m"$vehl"k-c"$capa"-d"$dela"-s"$velo"-x"$scal" \
                        &> /dev/null
                    done
                done
            done
        done
    done
done
cd script/
echo All done!

