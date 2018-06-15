#!/usr/bin/env bash
# Small
./rspgen ../../data/roadnetwork/cd1.rnet 11 2 SC -f 10 -o rs-sm-1
./rspgen ../../data/roadnetwork/mny.rnet 12 6 SR -f 20 -o rs-sm-2
./rspgen ../../data/roadnetwork/cd1.rnet 12 1 SR -f 5 -o rs-sm-3
./rspgen ../../data/roadnetwork/mny.rnet 13 7 R  -b 18 -o rs-sm-4 -i mny_trips.dat
./rspgen ../../data/roadnetwork/cd1.rnet 14 1 SC -f 2 -o rs-sm-5
# Medium
./rspgen ../../data/roadnetwork/bj5.rnet 865 173 SR -f 10 -o rs-md-1
./rspgen ../../data/roadnetwork/bj5.rnet 1130 226 SR -f 5 -o rs-md-2
./rspgen ../../data/roadnetwork/cd1.rnet 1477 148 SC -f 20 -o rs-md-3
./rspgen ../../data/roadnetwork/bj5.rnet 1931 966 SR -f 2 -o rs-md-4
./rspgen ../../data/roadnetwork/cd1.rnet 2527 505 SC -f 2 -o rs-md-5
./rspgen ../../data/roadnetwork/bj5.rnet 3307 331 SC -f 20 -o rs-md-6
./rspgen ../../data/roadnetwork/mny.rnet 4329 2165 R -b 18 -o rs-md-7 -i mny_trips.dat
./rspgen ../../data/roadnetwork/bj5.rnet 5668 2834 SC -f 5 -o rs-md-8
./rspgen ../../data/roadnetwork/cd1.rnet 7422 742 R -b 18 -o rs-md-9 -i cd1_trips.dat
./rspgen ../../data/roadnetwork/cd1.rnet 9720 1944 SR -f 10 -o rs-md-10
# Large
./rspgen ../../data/roadnetwork/bj5.rnet 12730 1273 SC -f 10 -o rs-lg-1
./rspgen ../../data/roadnetwork/cd1.rnet 16674 3335 SC -f 5 -o rs-lg-2
./rspgen ../../data/roadnetwork/cd1.rnet 21839 10920 SC -f 2 -o rs-lg-3
./rspgen ../../data/roadnetwork/bj5.rnet 28606 5721 SR -f 20 -o rs-lg-4
./rspgen ../../data/roadnetwork/mny.rnet 37471 3747 R -b 18 -o rs-lg-5 -i mny_trips.dat

