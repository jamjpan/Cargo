#!/usr/bin/env bash
rm data/benchmark/*.instance
tool/rspgen/rspgen data/roadnetwork/cd1.rnet 11 2 SC -f 10 -o rs-sm-1
tool/rspgen/rspgen data/roadnetwork/mny.rnet 12 6 SR -f 20 -o rs-sm-2
tool/rspgen/rspgen data/roadnetwork/cd1.rnet 12 1 SR -f 5 -o rs-sm-3
tool/rspgen/rspgen_real data/roadnetwork/mny.rnet 13 7 18 -o rs-sm-4
tool/rspgen/rspgen data/roadnetwork/cd1.rnet 14 1 SC -f 2 -o rs-sm-5
mv rs-sm-1_1.instance data/benchmark/rs-sm-1.instance
mv rs-sm-2_1.instance data/benchmark/rs-sm-2.instance
mv rs-sm-3_1.instance data/benchmark/rs-sm-3.instance
mv rs-sm-4 data/benchmarks/rs-sm-4.instance
mv rs-sm-5_1.instance data/benchmark/rs-sm-5.instance

tool/rspgen/rspgen data/roadnetwork/bj5.rnet 865 173 SR -f 10 -o rs-md-1
tool/rspgen/rspgen data/roadnetwork/bj5.rnet 1130 226 SR -f 5 -o rs-md-2
tool/rspgen/rspgen data/roadnetwork/cd1.rnet 1477 148 SC -f 20 -o rs-md-3
tool/rspgen/rspgen data/roadnetwork/bj5.rnet 1931 966 SR -f 2 -o rs-md-4
tool/rspgen/rspgen data/roadnetwork/cd1.rnet 2527 505 SC -f 2 -o rs-md-5
tool/rspgen/rspgen data/roadnetwork/bj5.rnet 3307 331 SC -f 20 -o rs-md-6
tool/rspgen/rspgen_real data/roadnetwork/mny.rnet 4329 2165 18 -o rs-md-7
tool/rspgen/rspgen data/roadnetwork/bj5.rnet 5668 2834 SC -f 5 -o rs-md-8
tool/rspgen/rspgen_real data/roadnetwork/cd1.rnet 7422 742 18 -o rs-md-9
tool/rspgen/rspgen data/roadnetwork/cd1.rnet 9720 1944 SR -f 10 -o rs-md-10
mv rs-md-1_1.instance data/benchmark/rs-md-1.instance
mv rs-md-2_1.instance data/benchmark/rs-md-2.instance
mv rs-md-3_1.instance data/benchmark/rs-md-3.instance
mv rs-md-4_1.instance data/benchmark/rs-md-4.instance
mv rs-md-5_1.instance data/benchmark/rs-md-5.instance
mv rs-md-6_1.instance data/benchmark/rs-md-6.instance
mv rs-md-7 data/benchmark/rs-md-7.instance
mv rs-md-8_1.instance data/benchmark/rs-md-8.instance
mv rs-md-9 data/benchmark/rs-md-9.instance
mv rs-md-10_1.instance data/benchmark/rs-md-10.instance

tool/rspgen/rspgen data/roadnetwork/bj5.rnet 12730 1273 SC -f 10 -o rs-lg-1
tool/rspgen/rspgen data/roadnetwork/cd1.rnet 16674 3335 SC -f 5 -o rs-lg-2
tool/rspgen/rspgen data/roadnetwork/cd1.rnet 21839 10920 SC -f 2 -o rs-lg-3
tool/rspgen/rspgen data/roadnetwork/bj5.rnet 28606 5721 SR -f 20 -o rs-lg-4
tool/rspgen/rspgen_real data/roadnetwork/mny.rnet 37471 3747 18 -o rs-lg-5
mv rs-lg-1_1.instance data/benchmark/rs-lg-1.instance
mv rs-lg-2_1.instance data/benchmark/rs-lg-2.instance
mv rs-lg-3_1.instance data/benchmark/rs-lg-3.instance
mv rs-lg-4_1.instance data/benchmark/rs-lg-4.instance
mv rs-lg-5 data/benchmark/rs-lg-5.instance

