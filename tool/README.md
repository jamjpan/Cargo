This directory contains tools related to Cargo.

datcvt/

    Please add a README so folks know what this is and how to use it.
    Thank you.


gtreebuilder/

    This tool builds a G-tree spatial index [1], given an edge file
    (see data/roadnetworks for edge files).

    [1] Zhong R, Li G, Tan K-L, Zhou L, Gong Z. G-Tree: an efficient and
        scalable index for spatial search on road networks. TKDE 2015


probplot/

    This tool plots a problem instance (see data/benchmarks for instance files).


rspoptsol/

    This directory contains GMPL (AMPL) model of the ridesharing problem as
    well as a few data files. The model and data files are used to optimally
    solve a ridesharing instance. The model defines the problem, and the
    data file specifies the instance (customers, vehicles).


solplot/

    This tool visualizes a simulation using the Cargo-generated .dat file.

