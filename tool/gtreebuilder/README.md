This tool will build a G-tree index [1] from an edges file (see
[Cargo_benchmark](https://github.com/jamjpan/Cargo_benchmark) for sample edges
files and format description)

Notes:

    - Nodes must be 0-indexed.
    - Requires the METIS graph partitioning library:
        http://glaros.dtc.umn.edu/gkhome/metis/metis/overview
    - Original gtree implementation:
        https://github.com/TsinghuaDatabaseGroup/GTree

[1] Zhong R, Li G, Tan K-L, Zhou L, Gong Z. G-Tree: an efficient and scalable
    index for spatial search on road networks. TKDE 2015

