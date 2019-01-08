The program rspgen_c generates a problem instance in the format described in
[Cargo_benchmark](https://github.com/jamjpan/Cargo_benchmark) by sampling
trips from a "trips" file. It mainly does two things:

    1. Matches the lng, lat coordinates of trips in the file to nodes
       in the road network. It does so by using a kd-tree to find the nearest
       node, given a coordinate.
    2. Computes the estimated travel time for each trip by dividing the length
       of the shortest path for a trip by user-supplied vehicle speed. It
       uses G-tree to compute shortest path lengths.

It then creates an "instance" file of vehicles and customers based on user
parameters. These parmaeters are described in the usage:

```
Generate problem instances for the RSP.
Usage: rspgen_c [OPTIONS] filename
Parameters:

        filename        output the generated instance
Options:

        -m      Number of vehicles (default=5000)
        -c      Vehicle capacity (default=3)
        -s      Vehicle speed (default=10 m/s)
        -t      Vehicle type (default=1(taxi))
        -d      Trip delay (default=6 min)
        -b      Begin hour (default=18)
        -r      Sampling duration (default=30 min)
        -l      Customer load (default=1)
        -x      Sampling scale (default=1)
        -i      Input road network (default=data/bj5.rnet)
        -f      Input trips file (default=data/bj5.dat)
        -g      Input gtree (default=data/bj5.gtree)
```

For historic purposes, the old python scripts are included in py/, but
they are much slower (and maybe buggy) than the C++ version.

Some build scripts are included in script/. These programmatically generate a
whole set of benchmarks. The script build_all.sh in particular builds 8,448
instances covering combinations of parameters "most reflective" of the real
world.

