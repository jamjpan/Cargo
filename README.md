(We are working on a publication comparing different ridesharing algorithms
using this system, but we could use some help and expertise! If you would like
to find out more or possibly collaborate, please reach out to me at
jamesjpan@outlook.com)

# Cargo
**(under development)**

*A C++11 static library plus benchmarks for implementing ridesharing algorithms
and evaluating them through simulation.*

Ridesharing is a type of vehicle routing problem (VRP). The objective is to
route vehicles to service customers as they continually appear on a road
network, while minimizing travel distance.

## What it does
Cargo provides key components for implementing and evaluating ridesharing
algorithms. Users can implement algorithms by extending `RSAlgorithm`, and then
evaluate them on the included benchmarks. The evaluation is through real-time
simulation on real-world road networks using real taxi customers and vehicles
data.

## How to install

Dependencies:
- [METIS - Serial Graph Partitioning and Fill-reducing Matrix
  Ordering](http://glaros.dtc.umn.edu/gkhome/metis/metis/overview)
- A POSIX threads implementation, such as `libpthread`
- `librt`, included in most Linux systems
- `libdl`, included in most Linux systems
- g++ 5.4.0 or higher?

### Building the library

Clone the repository then use `make`:
```
git clone https://github.com/jamjpan/Cargo.git && cd Cargo && make
```
The library will be located in `lib/libcargo.a`.

### Compiling your code

Place the contents of the `include/` directory somewhere your compiler can
access.  Using `g++`, you can compiled your code with the `-I` flag to point
the compiler to the location of these includes, if necessary. For example:
```
g++ -std=c++11 -g -c -I/home/myuser/Cargo/include mycode.cc -o mycode.o
```

### Linking

Place `libcargo.a` somewhere your compiler can access. Using `g++`, link your
code using `-lcargo`. Also link the dependencies using `-pthread -lrt -lmetis
-ldl`. Use the `-L` flag to locate the libraries, if necessary. For example:
```
g++ -pthread -lrt -lmetis -ldl -L/home/myuser/Cargo/lib -lcargo myalg.o -o myalg
```

## Usage

See `/docs` (in-progress).

## Example

See the `/example` for examples.

## Outputs

### Solution file (\*.sol)

```
```
Explanation:

### Data file (*.dat)

The data file is not meant for human consumption, but is human readable. It
comprises seven types of lines:

```
t [R, V, P, D, T, A, M] [data]
```

* t is the time step of the simulation
* R, V, P, D, T, A, M are each event codes
    * R indicates new route; data = [VehlId] [sequence of NodeIds]
    * V indicates new location; data = [VelId1] [NodeId] [VehlId2] [NodeId] ...
    * P indicates pickup; data = [CustId1] [CustId2] ...
    * D indicates dropoff; data = [CustId1] [CustId2] ...
    * T indicates timeout; data = [CustId1] [CustId2] ...
    * A indicates arrival; data = [VehlId1] [VehlId2] ...
    * M indicates match; data = [VehlId] [CustId1] [CustId2] ...

The data file can be fed into `tool/solplot/solplot.py` to generated an mp4
visualization.

## Bugs and Contributing

If you discover a bug, or have a suggestion, please
[Submit an Issue](https://github.com/jamjpan/Cargo/issues/new)!

You can also submit a pull request, preferrably against your own branch or
against the dev branch.

## License

Cargo is distributed under the MIT License.

