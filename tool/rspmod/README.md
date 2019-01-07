This directory contains GMPL (AMPL) model of the ridesharing problem as
well as a few data files. The model and data files are used to optimally
solve a ridesharing instance. The model defines the problem, and the
data file specifies the instance (customers, vehicles).

This model enforces that all customers must be matched. But, it can be easily
modified to relax this constraint, adding a penalty distance for each unmatched
customer equal to the shortest path length of that customer's trip:

1. Change the objective function to
   ```minimize cost: (sum{k in Vehicles, i in (N union {k,k+m}), j in (N union {k,k+m}): i!=j} sp[i,j]*x[k,i,j]) + (sum{i in CustOrigins} sp[i, i+n] * (1-sum{k in Vehicles, j in (N union {k+m}): j!=i} x[k,i,j]));```
2. Change eq2 from ```= 1``` to ```<= 1```.

Example using glpsol:
    ```glpsol -m rsp.mod -d data/sm-1.dat```

Notes:

- The model does NOT eliminate subtours, watch out.
- Using glpsol or even SYMPHONY, execution time can be a few hours or maybe
  even days, depending on the instance. But commercial solvers are much
  faster.
- The sm-* dat files in data/ correspond to benchmark instances in
  [Cargo_benchmark](https://github.com/jamjpan/Cargo_benchmark). The a2-16
  file is from Cordeau [1].

[1] Cordeau 2006, Op. Res. 54(3) pp.573-586.

