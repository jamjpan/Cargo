This directory contains a DARP/RSP mixed-integer programming model and
data instances to use with the model. The model is written in
[GMPL](https://en.wikibooks.org/wiki/GLPK/GMPL_%28MathProg%29) and can be loaded
into a variety of MIP solvers.

For example, to solve the tiny-n1m2 instance using glpsol, run
```
glpsol -m rsp.mod -d data/tiny-n1m2.instance.dat
```

I recommend using something other than glpsol for the a2-16 instance because it
will take a long time. Using [SYMPHONY](https://projects.coin-or.org/SYMPHONY)
seems to be a little faster. A commercial solver like Gurobi or CPLEX will
likely be even faster, though I haven't tried.
