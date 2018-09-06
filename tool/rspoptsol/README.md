This directory contains GMPL (AMPL) model of the ridesharing problem as
well as a few data files. The model and data files are used to optimally
solve a ridesharing instance. The model defines the problem, and the
data file specifies the instance (customers, vehicles).

Example using glpsol:
    glpsol -m rsp.mod -d data/tiny-n1m2.instance.dat

Note: the model does NOT eliminate subtours, watch out.

