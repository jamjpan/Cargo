### RSP Solver
# Source of the original: https://logistikforum.org/darp/dial-ride-problem-darp
# Author of original: Greg Nowak (Grzegorz Nowak)
# Modifications by James J. Pan
# Based on the model by Cordeau 2006, Op. Res. 54(3) pp.573-586.
#
# Features:
#   - vehicles have own origins/destinations
#   - nodes have service duration (to comply with DARP)
#   - requires all requests be served (to comply with DARP)
#
# Usage: glpsol -m rsp.mod -d data_file

### Constants
param m                         integer > 0;        # number of vehicles
param n                         integer > 0;        # number of customers
param speed                     integer > 0;        # vehicle speed m/s
param M := 10000                integer > 0;        # for linearization

### Sets
set Vehicles    := 1..m;                            # vehicles
set VehOrigins  := 1..m;                            # vehicle origins
set VehDests    := m+1..2*m;                        # vehicle destinations
set CustOrigins := (2*m+1)..(2*m+n);                # pickups
set CustDests   := (2*m+n+1)..((2*m)+(2*n));        # dropoffs

### Schema:            |--------- N --------|
# A := {1..m, m+1..2m, 2m+1..2mn, 2mn+1..2m2n}
#       |     |        |          |_CustDests
#       |     |        |_CustOrigins
#       |     |_VehDests
#       |_VehOrigins
set A    := CustOrigins union CustDests  union VehOrigins union VehDests;
set N    := CustOrigins union CustDests;

### Data vectors
param nodeid{i in A};                               # not used
param load  {i in A}            integer;            # loads on each node
param serv  {i in A}            integer >= 0;       # service duration
param early {i in A}            integer >= 0;       # early start for node i
param late  {i in A}            integer >= 0;       # late depart for node i
param ride  {i in CustOrigins}  integer > 0;        # max ride time of customer
param cap   {k in Vehicles}     integer >= 0;       # capacity for veh k
param trip  {k in Vehicles}     integer > 0;        # max trip time of vehicle
param sp    {i in A, j in A};                       # sp cost, meters

### Decision vars
var x {i in A, j in A, k in Vehicles: i!=j} binary; # x^k_{ij}
var ArriveTime {i in A, k in Vehicles} >= 0;        # B^k_i
var DepartLoad {i in A, k in Vehicles} >= 0;        # Q^k_i

### Objective function
minimize cost: sum{i in A, j in A, k in Vehicles: i!=j} sp[i,j]*x[i,j,k];

### Constraints
# Service constraints:
# - each customer should be served by at most 1 vehicle
# - if served, each customer should be served by the same vehicle
s.t. eq2  {i in CustOrigins}: sum{k in Vehicles, j in A: j!=i} x[i,j,k] = 1;
s.t. eq3  {k in Vehicles, i in CustOrigins}: (sum{j in A: j!=i} x[i,j,k]) - (sum{j in A: j!=i+n} x[i+n,j,k]) = 0;

# Depot constraints:
# - Vehicles start at their origin
# - If a vehicle enters node j, it must exist node j
# - Vehicles end at their destination
s.t. eq4  {k in Vehicles}: sum{j in A: j!=k} x[k,j,k] = 1;
s.t. eq5  {k in Vehicles, i in N}: (sum{j in A: j!=i} x[j,i,k]) - (sum{j in A: j!=i} x[i,j,k]) = 0;
s.t. eq6  {k in Vehicles}: sum{i in A: i!=k+m} x[i,k+m,k] = 1;

# Time and load constraints:
# (Eqs 7-8 are linearized using (15), (16) from the paper)
# (Eq 7 implies precedence constraint)
# - Arrival time at j must be greater than arrival time at i, plus the travel time
# - Load at j should match load at i plus the change in load
s.t. eq7  {k in Vehicles, i in A, j in A: i!=j}: ArriveTime[j,k] - (ArriveTime[i,k] + serv[i] + sp[i,j]/speed - M*(1-x[i,j,k])) >= 0;
s.t. eq8  {k in Vehicles, i in A, j in A: i!=j}: DepartLoad[j,k] - (DepartLoad[i,k] + load[j] - M*(1-x[i,j,k])) >= 0;

# Ride and trip duration constraints:
# (Eq 9a,b combine (9), (12) from the paper)
# - Customer ride time cannot exceed ride[i]
# - Vehicle trip time cannot exceed trip[k]
s.t. eq9a {k in Vehicles, i in CustOrigins}: ride[i] - (ArriveTime[i+n,k] - (ArriveTime[i,k] + serv[i])) >= 0;
s.t. eq9b {k in Vehicles, i in CustOrigins}: (ArriveTime[i+n,k] - (ArriveTime[i,k] + serv[i])) - sp[i,i+n]/speed >= 0;
s.t. eq10 {k in Vehicles}: trip[k] - (ArriveTime[k+m+n,k] - ArriveTime[k,k]) >= 0;

# Time window constraints
# (Eq 11a,b describe (11))
s.t. eq11a {k in Vehicles, i in A}: ArriveTime[i,k] - early[i] >= 0;
s.t. eq11b {k in Vehicles, i in A}: late[i] - ArriveTime[i,k] >= 0;

# Load constraints
# (Eq 13a,b describe (13))
s.t. eq13a {k in Vehicles, i in A}: DepartLoad[i,k] >= max(0,load[i]);
s.t. eq13b {k in Vehicles, i in A}: DepartLoad[i,k] <= min(cap[k],cap[k]+load[i]);

# Solve
solve;
printf "Solution:\n";
for {i in A}
    for {j in A: j!=i}
        for {k in Vehicles}
            printf (if x[i,j,k] = 1 then "x[%i,%i,%i]\n" else ""), i, j, k;
printf "cost: %.3f\n", cost;

end; # end model
