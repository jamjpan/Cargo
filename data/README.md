# Data

## Benchmarks

TODO (benchmarks aren't final yet)

| Benchmark | Road | n-custs | m-vehls | min t | max t |
| --------- | ---- | ------- | ------- | ----- | ----- |
| rs-lg-5   | mny  | 37,471  | 3,747   | 0     | 0     |

### File formats:

```
*.instance
Line 1 is the instance name.
Line 2 is the road network the instance corresponds to.
Line 3 gives the number of vehicles.
Line 4 gives the number of customers.
Line 5 is blank.
Line 6 gives the column headers
The column headers are:
    ID: the ID of the customer/vehicle
    ORIGIN: the origin node
    DEST: the destination node
    Q: the load (positive = customer; negative = vehicle seat capacity)
    EARLY: the time when the customer/vehicle appears
    LATE: the latest time the customer/vehicle should arrive at dest
```

## Road Networks

| Road | n-nodes | m-edges |
| ---- | ------- | ------- |
| mny  | 12,320  | 15,722  |
| bj5  | 351,290 | 371,911 |
| cd1  | 33,609  | 36,927  |

### File formats:

```
*.edges
The first line is the number of nodes followed by the number of edges.
The rest of the lines has three columns:
    Column 1: edge_id
    Column 2: from node_id of the edge
    Column 3: to node_id of the edge

*.rnet
Each line is seven columns:
    Column 1: edge_id (matches with *.edges)
    Column 2: from node_id of the edge
    Column 3: to node_id of the edge
    Column 4: longitude of the from node
    Column 5: latitude of the from node
    Column 6: longitude of the to node
    Column 7: latitude of the to node
```

