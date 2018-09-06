This directory contains benchmark problem instances and road networks.

benchmark/

    .instance file format:

    Line 1: instance name
    Line 2: road network (space) percentage taxis
    Line 3: VEHICLES (space) number of vehicles
    Line 4: CUSTOMERS (space) number of customers
    Line 5: (blank line)
    Line 6: (header row)
    Line 7-end: vehicles and customers

    Notes:
        * ORIGIN and DEST are specific to the road network in line 2
        * DEST == -1 indicates a taxi
        * Q < 0 indicates vehicle capacity

roadnetwork/

    .edges file format:

    Line 1: number nodes (space) number edges
    Line 2-end: from (space) to (space) weight


    .rnet file format:

    Column 1: edge id
    Column 2: from
    Column 3: to
    Column 4: from longitude
    Column 5: from latitude
    Column 6: to longitude
    Column 7: to latitude

    Notes:
        * from and to match with .edges file
        * Example - visualize using gnuplot:
            plot 'bj5.rnet' u 4:5:($6-$4):($7-$5) w vectors nohead

