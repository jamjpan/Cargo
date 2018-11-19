Trip Vehicle Grouping

Citation:

Alonso-Mora et. al. 2017. On-demand high-capacity ride-sharing via dynamic
trip-vehicle assignment. PNAS 114(3).

I get only 3448 matches running this method on rs-lg-5 (37,471 customers,
3,747 vehicles!!). The solution cost is 96% of the base cost, a meager 4%
savings. It turns out 20,796 matches get made but are rejected due to
synchronization failure.

Synchronization failure occurs due to matching
latency. Matching latency is the time it takes for when a request is received
until a request is assigned. The algorithm does matching based on the state
of the vehicles at the time the end of a batch. Consider a request
could come at the start of a batch; the batch ends after 30 seconds; the alg
takes another 30 seconds to find matches; by the time the match is returned to
the customer and communicated to the vehicle, the vehicle has already moved
1 minute on the road network. The match might have the vehicle performing a
turn at an intersection that the vehicle has already passed. In this case,
we call this a "synchronization failure" and reject the match.

If we include these rejected matches into the total matches, then this alg
matched 66% of the requests. So it might be better to handle these out-of-sync
matches some other way instead of rejecting them outright.

