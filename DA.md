# Data Access Documentation

### datbases

#### vehicle

id INT
o_id INT // avoid using oid, because sqlite use oid as an alias of rowid
d_id INT
early INT
late INT
demand INT
load INT
nnd INT
route vector
sched vector
lv_node INT
lv_stop INT
is_active INT

#### request

id INT
o_id INT
d_id INT
early INT
late INT
demand INT

### class DA

#### variables



#### methods

__int AddVehicle(Vehicle \*)__

param: vehicle pointer

return:
+ 0 for success
+ 1 for db error

__int UpdateLocation(VehicleId, int, int)__

param: vehicle id, lv_node, nnd

return:
+ 0 for success
+ 1 for db error

__int UpdateSchedule(Vehicle \*, Schedule \*, Route \*)__

param: vehicle pointer, new schedule pointer, new route pointer (the consistency of schedule and route should be managed by user)

return:
+ 0 for success
+ 1 for db error
+ 2 for invalid schedule(schedule that is conflict with the traveled route)

__int UpdateStop(Vehicle \*, int, SimTime)__

param: vehicle pointer, visited stop(index), visit time

return:
+ 0 for success
+ 1 for db error

__int 