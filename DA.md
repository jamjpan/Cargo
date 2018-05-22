# Data Access Documentation

### class DA

#### variables



#### methods

__int AddVehicle(Vehicle \*)__

param: vehicle pointer

return:
+ 0 for success
+ 1 for db error

__int UpdateLocation(Vehicle \*, NodeId)__

param: vehicle pointer, node id of the new location

return:
+ 0 for success
+ 1 for db error

__int UpdateSchedule(Vehicle \*, Schedule \*, Route \*)__

param: vehicle pointer, new schedule pointer

return:
+ 0 for success
+ 1 for db error
+ 2 for invalid schedule(schedule that is conflict with the traveled route)

route will be compute when schedule is updated.

__int UpdateStop(Vehicle \*, int, SimTime)__

param: vehicle pointer, visited stop(index), visit time

return:
+ 0 for success
+ 1 for db error

__int 