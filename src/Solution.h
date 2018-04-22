#include "common.h"
#include "gtree/GTree.h"

class Solution {
    public:
        Solution(GTree::G_Tree &);

        virtual void RequestOnline(trip_t);
        virtual void VehicleOnline(trip_t);

    private:
        GTree::G_Tree &gtree_;
};
