#include <iostream>
#include <fstream>
#include <sstream>
#include "gtree/gtree.h"

const int VEH_N = 3005;
const int CUST_N = 1005;
const int SPEED = 20;

struct trip {
    int id;
    int origin;
    int dest;
    int q;
    int early;
    int late;
};

int vehicle_num, customer_num;
trip *vehicles[VEH_N];
trip *customers[CUST_N];
GTree::G_Tree gtree;

int main(int argc, char *argv[])
{
    /* validation */
    if (argc != 4)
        std::cerr << "Usage: ./datcvt gtree_path instance_path, dat_path" << std::endl;
    
    /* configuration */
    std::string gtree_path(argv[1]);
    std::string instance_path(argv[2]);
    std::string dat_path(argv[3]);

    /* initialization */
    GTree::load(gtree_path);
    gtree = GTree::get();
    std::ifstream ifs(instance_path);
    std::ofstream ofs(dat_path);

    /* parse instance */
    std::string line, _;
    for (int i = 0; i < 2; ++i)
        std::getline(ifs, line);
    
    /** get total numbers **/
    std::getline(ifs, line);
    std::istringstream iss(line);
    iss >> _ >> vehicle_num;
    iss.clear();

    std::getline(ifs, line);
    iss.str(line);
    iss >> _ >> customer_num;

    std::cout << "GET: " << vehicle_num << " vehicles and " << customer_num << " customers" << std::endl;

    ofs << "data;" << std::endl;
    ofs << "param m := " << vehicle_num << ";" << std::endl;
    ofs << "param n := " << customer_num << ";" << std::endl;
    ofs << "param speed := " << SPEED << ";" << std::endl;
    ofs << std::endl;

    for (int i = 0; i < 2; ++i)
        std::getline(ifs, line);

    /** get trips data **/
    int id, origin, dest, q, early, late;
    for (int i = 0; i < vehicle_num; ++i) {
        ifs >> id >> origin >> dest >> q >> early >> late;
        vehicles[i] = new trip {id, origin, dest, q, early, late};
    }
    for (int i = 0; i < customer_num; ++i) {
        ifs >> id >> origin >> dest >> q >> early >> late;
        customers[i] = new trip {id, origin, dest, q, early, late};
    }

    ofs << "param:" << std::endl;
    ofs << "   serv load early late nodeid :=" << std::endl;

    for (int i = 0; i < vehicle_num; ++i)
        ofs << " " << i + 1 << " 0   0   0   900000   " << vehicles[i]->origin << std::endl;
    for (int i = 0; i < vehicle_num; ++i)
        ofs << " " << vehicle_num + i + 1 << " 0   0   0   900000   " << vehicles[i]->dest << std::endl;
    for (int i = 0; i < customer_num; ++i)
        ofs << " " << vehicle_num * 2 + i + 1 << " 0   1   0   900000   " << customers[i]->origin << std::endl;
    for (int i = 0; i < customer_num; ++i) {
        ofs << " " << vehicle_num * 2 + customer_num + i + 1 << " 0  -1   0   900000   " << customers[i]->dest;
        if (i == customer_num - 1)
            ofs << ";\n\n";
        else
            ofs << std::endl;
    }

    ofs << "param ride :=" << std::endl;
    for (int i = 0; i < customer_num; ++i) {
        ofs << " " << vehicle_num * 2 + i + 1 << " " << gtree.search(customers[i]->origin, customers[i]->dest) + SPEED * 600;
        if (i == customer_num - 1)
            ofs << ";\n\n";
        else
            ofs << std::endl;
    }

    ofs << "param trip :=" << std::endl;
    for (int i = 0; i < vehicle_num; ++i) {
        ofs << " " << i + 1 << " 900000";
        if (i == vehicle_num - 1)
            ofs << ";\n\n";
        else
            ofs << std::endl;
    }

    ofs << "param cap :=" << std::endl;
    for (int i = 0; i < vehicle_num; ++i) {
        ofs << " " << i + 1 << " 3";
        if (i == vehicle_num - 1)
            ofs << ";\n\n";
        else
            ofs << std::endl;
    }

    ofs << "param sp : ";
    for (int i = 0; i < 2 * (vehicle_num + customer_num); ++i)
        ofs << i + 1 << " ";
    ofs << ":=" << std::endl;

    std::vector<int> points;
    for (int i = 0; i < vehicle_num; ++i)
        points.push_back(vehicles[i]->origin);
    for (int i = 0; i < vehicle_num; ++i)
        points.push_back(vehicles[i]->dest);
    for (int i = 0; i < customer_num; ++i)
        points.push_back(customers[i]->origin);
    for (int i = 0; i < customer_num; ++i)
        points.push_back(customers[i]->dest);

    for (int i = 0; i < 2 * (vehicle_num + customer_num); ++i) {
        ofs << i + 1 << " ";
        for (int j = 0; j < 2 * (vehicle_num + customer_num); ++j)
            ofs << gtree.search(points[i], points[j]) << " ";
        if (i == 2 * (vehicle_num + customer_num) - 1)
            ofs << ";\n\n";
        else
            ofs << std::endl;
    }

    /* clean up */
    ifs.close();
    ofs.close();
    for (int i = 0; i < vehicle_num; ++i)
        delete vehicles[i];
    for (int i = 0; i < customer_num; ++i)
        delete customers[i];

    return 0;
}
