#pragma once

#include <queue>
#include <map>
#include <metis.h>
#include <mutex>
#include <cmath>
#include <vector>

namespace GTree {

const bool   Optimization_G_tree_Search   = true;
const bool   Optimization_KNN_Cut         = true;
const bool   Optimization_Euclidean_Cut   = false; // removed
const double Unit                         = 0.1;
const double R_earth                      = 6371000.0;
const double PI                           = acos(-1.0);
const int    Naive_Split_Limit            = 33;
const int    INF                          = 0x3fffffff;
const bool   RevE = true; // ReverseEdge: set true for undirectedd; false = directed
const bool   Distance_Offset              = false;
const int    Partition_Part               = 4; // this is fanout?

struct Heap {

    int n = 1;
    std::vector<int> id;
    std::vector<int> iid;
    std::vector<int> a;

    Heap();

    void clear();
    void swap_(int, int);
    void up(int);
    void down(int);
    int  top();
    int  top_id();
    int  size();
    void change(int, int);
    void add(int, int);
    void push(int);
    void draw();
};

void save_vector(const std::vector<int>&);
void load_vector(std::vector<int>&);
void save_vector_vector(const std::vector<std::vector<int>>&);
void load_vector_vector(std::vector<std::vector<int>>&);
void save_vector_pair(const std::vector<std::pair<int, int>>&);
void load_vector_pair(std::vector<std::pair<int, int>>&);
void save_map_int_pair(std::map<int, std::pair<int, int>>&);
void load_map_int_pair(std::map<int, std::pair<int, int>>&);
void save_map_int_int(std::map<int, int>&);
void load_map_int_int(std::map<int, int>&);

struct coor {
    coor(double a = 0.0, double b = 0.0) : x(a), y(b) {}
    double x, y;
};


double coor_dist(const coor&, const coor&);
double Distance_(double, double, double, double);
double Euclidean_Dist(int, int);

struct Graph {
    int n, m;
    int tot;
    std::vector<int> id;
    std::vector<int> head, list, next, cost;

    Graph();
    ~Graph();

    void save();
    void load();
    void add_D(int, int, int);
    void add(int, int, int);
    void init(int, int, int = 1);
    void clear();
    void draw();

    std::vector<int> color;
    std::vector<int> con;

    std::vector<int> Split(Graph**, int);
    std::vector<int> Split_Naive(Graph&, Graph&);
    int Split_Borders(int);

    struct state {
        state(int a = 0, int b = 0, int c = 0) : id(a), len(b), index(c) {}
        int id, len, index;
    }; // Dijkstra

    struct cmp {
        bool operator()(const state &a, const state &b) {
            return a.len > b.len;
        }
    }; // Priority queue

    void dijkstra(int, std::vector<int>&);
    std::vector<int> KNN(int, int, std::vector<int>);
    std::vector<int> find_path(int, int);
    int real_node();

    std::vector<std::vector<int>> K_Near_Dist, K_Near_Order;

    void KNN_init(const std::vector<int>&, int);
    std::vector<int>* KNN_Dijkstra(int);
};

struct Matrix {

    Matrix();
    ~Matrix();

    int n;
    //int **a;
    std::vector<std::vector<int>> a;

    void save();
    void load();
    void cover(int);
    void init(int);
    void clear();
    void floyd();
    void floyd(Matrix&);
    void write();
    Matrix &operator=(const Matrix&);

};

struct Node {
    Node();

    Graph G;

    int part;
    int n, father, deep;
    int *son;
    int catch_id, catch_bound;
    int min_border_dist;

    std::vector<int> color;
    std::vector<int> border_in_father;
    std::vector<int> border_in_son;
    std::vector<int> border_id;
    std::vector<int> border_id_innode;
    std::vector<int> border_son_id;
    std::vector<int> path_record;
    std::vector<int> catch_dist;

    std::vector<std::pair<int, int>> min_car_dist;
    std::map<int, std::pair<int, int>> borders;

    Matrix dist, order;

    void save();
    void load();
    void init(int);
    void clear();
    void make_border_edge();
    void write();
};

struct G_Tree { // Here we go!
    int root;
    std::vector<int> id_in_node;
    std::vector<std::vector<int>> car_in_node;
    std::vector<int> car_offset;

    int node_tot, node_size;
    //Node *node;
    std::vector<Node> node;

    // In case multiple threads are using the same gtree
    static std::mutex gtmx;

    void save();
    void load();
    void write();
    void add_border(int, int, int);
    void make_border(int, const std::vector<int>&);
    int  partition_root(int = 1);
    void build(const Graph &g, int = 1, int = 1);
    void build_dist1(int = 1);
    void build_dist2(int = 1);
    void build_border_in_father_son();
    void push_borders_up(int, std::vector<int>&, int);
    void push_borders_up_catch(int, int = INF);
    void push_borders_down_catch(int, int, int = INF);
    void push_borders_brother_catch(int, int, int = INF);
    void push_borders_up_path(int, std::vector<int>&);
    int  find_LCA(int, int);
    int  search(int, int);
    int  search_catch(int, int, int = INF);
    int  find_path(int, int, std::vector<int>&);
    int  real_border_number(int);
    void find_path_border(int, int, int, std::vector<int>&, int);
    std::vector<int> KNN(int, int, std::vector<int>);
    std::vector<int> KNN(int, int, std::vector<int>, std::vector<int>);
    std::vector<int> KNN_bound(int, int, std::vector<int>, int);
    std::vector<int> KNN_bound(int, int, std::vector<int>, int, std::vector<int>);
    std::vector<int> Range(int, int, std::vector<int>);
    std::vector<int> Range(int, int, std::vector<int>, std::vector<int>);
    void add_car(int, int);
    void del_car(int, int);
    void change_car_offset(int, int);
    int  get_car_offset(int);

    int begin[10000];
    int end[10000];

    bool push_borders_up_add_min_car_dist(int, int);
    bool push_borders_up_del_min_car_dist(int, int);
    int push_borders_up_catch_KNN_min_dist_car(int);
    std::vector<int> KNN_min_dist_car(int, int);
    bool check_min_car_dist(int = -1);
};

struct Wide_KNN_ {
    int S, K, bound, dist_now, tot;
    std::priority_queue<std::pair<int, int>> KNN;
    double Euclid;
    int Real_Dist;
    std::vector<int> re;

    void init(int, int);
    bool update(std::vector<std::pair<double, std::pair<int, int>>>);
    std::vector<int> result();
};

void init();
void read(const std::string &);
void save(G_Tree&);
void load();
void load(const std::string &);
void setAdMem(long long);

G_Tree get();
Graph getG();


} // End namespace GTree
