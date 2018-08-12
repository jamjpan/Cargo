#include <iostream>
#include <cstring>
#include <algorithm>
#include <mutex>

#include "gtree/gtree.h"

namespace GTree {

long long           Additional_Memory;
long long           ts, te;
static int          rootp = 0;
std::vector<coor>   coordinate;
Wide_KNN_           Wide_KNN;
G_Tree              tree;
Graph               G;

std::mutex G_Tree::gtmx;

G_Tree get() {
    return tree;
}

Graph getG() {
    return G;
}

Heap::Heap() {
    clear();
}

void Heap::clear() {
     n = 1;
     id.clear();
     iid.clear();
     a.clear();
     iid.push_back(0);
     a.push_back(0);
}

void Heap::swap_(int x, int y) {
    std::swap(a[x], a[y]);
    id[iid[x]] = y;
    id[iid[y]] = x;
    std::swap(iid[x], iid[y]);
}

void Heap::up(int x) {
    while (x > 1) {
        if (a[x >> 1] < a[x]) {
            swap_(x >> 1, x);
            x >>= 1;
        } else
            return;
    }
}

void Heap::down(int x) {
    while ((x << 1) < n) {
        int k;
        if ((x << 1) + 1 >= n || a[x << 1] > a[(x << 1) + 1])
            k = x << 1;
        else
            k = (x << 1) + 1;
        if (a[x] < a[k]) {
            swap_(x, k);
            x = k;
        } else
            return;
    }
}

int Heap::top() {
    return a[1];
}

int Heap::top_id() {
    return iid[1];
}

int Heap::size() {
    return n;
}

void Heap::change(int x, int num) {
    a[id[x]] = num;
    up(id[x]);
    down(id[x]);
}

void Heap::add(int x, int num) {
    a[id[x]] += num;
    up(id[x]);
    down(id[x]);
}

void Heap::push(int num) {
    id.push_back(n);
    iid.push_back(n - 1);
    a.push_back(num);
    n++;
    up(n - 1);
}

void Heap::draw() {
    // printf("Heap:%d n=%d\n", this, n - 1);

    printf("a:");
    for (auto i : a)
        printf(" %d", i);
    std::cout << std::endl;

    printf("id:");
    for (auto i : id)
        printf(" %d", i);
    std::cout << std::endl;

    printf("iid:");
    for (auto i : iid)
        printf(" %d", i);
    std::cout << std::endl;

    printf("draw_end\n");
}


void save_vector(const std::vector<int> &v) {
    printf("%d ", (int)v.size());
    for (auto i : v)
        printf("%d ", i);
    std::cout << std::endl;
}

void load_vector(std::vector<int> &v) {
    v.clear();
    int n, i, j;
    scanf("%d", &n);
    for (i = 0; i < n; i++) {
        scanf("%d", &j);
        v.push_back(j);
    }
}

void save_vector_vector(const std::vector<std::vector<int>> &v) {
    printf("%d\n", (int)v.size());
    for (int i = 0; i < (int)v.size(); i++)
        save_vector(v[i]);
    std::cout << std::endl;
}

void load_vector_vector(std::vector<std::vector<int>> &v) {
    v.clear();
    int n, i;
    scanf("%d", &n);
    std::vector<int> ls;
    for (i = 0; i < n; i++) {
        load_vector(ls);
        v.push_back(ls);
    }
}

void save_vector_pair(const std::vector<std::pair<int, int>> &v) {
    printf("%d ", (int)v.size());
    for (auto i : v)
        printf("%d %d ", i.first, i.second);
    std::cout << std::endl;
}

void load_vector_pair(std::vector<std::pair<int, int>> &v) {
    v.clear();
    int n, i, j, k;
    scanf("%d", &n);
    for (i = 0; i < n; i++) {
        scanf("%d%d", &j, &k);
        v.push_back(std::make_pair(j, k));
    }
}

void save_map_int_pair(std::map<int, std::pair<int, int>> &h) {
    printf("%d\n", (int)h.size());
    for (auto i : h)
        printf("%d %d %d\n", i.first, i.second.first, i.second.second);
}

void load_map_int_pair(std::map<int, std::pair<int, int>> &h) {
    int n, i, j, k, l;
    scanf("%d", &n);
    for (i = 0; i < n; i++) {
        scanf("%d%d%d", &j, &k, &l);
        h[j] = std::make_pair(k, l);
    }
}

void save_map_int_int(std::map<int, int> &h) {
    printf("%d\n", (int)h.size());
    for (auto i : h)
        printf("%d %d\n", i.first, i.second);
}

void load_map_int_int(std::map<int, int> &h) {
    int n, i, j, k;
    scanf("%d", &n);
    for (i = 0; i < n; i++) {
        scanf("%d%d", &j, &k);
        h[j] = k;
    }
}

double coor_dist(const coor &a, const coor &b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double Distance_(double a1, double b1, double a2, double b2) {
    double x, y, z, AB, A_B;
    if (b1 < 0)
        b1 = abs(b1) + 180;
    if (b2 < 0)
        b2 = abs(b2) + 180;
    a1 = a1 * PI / 180;
    b1 = b1 * PI / 180;
    a2 = a2 * PI / 180;
    b2 = b2 * PI / 180;
    x = R_earth * cos(b1) * cos(a1) - R_earth * cos(b2) * cos(a2);
    y = R_earth * cos(b1) * sin(a1) - R_earth * cos(b2) * sin(a2);
    z = R_earth * sin(b1) - R_earth * sin(b2);
    AB = sqrt(x * x + y * y + z * z);
    A_B = 2 * asin(AB / (2 * R_earth)) * R_earth;
    return A_B;
}

double Euclidean_Dist(int S, int T) {
    return Distance_(coordinate[S].x, coordinate[S].y, coordinate[T].x,
                     coordinate[T].y) / Unit;
}

Graph::Graph() {
    clear();
}

Graph::~Graph() {
    clear();
}

void Graph::save() {
    printf("%d %d %d\n", n, m, tot);
    save_vector(id);
    save_vector(head);
    save_vector(list);
    save_vector(next);
    save_vector(cost);
}

void Graph::load() {
    scanf("%d%d%d", &n, &m, &tot);
    load_vector(id);
    load_vector(head);
    load_vector(list);
    load_vector(next);
    load_vector(cost);
}

void Graph::add_D(int a, int b, int c) {
    tot++;
    list[tot] = b;
    cost[tot] = c;
    next[tot] = head[a];
    head[a] = tot;
}

void Graph::add(int a, int b, int c) {
    add_D(a, b, c);
    add_D(b, a, c);
}

void Graph::init(int N, int M, int t) {
    clear();
    n = N;
    m = M;
    tot = t;
    head = std::vector<int>(N);
    id = std::vector<int>(N);
    list = std::vector<int>(M * 2 + 2);
    next = std::vector<int>(M * 2 + 2);
    cost = std::vector<int>(M * 2 + 2);
}

void Graph::clear() {
    n = m = tot = 0;
    head.clear();
    list.clear();
    next.clear();
    cost.clear();
    id.clear();
}

void Graph::draw() {
    // printf("Graph:%d n=%d m=%d\n", this, n, m);
    for (int i = 0; i < n; i++)
        std::cout << id[i] << ' ';
    std::cout << std::endl;
    for (int i = 0; i < n; i++) {
        printf("%d:", i);
        for (int j = head[i]; j; j = next[j])
            printf(" %d", list[j]);
        std::cout << std::endl;
    }
    std::cout << "Graph_draw_end" << std::endl;
}

std::vector<int> Graph::Split(Graph *G[], int nparts) {
    std::vector<int> color(n);
    int i;
    if (n == nparts)
        for (i = 0; i < n; i++)
            color[i] = i;
    else {
        idx_t options[METIS_NOPTIONS]; // idx_t comes from METIS
        memset(options, 0, sizeof(options));
        {
            METIS_SetDefaultOptions(options);
            options[METIS_OPTION_PTYPE] = METIS_PTYPE_KWAY;    // _RB
            options[METIS_OPTION_OBJTYPE] = METIS_OBJTYPE_CUT; // _VOL
            options[METIS_OPTION_CTYPE] = METIS_CTYPE_SHEM;    // _RM
            options[METIS_OPTION_IPTYPE] =
                METIS_IPTYPE_RANDOM; // _GROW _EDGE _NODE
            options[METIS_OPTION_RTYPE] =
                METIS_RTYPE_FM; // _GREEDY _SEP2SIDED _SEP1SIDED
            // options[METIS_OPTION_NCUTS] = 1;
            // options[METIS_OPTION_NITER] = 10;
            /* balance factor, used to be 500 */
            options[METIS_OPTION_UFACTOR] = 500;
            // options[METIS_OPTION_MINCONN];
            options[METIS_OPTION_CONTIG] = 1;
            // options[METIS_OPTION_SEED];
            options[METIS_OPTION_NUMBERING] = 0;
            // options[METIS_OPTION_DBGLVL] = 0;
        }
        idx_t nvtxs = n;
        idx_t ncon = 1;
        // transform
        int *xadj = new idx_t[n + 1];
        int *adj = new idx_t[n + 1];
        int *adjncy = new idx_t[tot - 1];
        int *adjwgt = new idx_t[tot - 1];
        int *part = new idx_t[n];

        int xadj_pos = 1;
        int xadj_accum = 0;
        int adjncy_pos = 0;

        // xadj, adjncy, adjwgt
        xadj[0] = 0;
        for (int i = 0; i < n; i++) {
            // init node map

            /*int fanout = Nodes[nid].adjnodes.size();
            for (int j = 0; j < fanout; j++) {
                int enid = Nodes[nid].adjnodes[j];
                // ensure edges within
                if (nset.find(enid) != nset.end()) {
                    xadj_accum++;
                    adjncy[adjncy_pos] = enid;
                    adjwgt[adjncy_pos] = Nodes[nid].adjweight[j];
                    adjncy_pos++;
                }
            }

            xadj[xadj_pos++] = xadj_accum;*/
            for (int j = head[i]; j; j = next[j]) {
                int enid = list[j];
                xadj_accum++;
                adjncy[adjncy_pos] = enid;
                adjwgt[adjncy_pos] = cost[j];
                adjncy_pos++;
            }
            xadj[xadj_pos++] = xadj_accum;
        }

        // adjust nodes number started by 0

        // adjwgt -> 1
        for (int i = 0; i < adjncy_pos; i++)
            adjwgt[i] = 1;

        // nparts
        int objval = 0;
        // METIS
        METIS_PartGraphKway(&nvtxs, &ncon, xadj, adjncy, NULL, NULL, adjwgt,
                            &nparts, NULL, NULL, options, &objval, part);
        for (int i = 0; i < n; i++)
            color[i] = part[i];
        delete[] xadj;
        delete[] adj;
        delete[] adjncy;
        delete[] adjwgt;
        delete[] part;
    }

    int j;
    std::vector<int> new_id;
    std::vector<int> tot(nparts, 0), m(nparts, 0);
    for (i = 0; i < n; i++)
        new_id.push_back(tot[color[i]]++);
    for (i = 0; i < n; i++)
        for (j = head[i]; j; j = next[j])
            if (color[list[j]] == color[i])
                m[color[i]]++;
    for (int t = 0; t < nparts; t++) {
        (*G[t]).init(tot[t], m[t]);
        for (i = 0; i < n; i++)
            if (color[i] == t)
                for (j = head[i]; j; j = next[j])
                    if (color[list[j]] == color[i])
                        (*G[t]).add_D(new_id[i], new_id[list[j]], cost[j]);
    }
    for (i = 0; i < (int)tot.size(); i++)
        tot[i] = 0;
    for (i = 0; i < n; i++)
        (*G[color[i]]).id[tot[color[i]]++] = id[i];
    return color;
}

std::vector<int> Graph::Split_Naive(Graph &G1, Graph &G2) {
    color.clear();
    con.clear();

    int i, j, k = n / 2, l;
    for (i = 0; i < n; i++) {
        if (n - i == k) { /* || (k && rand() &1) */
            color.push_back(1);
            k--;
        } else
            color.push_back(0);
    }

    for (i = 0; i < n; i++) {
        k = 0;
        for (j = head[i]; j; j = next[j])
            if (color[list[j]] ^ color[i])
                k++;
        con.push_back(k);
    }

    Heap q[2];
    int ans = 0;

    for (i = 0; i < n; i++) {
        k = 0;
        for (j = head[i]; j; j = next[j])
            if (color[list[j]] ^ color[i])
                ans++, k++; // comma operator!
            else
                k--;
        q[color[i]].push(k);
        q[color[i] ^ 1].push(-k - INF);
    }

    ans /= 2;
    // for(i=0;i<n;i++)printf("%d %d %d\n",i,color[i],con[i]);

    while (q[0].top() + q[1].top() > 0) {
        int save_ans = ans;
        for (l = 0; l < 2; l++) {
            i = q[l].top_id();
            k = 0;
            for (j = head[i]; j; j = next[j]) {
                if (color[list[j]] ^ color[i])
                    k--;
                else
                    k++;
                q[color[i] ^ 1].add(list[j], -2);
                q[color[i]].add(list[j], 2);
            }
            ans += k;
            color[i] ^= 1;
            q[color[i]].change(i, k);
            q[color[i] ^ 1].change(i, -k - INF);
        }
        if (save_ans == ans)
            break;
    }

    std::vector<int> new_id;
    int tot0 = 0, tot1 = 0;
    int m1 = 0, m0 = 0;
    for (i = 0; i < n; i++) {
        if (color[i] == 0)
            new_id.push_back(tot0++);
        else
            new_id.push_back(tot1++);
    }
    for (i = 0; i < n; i++)
        for (j = head[i]; j; j = next[j])
            if (1 ^ color[list[j]] ^ color[i]) {
                if (color[i] == 0)
                    m0++;
                else
                    m1++;
            }

    G1.init(tot0, m0);
    for (i = 0; i < n; i++)
        if (color[i] == 0)
            for (j = head[i]; j; j = next[j])
                if (color[list[j]] == color[i])
                    G1.add_D(new_id[i], new_id[list[j]], cost[j]);
    G2.init(tot1, m1);
    for (i = 0; i < n; i++)
        if (color[i] == 1)
            for (j = head[i]; j; j = next[j])
                if (color[list[j]] == color[i])
                    G2.add_D(new_id[i], new_id[list[j]], cost[j]);
    tot0 = tot1 = 0;
    for (i = 0; i < n; i++) {
        if (color[i] == 0)
            G1.id[tot0++] = id[i];
        else
            G2.id[tot1++] = id[i];
    }
    return color;
}

int Graph::Split_Borders(int nparts) {
    if (n < Naive_Split_Limit)
        return Naive_Split_Limit;

    idx_t options[METIS_NOPTIONS];
    memset(options, 0, sizeof(options));
    {
        METIS_SetDefaultOptions(options);
        options[METIS_OPTION_PTYPE] = METIS_PTYPE_KWAY;    // _RB
        options[METIS_OPTION_OBJTYPE] = METIS_OBJTYPE_CUT; // _VOL
        options[METIS_OPTION_CTYPE] = METIS_CTYPE_SHEM;    // _RM
        options[METIS_OPTION_IPTYPE] =
            METIS_IPTYPE_RANDOM; // _GROW _EDGE _NODE
        options[METIS_OPTION_RTYPE] =
            METIS_RTYPE_FM; // _GREEDY _SEP2SIDED _SEP1SIDED
        // options[METIS_OPTION_NCUTS] = 1;
        // options[METIS_OPTION_NITER] = 10;
        /* balance factor, used to be 500 */
        options[METIS_OPTION_UFACTOR] = 500;
        // options[METIS_OPTION_MINCONN];
        options[METIS_OPTION_CONTIG] = 1;
        // options[METIS_OPTION_SEED];
        options[METIS_OPTION_NUMBERING] = 0;
        // options[METIS_OPTION_DBGLVL] = 0;
    }
    idx_t nvtxs = n;
    idx_t ncon = 1;
    std::vector<int> color(n);
    int *xadj = new idx_t[n + 1];
    int *adj = new idx_t[n + 1];
    int *adjncy = new idx_t[tot - 1];
    int *adjwgt = new idx_t[tot - 1];
    int *part = new idx_t[n];

    int xadj_pos = 1;
    int xadj_accum = 0;
    int adjncy_pos = 0;

    xadj[0] = 0;
    int i = 0;
    for (int i = 0; i < n; i++) {
        for (int j = head[i]; j; j = next[j]) {
            int enid = list[j];
            xadj_accum++;
            adjncy[adjncy_pos] = enid;
            adjwgt[adjncy_pos] = cost[j];
            adjncy_pos++;
        }
        xadj[xadj_pos++] = xadj_accum;
    }

    for (int i = 0; i < adjncy_pos; i++)
        adjwgt[i] = 1;

    int objval = 0;
    METIS_PartGraphKway(&nvtxs, &ncon, xadj, adjncy, NULL, NULL, adjwgt,
                        &nparts, NULL, NULL, options, &objval, part);

    for (int i = 0; i < n; i++)
        color[i] = part[i];

    int j, re = 0;
    for (i = 0; i < n; i++)
        for (j = head[i]; j; j = next[j])
            if (color[i] != color[list[j]]) {
                re++;
                break;
            }

    delete[] xadj;
    delete[] adj;
    delete[] adjncy;
    delete[] adjwgt;
    delete[] part;
    return re;
}

void Graph::dijkstra(int S, std::vector<int> &dist) {
    std::priority_queue<state, std::vector<state>, cmp> q;
    state now;
    int i;
    dist.clear();
    while ((int)dist.size() < n)
        dist.push_back(INF);
    q.push(state(S, 0));
    while (q.size()) {
        now = q.top();
        q.pop();
        if (dist[now.id] == INF) {
            dist[now.id] = now.len;
            for (i = head[now.id]; i; i = next[i])
                if (dist[list[i]] == INF)
                    q.push(state(list[i], dist[now.id] + cost[i]));
        }
    }
}

std::vector<int> Graph::KNN(int S, int K, std::vector<int> T) {
    int i;
    std::vector<int> dist(n, INF), Cnt(n, 0);
    for (i = 0; i < (int)T.size(); i++)
        Cnt[T[i]]++;
    std::priority_queue<state, std::vector<state>, cmp> q;
    state now;
    q.push(state(S, 0));
    int bound = INF, cnt = 0;

    while (q.size() && cnt < K) {
        now = q.top();
        q.pop();
        if (dist[now.id] == INF) {
            dist[now.id] = now.len;
            cnt += Cnt[now.id];
            if (cnt >= K)
                bound = now.len;
            for (i = head[now.id]; i; i = next[i])
                if (dist[list[i]] == INF)
                    q.push(state(list[i], dist[now.id] + cost[i]));
        }
    }

    std::vector<int> re;
    for (int i = 0; i < (int)T.size() && (int)re.size() < K; i++)
        if (dist[T[i]] <= bound)
            re.push_back(i);
    return re;
}

std::vector<int> Graph::find_path(int S, int T) {
    std::vector<int> dist, re, last;
    std::priority_queue<state, std::vector<state>, cmp> q;
    state now;
    int i;
    dist.clear();
    last.clear();
    re.clear();

    while ((int)dist.size() < n) {
        dist.push_back(INF);
        last.push_back(0);
    }

    q.push(state(S, 0));

    while (q.size()) {
        now = q.top();
        q.pop();
        if (dist[now.id] == INF) {
            dist[now.id] = now.len;
            for (i = head[now.id]; i; i = next[i]) {
                if (dist[list[i]] == INF)
                    q.push(state(list[i], dist[now.id] + cost[i]));
                if (dist[list[i]] + cost[i] == dist[now.id])
                    last[now.id] = list[i];
            }
        }
    }

    if (dist[T] == INF)
        return re;
    else {
        for (i = T; i != S; i = last[i])
            re.push_back(i);
        re.push_back(S);
        std::reverse(re.begin(), re.end());
        return re;
    }
}

int Graph::real_node() {
    int ans = 0;
    for (int i = 0; i < n; i++) {
        int k = 0;
        for (int j = head[i]; j; j = next[j])
            k++;
        if (k != 2)
            ans++;
    }
    return ans;
}

void Graph::KNN_init(const std::vector<int> &S, int K) {
    std::priority_queue<state, std::vector<state>, cmp> q;
    state now;
    int i;
    std::vector<int> empty;
    K_Near_Dist.clear();
    K_Near_Order.clear();
    while ((int)K_Near_Dist.size() < n) {
        K_Near_Dist.push_back(empty);
        K_Near_Order.push_back(empty);
    }
    for (int i = 0; i < (int)S.size(); i++)
        q.push(state(S[i], 0, i));
    while (q.size()) {
        now = q.top();
        q.pop();
        if ((int)K_Near_Dist[now.id].size() < K) {
            K_Near_Dist[now.id].push_back(now.len);
            K_Near_Order[now.id].push_back(now.index);
            for (i = head[now.id]; i; i = next[i])
                if ((int)K_Near_Dist[list[i]].size() < K)
                    q.push(state(list[i], now.len + cost[i]));
        }
    }
}

std::vector<int>* Graph::KNN_Dijkstra(int S) {
    return &K_Near_Order[S];
}

//Matrix::Matrix() : n(0), a(NULL) {}
Matrix::Matrix() : n(0) {}

Matrix::~Matrix() {
    clear();
}

void Matrix::save() {
    printf("%d\n", n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++)
            printf("%d ", a[i][j]);
        printf("\n");
    }
}

void Matrix::load() {
    scanf("%d", &n);
    //a = new int *[n];
    a.resize(n);
    for (int i = 0; i < n; i++)
        //a[i] = new int[n];
        a[i].resize(n);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            scanf("%d", &a[i][j]);
}

void Matrix::cover(int x) {
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            a[i][j] = x;
}

void Matrix::init(int N) {
    clear();
    n = N;
    //a = new int *[n];
    a.resize(n);
    for (int i = 0; i < n; i++)
        //a[i] = new int[n];
        a[i].resize(n);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            a[i][j] = INF;
    for (int i = 0; i < n; i++)
        a[i][i] = 0;
}

void Matrix::clear() {
//    for (int i = 0; i < n; i++)
//        delete[] a[i];
//    delete[] a;
}

void Matrix::floyd() {
    int i, j, k;
    for (k = 0; k < n; k++)
        for (i = 0; i < n; i++)
            for (j = 0; j < n; j++)
                if (a[i][j] > a[i][k] + a[k][j])
                    a[i][j] = a[i][k] + a[k][j];
}

void Matrix::floyd(Matrix &order) {
    int i, j, k;
    for (k = 0; k < n; k++)
        for (i = 0; i < n; i++)
            for (j = 0; j < n; j++)
                if (a[i][j] > a[i][k] + a[k][j]) {
                    a[i][j] = a[i][k] + a[k][j];
                    order.a[i][j] = k;
                }
}

void Matrix::write() {
    printf("n=%d\n", n);
    for (int i = 0; i < n; i++, std::cout << std::endl)
        for (int j = 0; j < n; j++)
            printf("%d ", a[i][j]);
}

Matrix& Matrix::operator=(const Matrix &m) {
    if (this != (&m)) {
        init(m.n);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                a[i][j] = m.a[i][j];
    }
    return *this;
}

Node::Node() {
    //clear();
    part = n = father = deep = 0;
    catch_id = -1;
}

void Node::save() {
    printf("%d %d %d %d %d %d %d\n", n, father, part, deep, catch_id,
           catch_bound, min_border_dist);

    for (int i = 0; i < part; i++)
        printf("%d ", son[i]);
    std::cout << std::endl;

    save_vector(color);
    dist.save();
    order.save();
    save_map_int_pair(borders);
    save_vector(border_in_father);
    save_vector(border_in_son);
    save_vector(border_id);
    save_vector(border_id_innode);
    save_vector(path_record);
    save_vector(catch_dist);
    save_vector_pair(min_car_dist);
}

void Node::load() {
    scanf("%d%d%d%d%d%d%d", &n, &father, &part, &deep, &catch_id,
          &catch_bound, &min_border_dist);
    //if (son != NULL)
    //    delete[] son;
    son = new int[part];
    for (int i = 0; i < part; i++)
        scanf("%d", &son[i]);
    load_vector(color);
    dist.load();
    order.load();
    load_map_int_pair(borders);
    load_vector(border_in_father);
    load_vector(border_in_son);
    load_vector(border_id);
    load_vector(border_id_innode);
    load_vector(path_record);
    load_vector(catch_dist);
    load_vector_pair(min_car_dist);
}

void Node::init(int n) {
    part = n;
    son = new int[n];
    for (int i = 0; i < n; i++)
        son[i] = 0;
}

void Node::clear() {
    part = n = father = deep = 0;
    delete[] son;
    dist.clear();
    order.clear();
    G.clear();
    color.clear();
    borders.clear();
    border_in_father.clear();
    border_in_son.clear();
    border_id.clear();
    border_id_innode.clear();
    catch_dist.clear();
    catch_id = -1;
}

void Node::make_border_edge() {
    int i, j;
    std::map<int, std::pair<int, int>>::iterator iter;
    for (iter = borders.begin(); iter != borders.end(); iter++) {
        i = iter->second.second;
        for (j = G.head[i]; j; j = G.next[j])
            if (color[i] != color[G.list[j]]) {
                int id1, id2;
                id1 = iter->second.first;
                id2 = borders[G.id[G.list[j]]].first;
                if (dist.a[id1][id2] > G.cost[j]) {
                    dist.a[id1][id2] = G.cost[j];
                    order.a[id1][id2] = -1;
                }
            }
    }
}

void Node::write() {
    printf("n=%d deep=%d father=%d", n, deep, father);
    printf(" son:(");
    for (int i = 0; i < part; i++) {
        if (i > 0)
            printf(" ");
        printf("%d", son[i]);
    }
    printf(")\n");
    // G.draw();
    printf("color:");
    for (int i = 0; i < (int)color.size(); i++)
        printf("%d ", color[i]);
    std::cout << std::endl;
    printf("dist:\n");
    dist.write();
    printf("order:\n");
    order.write();
    printf("borders:");
    for (auto iter = borders.begin(); iter != borders.end(); iter++)
        printf("(%d,%d,%d)", iter->first, iter->second.first,
               iter->second.second);
    std::cout << std::endl;
    printf("border_id");
    for (int i = 0; i < (int)borders.size(); i++)
        printf("(%d,%d)", i, border_id[i]);
    printf("\n");
    printf("border_in_father");
    for (int i = 0; i < (int)borders.size(); i++)
        printf("(%d,%d)", i, border_in_father[i]);
    printf("\n");
    printf("border_in_son");
    for (int i = 0; i < (int)borders.size(); i++)
        printf("(%d,%d)", i, border_in_son[i]);
    printf("\n");
    printf("catch_dist ");
    for (int i = 0; i < (int)catch_dist.size(); i++)
        printf("(%d,%d)", i, catch_dist[i]);
    printf("\n");
    printf("min_car_dist ");
    for (int i = 0; i < (int)min_car_dist.size(); i++)
        printf("(i:%d,D:%d,id:%d)", i, min_car_dist[i].first,
               min_car_dist[i].second);
    printf("\n");
}

void G_Tree::save() {
    G.save();
    printf("%d %d %d\n", root, node_tot, node_size);

    //SANITY
    std::cerr << node_size << std::endl;
    std::cerr << id_in_node.size() << std::endl;

    save_vector(id_in_node);
    save_vector_vector(car_in_node);
    save_vector(car_offset);
    for (int i = 0; i < node_size; i++) {
        printf("\n");
        node[i].save();
    }
}

void G_Tree::load() {
    G.load();
    scanf("%d%d%d", &root, &node_tot, &node_size);
    load_vector(id_in_node);
    load_vector_vector(car_in_node);
    load_vector(car_offset);
    //node = new Node[G.n * 2 + 2];
    node.resize(G.n*2 + 2);
    for (int i = 0; i < node_size; i++)
        node[i].load();
}

void G_Tree::write() {
    printf("root=%d node_tot=%d\n", root, node_tot);
    for (int i = 1; i < node_tot; i++) {
        printf("node:%d\n", i);
        node[i].write();
        std::cout << std::endl;
    }
}

void G_Tree::add_border(int x, int id, int id2) {
    std::map<int, std::pair<int, int>>::iterator iter;
    iter = node[x].borders.find(id);
    if (iter == node[x].borders.end()) {
        std::pair<int, int> second =
            std::make_pair((int)node[x].borders.size(), id2);
        node[x].borders[id] = second;
    }
}

void G_Tree::make_border(int x, const std::vector<int> &color) {
    for (int i = 0; i < node[x].G.n; i++) {
        int id = node[x].G.id[i];
        for (int j = node[x].G.head[i]; j; j = node[x].G.next[j])
            if (color[i] != color[node[x].G.list[j]]) {
                add_border(x, id, i);
                break;
            }
    }
}

int G_Tree::partition_root(int x) {
    if ((long long)node[x].G.n * node[x].G.n <= Additional_Memory)
        return node[x].G.n;
    int l = 2, r = std::max(2, (int)sqrt(Additional_Memory)), mid;
    while (l < r) {
        mid = (l + r + 1) >> 1;
        int num = node[x].G.Split_Borders(mid);
        if (num * num > Additional_Memory)
            r = mid - 1;
        else
            l = mid;
    }
    return l;
}

void G_Tree::build(const Graph &g, int x, int f) {
    if (x == 1) {
        //node = new Node[G.n * 2 + 2];
        node.resize(G.n*2 + 2);
        node_size = G.n * 2;
        node_tot = 2;
        root = 1;
        node[x].deep = 1;
        node[1].G = g;
    } else {
        node[x].deep = node[node[x].father].deep + 1;
    }
    node[x].n = node[x].G.n;
    if (x == root && Optimization_G_tree_Search) {
        node[x].init(partition_root(x));
        printf("root's part:%d\n", node[x].part);
        rootp = node[x].part;
    } else if (node[x].G.n < Naive_Split_Limit) {
        node[x].init(node[x].n);
    } else
        node[x].init(Partition_Part);

    if (node[x].n > 50)
        printf("x=%d deep=%d n=%d ", x, node[x].deep, node[x].G.n);
    if (node[x].n > f) {
        int top = node_tot;
        for (int i = 0; i < node[x].part; i++) {
            node[x].son[i] = top + i;
            node[top + i].father = x;
        }
        node_tot += node[x].part;

        Graph **graph;
        graph = new Graph *[node[x].part];

        for (int i = 0; i < node[x].part; i++)
            graph[i] = &node[node[x].son[i]].G;

        node[x].color = node[x].G.Split(graph, node[x].part);
        delete[] graph;
        make_border(x, node[x].color);
        if (node[x].n > 50)
            printf("border=%d\n", (int)node[x].borders.size());
        std::map<int, std::pair<int, int>>::iterator iter;
        for (iter = node[x].borders.begin(); iter != node[x].borders.end();
             iter++) {
            // printf("(%d,%d,%d)",iter->first,iter->second.first,iter->second.second);
            node[x].color[iter->second.second] =
                -node[x].color[iter->second.second] - 1;
        }
        // cout<<endl;
        std::vector<int> tot(node[x].part, 0);
        for (int i = 0; i < node[x].n; i++) {
            if (node[x].color[i] < 0) {
                node[x].color[i] = -node[x].color[i] - 1;
                add_border(node[x].son[node[x].color[i]], node[x].G.id[i],
                           tot[node[x].color[i]]);
            }
            tot[node[x].color[i]]++;
        }
        for (int i = 0; i < node[x].part; i++)
            build(g, node[x].son[i]);
    } else if (node[x].n > 50)
        std::cout << std::endl;
    node[x].dist.init(node[x].borders.size());
    node[x].order.init(node[x].borders.size());
    node[x].order.cover(-INF);
    if (x == 1) {
        for (int i = 1; i < std::min(1000, node_tot - 1); i++)
            if (node[i].n > 50) {
                printf("x=%d deep=%d n=%d ", i, node[i].deep, node[i].G.n);
                printf("border=%d real_border=%d\n", (int)node[i].borders.size(),
                       real_border_number(i));
            }
        printf("begin_build_border_in_father_son\n");
        build_border_in_father_son();
        printf("begin_build_dist\n");
        build_dist1(root);
        printf("begin_build_dist2\n");
        build_dist2(root);
        id_in_node.clear();
        for (int i = 0; i < node[root].G.n; i++)
            id_in_node.push_back(-1);
        for (int i = 1; i < node_tot; i++)
            if (node[i].G.n == 1)
                id_in_node[node[i].G.id[0]] = i;
        for (int i = 1; i <= node_tot; i++) {
            node[i].catch_id = -1;
            for (int j = 0; j < (int)node[i].borders.size(); j++) {
                node[i].catch_dist.push_back(0);
                node[i].min_car_dist.push_back(std::make_pair(INF, -1));
            }
        }
        {
            std::vector<int> empty_vector;
            empty_vector.clear();
            car_in_node.clear();
            for (int i = 0; i < G.n; i++)
                car_in_node.push_back(empty_vector);
        }
    }
}

void G_Tree::build_dist1(int x) {
    for (int i = 0; i < node[x].part; i++)
        if (node[x].son[i])
            build_dist1(node[x].son[i]);
    if (node[x].son[0]) {
        node[x].make_border_edge();
        node[x].dist.floyd(node[x].order);
    } else {
        ;
    }
    if (node[x].father) {
        int y = node[x].father, i, j;
        std::map<int, std::pair<int, int>>::iterator x_iter1, y_iter1;
        std::vector<int> id_in_fa(node[x].borders.size());
        for (x_iter1 = node[x].borders.begin();
             x_iter1 != node[x].borders.end(); x_iter1++) {
            y_iter1 = node[y].borders.find(x_iter1->first);
            if (y_iter1 == node[y].borders.end())
                id_in_fa[x_iter1->second.first] = -1;
            else
                id_in_fa[x_iter1->second.first] = y_iter1->second.first;
        }
        for (i = 0; i < (int)node[x].borders.size(); i++)
            for (j = 0; j < (int)node[x].borders.size(); j++)
                if (id_in_fa[i] != -1 && id_in_fa[j] != -1) {
                    int *p = &node[y].dist.a[id_in_fa[i]][id_in_fa[j]];
                    if ((*p) > node[x].dist.a[i][j]) {
                        (*p) = node[x].dist.a[i][j];
                        node[y].order.a[id_in_fa[i]][id_in_fa[j]] = -3;
                    }
                }
    }
    return;
}

void G_Tree::build_dist2(int x) {
    if (x != root)
        node[x].dist.floyd(node[x].order);
    if (node[x].son[0]) {
        std::vector<int> id_(node[x].borders.size());
        std::vector<int> color_(node[x].borders.size());
        std::map<int, std::pair<int, int>>::iterator iter1, iter2;
        for (iter1 = node[x].borders.begin();
             iter1 != node[x].borders.end(); iter1++) {
            int c = node[x].color[iter1->second.second];
            color_[iter1->second.first] = c;
            int y = node[x].son[c];
            id_[iter1->second.first] = node[y].borders[iter1->first].first;
        }
        for (int i = 0; i < (int)node[x].borders.size(); i++)
            for (int j = 0; j < (int)node[x].borders.size(); j++)
                if (color_[i] == color_[j]) {
                    int y = node[x].son[color_[i]];
                    int *p = &node[y].dist.a[id_[i]][id_[j]];
                    if ((*p) > node[x].dist.a[i][j]) {
                        (*p) = node[x].dist.a[i][j];
                        node[y].order.a[id_[i]][id_[j]] = -2;
                    }
                }
        for (int i = 0; i < node[x].part; i++)
            if (node[x].son[i])
                build_dist2(node[x].son[i]);
    }
}

void G_Tree::build_border_in_father_son() {
    int i, x, y;
    for (x = 1; x < node_tot; x++) {
        for (i = 0; i < (int)node[x].borders.size(); i++)
            node[x].border_id.push_back(0);
        for (i = 0; i < (int)node[x].borders.size(); i++)
            node[x].border_id_innode.push_back(0);
        for (std::map<int, std::pair<int, int>>::iterator iter =
                 node[x].borders.begin();
             iter != node[x].borders.end(); iter++) {
            node[x].border_id[iter->second.first] = iter->first;
            node[x].border_id_innode[iter->second.first] =
                iter->second.second;
        }
        node[x].border_in_father.clear();
        node[x].border_in_son.clear();
        for (i = 0; i < (int)node[x].borders.size(); i++) {
            node[x].border_in_father.push_back(-1);
            node[x].border_in_son.push_back(-1);
        }
        if (node[x].father) {
            y = node[x].father;
            std::map<int, std::pair<int, int>>::iterator iter;
            for (iter = node[x].borders.begin();
                 iter != node[x].borders.end(); iter++) {
                std::map<int, std::pair<int, int>>::iterator iter2;
                iter2 = node[y].borders.find(iter->first);
                if (iter2 != node[y].borders.end())
                    node[x].border_in_father[iter->second.first] =
                        iter2->second.first;
            }
        }
        if (node[x].son[0]) {
            std::map<int, std::pair<int, int>>::iterator iter;
            for (iter = node[x].borders.begin();
                 iter != node[x].borders.end(); iter++) {
                y = node[x].son[node[x].color[iter->second.second]];
                std::map<int, std::pair<int, int>>::iterator iter2;
                iter2 = node[y].borders.find(iter->first);
                if (iter2 != node[y].borders.end())
                    node[x].border_in_son[iter->second.first] =
                        iter2->second.first;
            }
            for (int i = 0; i < (int)node[x].borders.size(); i++)
                node[x].border_son_id.push_back(
                    node[x]
                        .son[node[x].color[node[x].border_id_innode[i]]]);
        }
    }
}

void G_Tree::push_borders_up(int x, std::vector<int> &dist1, int type) {
    if (node[x].father == 0)
        return;
    int y = node[x].father;
    std::vector<int> dist2(node[y].borders.size(), INF);
    for (int i = 0; i < (int)node[x].borders.size(); i++)
        if (node[x].border_in_father[i] != -1)
            dist2[node[x].border_in_father[i]] = dist1[i];
    // printf("dist2:");save_vector(dist2);
    //int **dist = node[y].dist.a;
    std::vector<std::vector<int>> dist = node[y].dist.a;
    // vector<int>begin,end;
    int *begin, *end;
    begin = new int[node[x].borders.size()];
    end = new int[node[y].borders.size()];
    int tot0 = 0, tot1 = 0;
    for (int i = 0; i < (int)dist2.size(); i++) {
        if (dist2[i] < INF)
            begin[tot0++] = i;
        else if (node[y].border_in_father[i] != -1)
            end[tot1++] = i;
    }
    if (type == 0) {
        for (int i = 0; i < tot0; i++) {
            int i_ = begin[i];
            for (int j = 0; j < tot1; j++) {
                int j_ = end[j];
                if (dist2[j_] > dist2[i_] + dist[i_][j_])
                    dist2[j_] = dist2[i_] + dist[i_][j_];
            }
        }
    } else {
        for (int i = 0; i < tot0; i++) {
            int i_ = begin[i];
            for (int j = 0; j < tot1; j++) {
                int j_ = end[j];
                if (dist2[j_] > dist2[i_] + dist[j_][i_])
                    dist2[j_] = dist2[i_] + dist[j_][i_];
            }
        }
    }
    /*
    int bit[2]={0,0xffffffff};
    int i,i_,j,j_,ls1,ls2;
    for(i=0;i<tot0;++i)
    {
    i_=begin[i];
    ls1=dist2[i_];
    for(j=0;j<tot1;++j)
    {
    j_=end[j];
    ls2=ls1+dist[i_][j_];
    dist2[j_]+=((ls2-dist2[j_])&bit[dist2[j_]>ls2]);
    }
    }
    */
    dist1 = dist2;
    delete[] begin;
    delete[] end;
}

void G_Tree::push_borders_up_catch(int x, int bound) {
    if (node[x].father == 0)
        return;
    int y = node[x].father;
    if (node[x].catch_id == node[y].catch_id &&
        bound <= node[y].catch_bound)
        return;
    node[y].catch_id = node[x].catch_id;
    node[y].catch_bound = bound;
    std::vector<int> *dist1 = &node[x].catch_dist,
                     *dist2 = &node[y].catch_dist;
    for (int i = 0; i < (int)(*dist2).size(); i++)
        (*dist2)[i] = INF;
    for (int i = 0; i < (int)node[x].borders.size(); i++)
        if (node[x].border_in_father[i] != -1) {
            if (node[x].catch_dist[i] < bound)
                (*dist2)[node[x].border_in_father[i]] = (*dist1)[i];
            else
                (*dist2)[node[x].border_in_father[i]] = -1;
        }
    //int **dist = node[y].dist.a;
    auto dist = node[y].dist.a;
    int *begin, *end;
    begin = new int[node[x].borders.size()];
    end = new int[node[y].borders.size()];
    int tot0 = 0, tot1 = 0;
    for (int i = 0; i < (int)(*dist2).size(); i++) {
        if ((*dist2)[i] == -1)
            (*dist2)[i] = INF;
        else if ((*dist2)[i] < INF)
            begin[tot0++] = i;
        else if (node[y].border_in_father[i] != -1) {
            if (Optimization_Euclidean_Cut == false ||
                Euclidean_Dist(node[x].catch_id, node[y].border_id[i]) <
                    bound)
                end[tot1++] = i;
        }
    }
    for (int i = 0; i < tot0; i++) {
        int i_ = begin[i];
        for (int j = 0; j < tot1; j++) {
            if ((*dist2)[end[j]] > (*dist2)[i_] + dist[i_][end[j]])
                (*dist2)[end[j]] = (*dist2)[i_] + dist[i_][end[j]];
        }
    }
    delete[] begin;
    delete[] end;
    node[y].min_border_dist = INF;
    for (int i = 0; i < (int)node[y].catch_dist.size(); i++)
        if (node[y].border_in_father[i] != -1)
            node[y].min_border_dist =
                std::min(node[y].min_border_dist, node[y].catch_dist[i]);
}

void G_Tree::push_borders_down_catch(int x, int y, int bound) {
    if (node[x].catch_id == node[y].catch_id &&
        bound <= node[y].catch_bound)
        return;
    node[y].catch_id = node[x].catch_id;
    node[y].catch_bound = bound;
    std::vector<int> *dist1 = &node[x].catch_dist,
                     *dist2 = &node[y].catch_dist;
    for (int i = 0; i < (int)(*dist2).size(); i++)
        (*dist2)[i] = INF;
    for (int i = 0; i < (int)node[x].borders.size(); i++)
        if (node[x].son[node[x].color[node[x].border_id_innode[i]]] == y) {
            if (node[x].catch_dist[i] < bound)
                (*dist2)[node[x].border_in_son[i]] = (*dist1)[i];
            else
                (*dist2)[node[x].border_in_son[i]] = -1;
        }
    //int **dist = node[y].dist.a;
    auto dist = node[y].dist.a;
    int *begin, *end;
    begin = new int[node[y].borders.size()];
    end = new int[node[y].borders.size()];
    int tot0 = 0, tot1 = 0;
    for (int i = 0; i < (int)(*dist2).size(); i++) {
        if ((*dist2)[i] == -1)
            (*dist2)[i] = INF;
        else if ((*dist2)[i] < INF)
            begin[tot0++] = i;
        else {
            if (Optimization_Euclidean_Cut == false ||
                Euclidean_Dist(node[x].catch_id, node[y].border_id[i]) <
                    bound)
                end[tot1++] = i;
        }
    }
    for (int i = 0; i < tot0; i++) {
        int i_ = begin[i];
        for (int j = 0; j < tot1; j++) {
            if ((*dist2)[end[j]] > (*dist2)[i_] + dist[i_][end[j]])
                (*dist2)[end[j]] = (*dist2)[i_] + dist[i_][end[j]];
        }
    }
    delete[] begin;
    delete[] end;
    node[y].min_border_dist = INF;
    for (int i = 0; i < (int)node[y].catch_dist.size(); i++)
        if (node[y].border_in_father[i] != -1)
            node[y].min_border_dist =
                std::min(node[y].min_border_dist, node[y].catch_dist[i]);
}

void G_Tree::push_borders_brother_catch(int x, int y, int bound) {
    int S = node[x].catch_id, LCA = node[x].father, i, j;
    if (node[y].catch_id == S && node[y].catch_bound >= bound)
        return;
    int p;
    node[y].catch_id = S;
    node[y].catch_bound = bound;
    std::vector<int> id_LCA[2], id_now[2];
    for (int t = 0; t < 2; t++) {
        if (t == 0)
            p = x;
        else
            p = y;
        for (i = j = 0; i < (int)node[p].borders.size(); i++)
            if (node[p].border_in_father[i] != -1)
                if ((t == 1 &&
                     (Optimization_Euclidean_Cut == false ||
                      Euclidean_Dist(node[x].catch_id,
                                     node[p].border_id[i]) < bound)) ||
                    (t == 0 && node[p].catch_dist[i] < bound)) {
                    id_LCA[t].push_back(node[p].border_in_father[i]);
                    id_now[t].push_back(i);
                }
    }
    for (int i = 0; i < (int)node[y].catch_dist.size(); i++)
        node[y].catch_dist[i] = INF;
    for (int i = 0; i < (int)id_LCA[0].size(); i++)
        for (int j = 0; j < (int)id_LCA[1].size(); j++) {
            int k = node[x].catch_dist[id_now[0][i]] +
                    node[LCA].dist.a[id_LCA[0][i]][id_LCA[1][j]];
            if (k < node[y].catch_dist[id_now[1][j]])
                node[y].catch_dist[id_now[1][j]] = k;
        }
    //int **dist = node[y].dist.a;
    auto dist = node[y].dist.a;
    // vector<int>begin,end;
    int *begin, *end;
    begin = new int[node[y].borders.size()];
    end = new int[node[y].borders.size()];
    int tot0 = 0, tot1 = 0;
    for (int i = 0; i < (int)node[y].catch_dist.size(); i++) {
        if (node[y].catch_dist[i] < bound)
            begin[tot0++] = i;
        else if (node[y].catch_dist[i] == INF) {
            if (Optimization_Euclidean_Cut == false ||
                Euclidean_Dist(node[x].catch_id, node[y].border_id[i]) <
                    bound)
                end[tot1++] = i;
        }
    }
    for (int i = 0; i < tot0; i++) {
        int i_ = begin[i];
        for (int j = 0; j < tot1; j++) {
            if (node[y].catch_dist[end[j]] >
                node[y].catch_dist[i_] + dist[i_][end[j]])
                node[y].catch_dist[end[j]] =
                    node[y].catch_dist[i_] + dist[i_][end[j]];
        }
    }
    delete[] begin;
    delete[] end;
    node[y].min_border_dist = INF;
    for (int i = 0; i < (int)node[y].catch_dist.size(); i++)
        if (node[y].border_in_father[i] != -1)
            node[y].min_border_dist =
                std::min(node[y].min_border_dist, node[y].catch_dist[i]);
}

void G_Tree::push_borders_up_path(int x, std::vector<int> &dist1) {
    if (node[x].father == 0)
        return;
    int y = node[x].father;
    std::vector<int> dist3(node[y].borders.size(), INF);
    std::vector<int> *order = &node[y].path_record;
    (*order).clear();
    for (int i = 0; i < (int)node[y].borders.size(); i++)
        (*order).push_back(-INF);
    for (int i = 0; i < (int)node[x].borders.size(); i++)
        if (node[x].border_in_father[i] != -1) {
            dist3[node[x].border_in_father[i]] = dist1[i];
            (*order)[node[x].border_in_father[i]] = -x;
        }
    // printf("dist3:");save_vector(dist3);
    //int **dist = node[y].dist.a;
    auto dist = node[y].dist.a;
    // vector<int>begin,end;
    int *begin, *end;
    begin = new int[node[x].borders.size()];
    end = new int[node[y].borders.size()];
    int tot0 = 0, tot1 = 0;
    for (int i = 0; i < (int)dist3.size(); i++) {
        if (dist3[i] < INF)
            begin[tot0++] = i;
        else if (node[y].border_in_father[i] != -1)
            end[tot1++] = i;
    }
    for (int i = 0; i < tot0; i++) {
        int i_ = begin[i];
        for (int j = 0; j < tot1; j++) {
            if (dist3[end[j]] > dist3[i_] + dist[i_][end[j]]) {
                dist3[end[j]] = dist3[i_] + dist[i_][end[j]];
                (*order)[end[j]] = i_;
            }
        }
    }
    dist1 = dist3;
    delete[] begin;
    delete[] end;
}

int G_Tree::find_LCA(int x, int y) {
    if (node[x].deep < node[y].deep)
        std::swap(x, y);
    while (node[x].deep > node[y].deep)
        x = node[x].father;
    while (x != y) {
        x = node[x].father;
        y = node[y].father;
    }
    return x;
}

int G_Tree::search(int S, int T) {
    std::lock_guard<std::mutex> gtlock(gtmx);  // Lock acquired
    if (S == T)
        return 0;

    int i, j, k, p;
    int LCA, x = id_in_node[S], y = id_in_node[T];
    if (node[x].deep < node[y].deep)
        std::swap(x, y);
    while (node[x].deep > node[y].deep)
        x = node[x].father;
    while (x != y) {
        x = node[x].father;
        y = node[y].father;
    }
    LCA = x;
    std::vector<int> dist[2], dist_;
    dist[0].push_back(0);
    dist[1].push_back(0);
    x = id_in_node[S], y = id_in_node[T];

    for (int t = 0; t < 2; t++) {
        if (t == 0)
            p = x;
        else
            p = y;
        while (node[p].father != LCA) {
            push_borders_up(p, dist[t], t);
            p = node[p].father;
        }
        if (t == 0)
            x = p;
        else
            y = p;
    }
    std::vector<int> id[2];
    for (int t = 0; t < 2; t++) {
        if (t == 0)
            p = x;
        else
            p = y;
        for (i = j = 0; i < (int)dist[t].size(); i++)
            if (node[p].border_in_father[i] != -1) {
                id[t].push_back(node[p].border_in_father[i]);
                dist[t][j] = dist[t][i];
                j++;
            }
        while (dist[t].size() > id[t].size()) {
            dist[t].pop_back();
        }
    }
    int MIN = INF;
    for (i = 0; i < (int)dist[0].size(); i++) {
        int i_ = id[0][i];
        for (j = 0; j < (int)dist[1].size(); j++) {
            k = dist[0][i] + dist[1][j] + node[LCA].dist.a[i_][id[1][j]];
            if (k < MIN)
                MIN = k;
        }
    }
    return MIN;
}

int G_Tree::search_catch(int S, int T, int bound) {

    if (S == T)
        return 0;

    int i, p;
    int x = id_in_node[S], y = id_in_node[T];
    int LCA = find_LCA(x, y);

    std::vector<int> node_path[2];
    for (int t = 0; t < 2; t++) {
        if (t == 0)
            p = x;
        else
            p = y;
        while (node[p].father != LCA) {
            node_path[t].push_back(p);
            p = node[p].father;
        }
        node_path[t].push_back(p);
        if (t == 0)
            x = p;
        else
            y = p;
    }

    node[id_in_node[S]].catch_id = S;
    node[id_in_node[S]].catch_bound = bound;
    node[id_in_node[S]].min_border_dist = 0;
    node[id_in_node[S]].catch_dist[0] = 0;
    for (i = 0; i + 1 < (int)node_path[0].size(); i++) {
        if (node[node_path[0][i]].min_border_dist >= bound)
            return INF;
        push_borders_up_catch(node_path[0][i]);
    }

    if (node[x].min_border_dist >= bound)
        return INF;
    push_borders_brother_catch(x, y);
    for (int i = (int)node_path[1].size() - 1; i > 0; i--) {
        if (node[node_path[1][i]].min_border_dist >= bound)
            return INF;
        push_borders_down_catch(node_path[1][i], node_path[1][i - 1]);
    }

    return node[id_in_node[T]].catch_dist[0];
}

int G_Tree::find_path(int S, int T, std::vector<int> &order) {
    std::lock_guard<std::mutex> gtlock(gtmx);  // Lock acquired
    order.clear();
    if (S == T) {
        order.push_back(S);
        //dists[S] = 0;
        return 0;
    }
    int i, j, k, p;
    int LCA, x = id_in_node[S], y = id_in_node[T];
    if (node[x].deep < node[y].deep)
        std::swap(x, y);
    while (node[x].deep > node[y].deep)
        x = node[x].father;
    while (x != y) {
        x = node[x].father;
        y = node[y].father;
    }
    LCA = x;
    std::vector<int> dist[2], dist_;
    dist[0].push_back(0);
    dist[1].push_back(0);
    x = id_in_node[S], y = id_in_node[T];
    // printf("LCA=%d x=%d y=%d\n",LCA,x,y);
    while (node[x].father != LCA) {
        push_borders_up_path(x, dist[0]);
        x = node[x].father;
    }
    while (node[y].father != LCA) {
        push_borders_up_path(y, dist[1]);
        y = node[y].father;
    }
    std::vector<int> id[2];
    for (int t = 0; t < 2; t++) {
        if (t == 0)
            p = x;
        else
            p = y;
        for (i = j = 0; i < (int)dist[t].size(); i++)
            if (node[p].border_in_father[i] != -1) {
                id[t].push_back(node[p].border_in_father[i]);
                dist[t][j] = dist[t][i];
                j++;
            }
        while (dist[t].size() > id[t].size()) {
            dist[t].pop_back();
        }
    }
    int MIN = INF;
    int S_ = -1, T_ = -1;
    for (i = 0; i < (int)dist[0].size(); i++)
        for (j = 0; j < (int)dist[1].size(); j++) {
            k = dist[0][i] + dist[1][j] +
                node[LCA].dist.a[id[0][i]][id[1][j]];
            if (k < MIN) {
                MIN = k;
                S_ = id[0][i];
                T_ = id[1][j];
            }
        }
    if (MIN < INF) {
        for (int t = 0; t < 2; t++) {
            int p, now;
            if (t == 0)
                p = x, now = node[LCA].border_in_son[S_];
            else
                p = y, now = node[LCA].border_in_son[T_];
            while (node[p].n > 1) {
                // printf("t=%d p=%d now=%d
                // node[p].path_record[now]=%d\n",t,p,now,node[p].path_record[now]);
                if (node[p].path_record[now] >= 0) {
                    find_path_border(p, now, node[p].path_record[now],
                                     order, 0);
                    now = node[p].path_record[now];
                } else if (node[p].path_record[now] > -INF) {
                    int temp = now;
                    now = node[p].border_in_son[now];
                    p = -node[p].path_record[temp];
                } else
                    break;
            }
            if (t == 0) {
                reverse(order.begin(), order.end());
                order.push_back(node[LCA].border_id[S_]);
                find_path_border(LCA, S_, T_, order, 0);
            }
        }
    }
    return MIN;
    // cout<<"QY5";
}

int G_Tree::real_border_number(int x) {
    int i, j, re = 0, id;
    std::map<int, int> vis;
    for (i = 0; i < node[x].G.n; i++)
        vis[node[x].G.id[i]] = 1;
    for (i = 0; i < node[x].G.n; i++) {
        id = node[x].G.id[i];
        for (j = G.head[id]; j; j = G.next[j])
            if (vis[G.list[j]] == 0) {
                re++;
                break;
            }
    }
    return re;
}

void G_Tree::find_path_border(int x, int S, int T, std::vector<int> &v, int rev) {
    /*printf("find:x=%d S=%d T=%d\n",x,S,T);
    printf("node:%d\n",x);
    node[x].write();
    printf("\n\n\n\n");*/
    if (node[x].order.a[S][T] == -1) {
        if (rev == 0)
            v.push_back(node[x].border_id[T]);
        else
            v.push_back(node[x].border_id[S]);
    } else if (node[x].order.a[S][T] == -2) {
        find_path_border(node[x].father, node[x].border_in_father[S],
                         node[x].border_in_father[T], v, rev);
    } else if (node[x].order.a[S][T] == -3) {
        find_path_border(
            node[x].son[node[x].color[node[x].border_id_innode[S]]],
            node[x].border_in_son[S], node[x].border_in_son[T], v, rev);
    } else if (node[x].order.a[S][T] >= 0) {
        int k = node[x].order.a[S][T];
        if (rev == 0) {
            find_path_border(x, S, k, v, rev);
            find_path_border(x, k, T, v, rev);
        } else {
            find_path_border(x, k, T, v, rev);
            find_path_border(x, S, k, v, rev);
        }
    }
}

std::vector<int> G_Tree::KNN(int S, int K, std::vector<int> T) {
    std::priority_queue<int> K_Value;
    std::vector<std::pair<int, int>> query;
    for (int i = 0; i < (int)T.size(); i++)
        query.push_back(std::make_pair(
            -node[find_LCA(id_in_node[S], id_in_node[T[i]])].deep, i));
    sort(query.begin(), query.end());
    std::vector<int> re, ans;
    if (K <= 0)
        return re;
    for (int i = 0; i < (int)T.size(); i++) {
        int bound = (int)K_Value.size() < K ? INF : K_Value.top();
        // if(bound==INF)printf("- ");else printf("%d ",bound);
        if (Optimization_KNN_Cut)
            ans.push_back(search_catch(S, T[query[i].second], bound));
        else
            ans.push_back(search_catch(S, T[query[i].second]));
        if ((int)K_Value.size() < K)
            K_Value.push(ans[i]);
        else if (ans[i] < K_Value.top()) {
            K_Value.pop();
            K_Value.push(ans[i]);
        }
    }
    int bound = ((int)ans.size() <= K) ? INF : K_Value.top();
    for (int i = 0; i < (int)T.size() && (int)re.size() < K; i++)
        if (ans[i] <= bound)
            re.push_back(query[i].second);
    sort(re.begin(), re.end());
    return re;
}

std::vector<int> G_Tree::KNN(int S, int K, std::vector<int> T,
                     std::vector<int> offset) {
    std::priority_queue<int> K_Value;
    std::vector<std::pair<int, int>> query;
    for (int i = 0; i < (int)T.size(); i++)
        query.push_back(std::make_pair(
            -node[find_LCA(id_in_node[S], id_in_node[T[i]])].deep, i));
    sort(query.begin(), query.end());
    std::vector<int> re, ans;
    if (K <= 0)
        return re;
    for (int i = 0; i < (int)T.size(); i++) {
        int bound = (int)K_Value.size() < K ? INF : K_Value.top();
        // if(bound==INF)printf("- ");else printf("%d ",bound);
        if (Optimization_KNN_Cut)
            ans.push_back(search_catch(S, T[query[i].second], bound) +
                          offset[query[i].second]);
        else
            ans.push_back(search_catch(S, T[query[i].second]) +
                          offset[query[i].second]);
        if ((int)K_Value.size() < K)
            K_Value.push(ans[i]);
        else if (ans[i] < K_Value.top()) {
            K_Value.pop();
            K_Value.push(ans[i]);
        }
    }
    int bound = ((int)ans.size() <= K) ? INF : K_Value.top();
    for (int i = 0; i < (int)T.size() && (int)re.size() < K; i++)
        if (ans[i] <= bound)
            re.push_back(query[i].second);
    sort(re.begin(), re.end());
    return re;
}

std::vector<int> G_Tree::KNN_bound(int S, int K, std::vector<int> T, int bound) {
    std::priority_queue<int> K_Value;
    std::vector<std::pair<int, int>> query;
    for (int i = 0; i < (int)T.size(); i++)
        query.push_back(std::make_pair(
            -node[find_LCA(id_in_node[S], id_in_node[T[i]])].deep, i));
    sort(query.begin(), query.end());
    std::vector<int> re, ans;
    if (K <= 0)
        return re;
    for (int i = 0; i < (int)T.size(); i++) {
        if (Optimization_KNN_Cut)
            ans.push_back(search_catch(S, T[query[i].second], bound));
        else
            ans.push_back(search_catch(S, T[query[i].second]));
        if ((int)K_Value.size() < K)
            K_Value.push(ans[i]);
        else if (ans[i] < K_Value.top()) {
            K_Value.pop();
            K_Value.push(ans[i]);
        }
    }
    for (int i = 0; i < (int)T.size() && (int)re.size() < K; i++)
        if (ans[i] <= bound)
            re.push_back(query[i].second);
    sort(re.begin(), re.end());
    return re;
}

std::vector<int> G_Tree::KNN_bound(int S, int K, std::vector<int> T, int bound,
                           std::vector<int> offset) {
    std::priority_queue<int> K_Value;
    std::vector<std::pair<int, int>> query;
    for (int i = 0; i < (int)T.size(); i++)
        query.push_back(std::make_pair(
            -node[find_LCA(id_in_node[S], id_in_node[T[i]])].deep, i));
    sort(query.begin(), query.end());
    std::vector<int> re, ans;
    if (K <= 0)
        return re;
    for (int i = 0; i < (int)T.size(); i++) {
        if (Optimization_KNN_Cut)
            ans.push_back(search_catch(S, T[query[i].second], bound) +
                          offset[query[i].second]);
        else
            ans.push_back(search_catch(S, T[query[i].second]) +
                          offset[query[i].second]);
        if ((int)K_Value.size() < K)
            K_Value.push(ans[i]);
        else if (ans[i] < K_Value.top()) {
            K_Value.pop();
            K_Value.push(ans[i]);
        }
    }
    for (int i = 0; i < (int)T.size() && (int)re.size() < K; i++)
        if (ans[i] <= bound)
            re.push_back(query[i].second);
    sort(re.begin(), re.end());
    return re;
}

std::vector<int> G_Tree::Range(int S, int R, std::vector<int> T) {
    std::vector<int> re;
    for (int i = 0; i < (int)T.size(); i++) {
        if (search_catch(S, T[i], Optimization_KNN_Cut ? R : INF) < R)
            re.push_back(i);
    }
    return re;
}

std::vector<int> G_Tree::Range(int S, int R, std::vector<int> T,
                       std::vector<int> offset) {
    std::vector<int> re;
    for (int i = 0; i < (int)T.size(); i++) {
        if (offset[i] +
                search_catch(S, T[i], Optimization_KNN_Cut ? R : INF) <
            R)
            re.push_back(i);
    }
    return re;
}

void G_Tree::add_car(int node_id, int car_id) {
    car_in_node[node_id].push_back(car_id);
    if ((int)car_in_node[node_id].size() == 1) {
        int S = id_in_node[node_id];
        node[S].min_car_dist[0] = std::make_pair(0, node_id);
        for (int p = S; push_borders_up_add_min_car_dist(p, node_id);
             p = node[p].father)
            ;
    }
}

void G_Tree::del_car(int node_id, int car_id) {
    int i;
    for (i = 0; i < (int)car_in_node[node_id].size(); i++)
        if (car_in_node[node_id][i] == car_id)
            break;
    if (i == (int)car_in_node[node_id].size())
        printf("Error: del_car find none car!");
    while (i + 1 < (int)car_in_node[node_id].size()) {
        std::swap(car_in_node[node_id][i], car_in_node[node_id][i + 1]);
        i++;
    }
    car_in_node[node_id].pop_back();
    if ((int)car_in_node[node_id].size() == 0) {
        int S = id_in_node[node_id];
        node[S].min_car_dist[0] = std::make_pair(INF, -1);
        for (int p = S; push_borders_up_del_min_car_dist(p, node_id);
             p = node[p].father)
            ;
    }
}

void G_Tree::change_car_offset(int car_id, int dist) {
    while ((int)car_offset.size() <= car_id)
        car_offset.push_back(0);
    car_offset[car_id] = dist;
}

int G_Tree::get_car_offset(int car_id) {
    while ((int)car_offset.size() <= car_id)
        car_offset.push_back(0);
    return car_offset[car_id];
}

bool G_Tree::push_borders_up_add_min_car_dist(int x, int start_id) {
    int re = false;
    if (node[x].father == 0)
        return re;
    int y = node[x].father;
    std::vector<std::pair<int, int>> *dist1 = &node[x].min_car_dist,
                                     *dist2 = &node[y].min_car_dist;
    for (int i = 0; i < (int)node[x].borders.size(); i++)
        if (node[x].min_car_dist[i].second == start_id) {
            re = true;
            if (node[x].border_in_father[i] != -1) {
                if ((*dist2)[node[x].border_in_father[i]].first >
                    (*dist1)[i].first)
                    (*dist2)[node[x].border_in_father[i]] = (*dist1)[i];
            }
        }
    if (y != root) {
        //int **dist = node[y].dist.a;
        auto dist = node[y].dist.a;
        int tot0 = 0, tot1 = 0;
        for (int i = 0; i < (int)(*dist2).size(); i++) {
            if ((*dist2)[i].second == start_id)
                begin[tot0++] = i;
            else
                end[tot1++] = i;
        }
        for (int i = 0; i < tot0; i++) {
            int i_ = begin[i];
            for (int j = 0; j < tot1; j++) {
                if ((*dist2)[end[j]].first >
                    (*dist2)[i_].first + dist[end[j]][i_]) {
                    (*dist2)[end[j]].first =
                        (*dist2)[i_].first + dist[end[j]][i_];
                    (*dist2)[end[j]].second = (*dist2)[i_].second;
                }
            }
        }
    }
    return re;
}

bool G_Tree::push_borders_up_del_min_car_dist(int x, int start_id) {
    int re = false;
    if (node[x].father == 0)
        return false;
    int y = node[x].father;
    // printf("min_car_dist ");for(int
    // i=0;i<node[y].min_car_dist.size();i++)printf("(i:%d,D:%d,id:%d)",i,node[y].min_car_dist[i].first,node[y].min_car_dist[i].second);printf("\n");
    std::vector<std::pair<int, int>> *dist1 = &node[x].min_car_dist,
                                     *dist2 = &node[y].min_car_dist;
    int tot0 = 0, tot1 = 0;
    if (y == root) {
        for (int i = 0; i < (int)node[x].borders.size(); i++)
            if (node[x].border_in_father[i] != -1) {
                if ((*dist2)[node[x].border_in_father[i]].second ==
                    start_id) {
                    (*dist2)[node[x].border_in_father[i]] =
                        std::make_pair(INF, -1);
                    re = true;
                }
            }
        for (int i = 0; i < (int)node[x].borders.size(); i++) {
            if (node[x].border_in_father[i] != -1) {
                if ((*dist2)[node[x].border_in_father[i]].first >
                    (*dist1)[i].first)
                    (*dist2)[node[x].border_in_father[i]] = (*dist1)[i];
            }
        }
    } else {
        int SIZE = (int)node[y].borders.size();
        for (int i = 0; i < SIZE; i++) {
            if ((*dist2)[i].second == start_id) {
                (*dist2)[i] = node[node[y].border_son_id[i]]
                                  .min_car_dist[node[y].border_in_son[i]];
                end[tot1++] = i;
                re = true;
            } else
                begin[tot0++] = i;
        }
    }
    if (re) {
        if (y != root) {
            //int **dist = node[y].dist.a;
            auto dist = node[y].dist.a;
            for (int i = 0; i < tot0; i++) {
                int i_ = begin[i];
                for (int j = 0; j < tot1; j++) {
                    if ((*dist2)[end[j]].first >
                        (*dist2)[i_].first + dist[end[j]][i_]) {
                        (*dist2)[end[j]].first =
                            (*dist2)[i_].first + dist[end[j]][i_];
                        (*dist2)[end[j]].second = (*dist2)[i_].second;
                    }
                }
            }
            while (tot1) {
                for (int i = 0; i < tot1 - 1; i++)
                    if ((*dist2)[end[i]].first < (*dist2)[end[i + 1]].first)
                        std::swap(end[i], end[i + 1]);
                tot1--;
                int i_ = end[tot1];
                for (int j = 0; j < tot1; j++) {
                    if ((*dist2)[end[j]].first >
                        (*dist2)[i_].first + dist[i_][end[j]]) {
                        (*dist2)[end[j]].first =
                            (*dist2)[i_].first + dist[i_][end[j]];
                        (*dist2)[end[j]].second = (*dist2)[i_].second;
                    }
                }
            }
        }
    }
    // if(DEBUG_)printf("re=%d\n",re);
    return re;
}

int G_Tree::push_borders_up_catch_KNN_min_dist_car(int x) {
    if (node[x].father == 0)
        return INF;
    int re = INF + 1;
    int y = node[x].father;
    node[y].catch_id = node[x].catch_id;
    node[y].catch_bound = -1;
    std::vector<int> *dist1 = &node[x].catch_dist,
                     *dist2 = &node[y].catch_dist;
    for (int i = 0; i < (int)(*dist2).size(); i++)
        (*dist2)[i] = INF;
    for (int i = 0; i < (int)node[x].borders.size(); i++)
        if (node[x].border_in_father[i] != -1)
            (*dist2)[node[x].border_in_father[i]] = (*dist1)[i];
    //int **dist = node[y].dist.a;
    auto dist = node[y].dist.a;
    int *begin, *end;
    begin = new int[node[x].borders.size()];
    end = new int[node[y].borders.size()];
    int tot0 = 0, tot1 = 0;
    for (int i = 0; i < (int)(*dist2).size(); i++) {
        if ((*dist2)[i] < INF)
            begin[tot0++] = i;
        else
            end[tot1++] = i;
    }
    for (int i = 0; i < tot0; i++) {
        int i_ = begin[i];
        for (int j = 0; j < tot1; j++) {
            if ((*dist2)[end[j]] > (*dist2)[i_] + dist[i_][end[j]])
                (*dist2)[end[j]] = (*dist2)[i_] + dist[i_][end[j]];
        }
    }
    if (y == root)
        re = INF + 1;
    else
        for (int i = 0; i < (int)node[y].borders.size(); i++)
            if (node[y].border_in_father[i] != -1)
                re = std::min(re, (*dist2)[i]);
    delete[] begin;
    delete[] end;
    return re;
}

std::vector<int> G_Tree::KNN_min_dist_car(int S, int K) {
    int Now_Catch_P = id_in_node[S], Now_Catch_Dist = 0;
    std::priority_queue<std::pair<int, std::pair<int, int>>> q;
    {
        node[id_in_node[S]].catch_id = S;
        node[id_in_node[S]].catch_bound = INF;
        node[id_in_node[S]].min_border_dist = 0;
        node[id_in_node[S]].catch_dist[0] = 0;
        /*for(int p=id_in_node[S];p!=root;p=node[p].father)
        push_borders_up_catch_KNN_min_dist_car(p);*/
    }
    for (int i = 0; i < (int)node[Now_Catch_P].borders.size(); i++)
        q.push(std::make_pair(-(node[Now_Catch_P].catch_dist[i] +
                                node[Now_Catch_P].min_car_dist[i].first),
                              std::make_pair(Now_Catch_P, i)));
    std::vector<int> ans, ans2;
    if (Distance_Offset == false) {
        while (K) {
            int Dist = q.top().first;
            int node_id = q.top().second.first;
            int border_id = q.top().second.second;
            int real_Dist = -(node[node_id].catch_dist[border_id] +
                              node[node_id].min_car_dist[border_id].first);
            if (Dist != real_Dist) {
                q.pop();
                q.push(std::make_pair(real_Dist,
                                      std::make_pair(node_id, border_id)));
                continue;
            }
            if (-Dist > Now_Catch_Dist && Now_Catch_P != root) {
                Now_Catch_Dist =
                    push_borders_up_catch_KNN_min_dist_car(Now_Catch_P);
                Now_Catch_P = node[Now_Catch_P].father;
                for (int i = 0; i < (int)node[Now_Catch_P].borders.size(); i++) {
                    q.push(std::make_pair(
                        -(node[Now_Catch_P].catch_dist[i] +
                          node[Now_Catch_P].min_car_dist[i].first),
                        std::make_pair(Now_Catch_P, i)));
                }
                continue;
            }
            int real_node_id = node[node_id].min_car_dist[border_id].second;

            if (real_node_id == -1)
                break;
            int car_id = car_in_node[real_node_id][0];
            q.pop();
            ans.push_back(car_id);
            ans2.push_back(real_node_id);
            del_car(real_node_id, car_id);
            q.push(std::make_pair(
                -(node[node_id].catch_dist[border_id] +
                  node[node_id].min_car_dist[border_id].first),
                std::make_pair(node_id, border_id)));
            K--;
        }
        for (int i = 0; i < (int)ans.size(); i++)
            add_car(ans2[i], ans[i]);
    } else {
        std::priority_queue<int> KNN_Dist;
        std::vector<int> ans3;
        while ((int)KNN_Dist.size() < K || KNN_Dist.top() <= -q.top().first) {
            int Dist = q.top().first;
            int node_id = q.top().second.first;
            int border_id = q.top().second.second;
            int real_Dist = -(node[node_id].catch_dist[border_id] +
                              node[node_id].min_car_dist[border_id].first);
            if (Dist != real_Dist) {
                q.pop();
                q.push(std::make_pair(real_Dist,
                                      std::make_pair(node_id, border_id)));
                continue;
            }
            if (-Dist > Now_Catch_Dist && Now_Catch_P != root) {
                Now_Catch_Dist =
                    push_borders_up_catch_KNN_min_dist_car(Now_Catch_P);
                Now_Catch_P = node[Now_Catch_P].father;
                for (int i = 0; i < (int)node[Now_Catch_P].borders.size(); i++)
                    q.push(std::make_pair(
                        -(node[Now_Catch_P].catch_dist[i] +
                          node[Now_Catch_P].min_car_dist[i].first),
                        std::make_pair(Now_Catch_P, i)));
                continue;
            }
            int real_node_id = node[node_id].min_car_dist[border_id].second;

            if (real_node_id == -1)
                break;
            int car_id = car_in_node[real_node_id][0];
            q.pop();
            del_car(real_node_id, car_id);
            q.push(std::make_pair(
                -(node[node_id].catch_dist[border_id] +
                  node[node_id].min_car_dist[border_id].first),
                std::make_pair(node_id, border_id)));
            int car_dist = get_car_offset(car_id) - real_Dist;
            if ((int)KNN_Dist.size() < K)
                KNN_Dist.push(car_dist);
            else if (KNN_Dist.top() > car_dist) {
                KNN_Dist.pop();
                KNN_Dist.push(car_dist);
            }
            ans.push_back(car_id);
            ans2.push_back(real_node_id);
            ans3.push_back(car_dist);
        }
        for (int i = 0; i < (int)ans.size(); i++)
            add_car(ans2[i], ans[i]);
        int j = 0;
        for (int i = 0; i < (int)ans.size(); i++)
            if (ans3[i] <= KNN_Dist.top())
                ans[j++] = ans[i];
        while ((int)ans.size() > K)
            ans.pop_back();
    }
    return ans;
}

bool G_Tree::check_min_car_dist(int x_) {
    for (int x = (x_ == -1 ? node_tot : x_ + 1) - 1;
         x >= (x_ == -1 ? root : x_); x--) {
        if (x == root)
            continue;
        if (node[x].borders.size() == 1)
            continue;
        int i, j, ans;
        for (i = 0; i < (int)node[x].borders.size(); i++) {
            ans = INF;
            int ans_id = -1, order = -1;
            if (ans >
                node[node[x]
                         .son[node[x].color[node[x].border_id_innode[i]]]]
                    .min_car_dist[node[x].border_in_son[i]]
                    .first) {
                ans = node[node[x].son
                               [node[x].color[node[x].border_id_innode[i]]]]
                          .min_car_dist[node[x].border_in_son[i]]
                          .first;
                ans_id =
                    node[node[x].son
                             [node[x].color[node[x].border_id_innode[i]]]]
                        .min_car_dist[node[x].border_in_son[i]]
                        .second;
                order = -2;
            }
            for (j = 0; j < (int)node[x].borders.size(); j++)
                if (j != i) {
                    if (ans > node[x].min_car_dist[j].first +
                                  node[x].dist.a[i][j]) {
                        ans = node[x].min_car_dist[j].first +
                              node[x].dist.a[i][j];
                        ans_id = node[x].min_car_dist[j].second;
                        order = j;
                    }
                }

            if (ans !=
                node[x]
                    .min_car_dist[i]
                    .first /*||ans_id!=node[x].min_car_dist[i].second*/) {
                printf("x=%d i=%d ans=%d ans_id=%d min=%d min_id=%d "
                       "order=%d\n",
                       x, i, ans, ans_id, node[x].min_car_dist[i].first,
                       node[x].min_car_dist[i].second, order);
                printf(
                    "node[x].son[node[x].color[node[x].border_id_innode[i]]"
                    "]=%d "
                    "node[x].border_in_son[i]=%d\n",
                    node[x].son[node[x].color[node[x].border_id_innode[i]]],
                    node[x].border_in_son[i]);
                return false;
            }
        }
    }
    return true;
}

void Wide_KNN_::init(int s, int k) {
    S = s;
    K = k;
    tot = 0;
    Real_Dist = INF;
    Euclid = 0;
    while (KNN.size())
        KNN.pop();
    re.clear();
}

bool Wide_KNN_::update(std::vector<std::pair<double, std::pair<int, int>>> a) {
    std::sort(a.begin(), a.end());
    for (int i = 0; i < (int)a.size(); i++) {
        bound = (int)KNN.size() < K ? INF : KNN.top().first;
        dist_now = tree.search_catch(S, a[i].second.first, bound) +
                   a[i].second.second;
        if ((int)KNN.size() < K)
            KNN.push(std::make_pair(dist_now, tot));
        else if (dist_now < KNN.top().first) {
            KNN.pop();
            KNN.push(std::make_pair(dist_now, tot));
        }
        tot++;
        Real_Dist = bound;
        Euclid = a[i].first;
        if (Real_Dist < Euclid) {
            while (KNN.size()) {
                re.push_back(KNN.top().second);
                KNN.pop();
            }
            return true;
        }
    }
    return false;
}

std::vector<int> Wide_KNN_::result() { return re; }

void init() {
    srand(747929791);
    Additional_Memory = 0;
}

void read(const std::string &fn) {
    const char *c_fn;
    c_fn = fn.c_str();
    printf("begin read\n");
    FILE *in = NULL;
    in = fopen(c_fn, "r");
    if (!in) {
        std::printf("File %s not found!\n", c_fn);
        exit(EXIT_FAILURE);
    }
    fscanf(in, "%d %d\n", &G.n, &G.m);
    G.init(G.n, G.m);
    for (int i = 0; i < G.n; i++)
        G.id[i] = i;
    int i, j, k, l;
    for (i = 0; i < G.m; i++) {
        fscanf(in, "%d %d %d\n", &j, &k, &l);
        if (RevE) G.add(j, k, l);       // input is undirected
        else      G.add_D(j, k, l);     // input is directed
    }
    fclose(in);
//    if (Optimization_Euclidean_Cut) {
//        in = fopen(Node_File, "r");
//        double d1, d2;
//        for (i = 0; i < G.n; i++) {
//            fscanf(in, "%d %lf %lf\n", &j, &d1, &d2);
//            coordinate.push_back(coor(d1, d2));
//        }
//        fclose(in);
//        printf("read over\n");
//    }
}

void save(G_Tree& gtree) {
    printf("begin save\n");
    freopen("GP_Tree.gtree", "w", stdout);
    gtree.save();
    freopen("/dev/tty", "w", stdout);
    printf("save_over\n");
}

void load() {
    load("GP_Tree.gtree");
}

void load(const std::string &fn) {
    const char *c_fn;
    c_fn = fn.c_str();
    freopen(c_fn, "r", stdin);
    tree.load();
    freopen("/dev/tty", "r", stdin);
}

void setAdMem(long long x) {
    Additional_Memory = x;
}

} // End namespace GTree

