#include "catch.hpp"
#include "libcargo.h"

#include <stdio.h>

using namespace cargo;

static int callback(void *NotUsed, int argc, char **argv, char **azColName) {
    int i;
    for (i = 0; i < argc; i++) {
        printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
    }
    printf("\n");
    return 0;
}

TEST_CASE("Sqlite3 works", "[sqlite3]") {
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;
    rc = sqlite3_open(":memory:", &db);
    if (rc) {
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        sqlite3_close(db);
    } else {
        rc = sqlite3_exec(db, "CREATE TABLE t(x INTEGER);", callback, 0,
                          &zErrMsg);
        rc = sqlite3_exec(db, "INSERT INTO t VALUES(42);", callback, 0,
                          &zErrMsg);
        rc = sqlite3_exec(db, "SELECT * FROM t;", callback, 0, &zErrMsg);
        if (rc != SQLITE_OK) {
            fprintf(stderr, "SQL error: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        }
    }
    sqlite3_close(db);
}

static int circle_query_callback(sqlite3_rtree_query_info *p) {
    int nParam = p->nParam;
    // std::cout << p->nParam << " " << p->nCoord << std::endl;
    double c_lng = p->aParam[0];
    double c_lat = p->aParam[1];
    // std::cout << c_lng << " " << c_lat << std::endl;
    float lng = p->aCoord[0];
    float lat = p->aCoord[2];
    // std::cout << lng << " " << lat << std::endl;
    Point a = {(float)p->aParam[0], (float)p->aParam[1]};
    Point b = {(float)p->aCoord[0], (float)p->aCoord[2]};
    // if ((lng - c_lng) * (lng - c_lng) + (lat - c_lat) * (lat - c_lat) <
    //     0.02 * 0.02)
    //     p->eWithin = FULLY_WITHIN;
    // else
    //     p->eWithin = NOT_WITHIN;
    Distance d = distance::haversine(a, b);
    // std::cout << "d is: " << d << std::endl;
    // std::cout << "p is: " << p->eParentWithin << std::endl;
    // if (distance::haversine(a, b) < 300)
    // std::cout << p->iLevel << " " << p->mxLevel << std::endl;
    if (d < 1000)
        p->eWithin = FULLY_WITHIN;
    else {
        if (p->iLevel == p->mxLevel) {
            p->eWithin = NOT_WITHIN;
            std::cout << "hi" << std::endl;
        } else {
            p->eWithin = PARTLY_WITHIN;
            p->rScore = p->iLevel;
            std::cout << "hello" << std::endl;
        }
    }
    return SQLITE_OK;
}

static int circle_geometry_callback(sqlite3_rtree_geometry *g, int nCoord,
                                    sqlite3_rtree_dbl *aCoord, int *pRes) {
    Point a = {(float)g->aParam[0], (float)g->aParam[1]};
    Point b = {(float)aCoord[0], (float)aCoord[2]};
    // if ((lng - c_lng) * (lng - c_lng) + (lat - c_lat) * (lat - c_lat) <
    //     0.02 * 0.02)
    //     p->eWithin = FULLY_WITHIN;
    // else
    //     p->eWithin = NOT_WITHIN;
    Distance d = distance::haversine(a, b);
    std::cout << "d is: " << d << std::endl;
    // std::cout << "p is: " << p->eParentWithin << std::endl;
    // if (distance::haversine(a, b) < 300)
    // std::cout << p->iLevel << " " << p->mxLevel << std::endl;
    if (d < 1000)
        *pRes = 1;
    else
        *pRes = 0;
    return SQLITE_OK;
}

static int exec_query_callback(void *unused, int count, char **data,
                               char **columns) {
    // for (int i = 0; i < count; ++i)
    //     std::cout << data[i] << " ";
    // std::cout << std::endl;
    int &num = *(int *)unused;
    num++;
    return 0;
}

TEST_CASE("Sqlite3 Rtree", "[sqlite3]") {
    std::cout << "rtree test" << std::endl;
    Longitude minX, maxX;
    Latitude minY, maxY;
    KeyValueNodes nodes;
    file::ReadNodes("../data/roadnetwork/mny.rnet", nodes, minX, maxX, minY,
                    maxY);
    std::cout << nodes.size() << std::endl;
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;
    rc = sqlite3_open(":memory:", &db);
    if (rc != SQLITE_OK) {
        std::cerr << "open error: " << sqlite3_errmsg(db) << std::endl;
        sqlite3_close(db);
    } else {
        rc = sqlite3_exec(db,
                          "CREATE VIRTUAL TABLE location_index USING rtree "
                          "(id, minX, maxX, minY, maxY)",
                          NULL, NULL, &zErrMsg);
        if (rc != SQLITE_OK)
            std::cout << "exec error: " << zErrMsg << std::endl;
        // rc = sqlite3_rtree_query_callback(db, "circle",
        // circle_query_callback,
        //                                   NULL, NULL);
        // if (rc != SQLITE_OK)
        //     std::cout << "rtree error: " << sqlite3_errmsg(db) << std::endl;
        rc = sqlite3_rtree_geometry_callback(db, "circle",
                                             circle_geometry_callback, NULL);
        if (rc != SQLITE_OK)
            std::cout << "rtree error: " << sqlite3_errmsg(db) << std::endl;

        sqlite3_stmt *insert_node;
        rc = sqlite3_prepare_v2(db,
                                "INSERT INTO location_index(id, minX, maxX, "
                                "minY, maxY) VALUES(?, ?, ?, ?, ?)",
                                -1, &insert_node, NULL);
        for (auto &kv : nodes) {
            sqlite3_bind_int(insert_node, 1, kv.first);
            sqlite3_bind_double(insert_node, 2, kv.second.lng);
            sqlite3_bind_double(insert_node, 3, kv.second.lng);
            sqlite3_bind_double(insert_node, 4, kv.second.lat);
            sqlite3_bind_double(insert_node, 5, kv.second.lat);
            // sqlite3_exec(db, "SELECT * FROM location_index",
            //  exec_query_callback, NULL, &zErrMsg);
            rc = sqlite3_step(insert_node);
            if (rc != SQLITE_DONE)
                std::cout << "step error: " << sqlite3_errmsg(db) << std::endl;
            rc = sqlite3_reset(insert_node);
        }

        auto iter = nodes.begin();
        // iter++;
        // iter++;
        std::string select_query =
            "SELECT * from location_index WHERE id MATCH circle(";
        select_query.append(std::to_string(iter->second.lng));
        select_query.append(", ");
        select_query.append(std::to_string(iter->second.lat));
        select_query.append(")");
        int num = 0;
        rc = sqlite3_exec(db, select_query.c_str(), exec_query_callback, &num,
                          &zErrMsg);
        std::cout << num << std::endl;
        if (rc != SQLITE_OK)
            std::cout << "exec error: " << sqlite3_errmsg(db) << std::endl;

        sqlite3_finalize(insert_node);
        sqlite3_close(db);
    }
}