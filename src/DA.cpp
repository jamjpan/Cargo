#include "libcargo/DA.h"
#include <iostream>

namespace cargo {
DA::DA() {
    rc = sqlite3_open(":memory:", &db);
    if (rc != SQLITE_OK) {
        std::cerr << "Cannot open database: " << sqlite3_errmsg(db)
                  << std::endl;
        sqlite3_close(db);
    } else {
        // create vehicle table
        rc = sqlite3_exec(db,
                          "CREATE TABLE vehicle(id INT, o_id INT, d_id INT, "
                          "early INT, late INT, "
                          "demand INT, load INT, nnd INT, route TEXT, sched "
                          "TEXT, lv_node INT, "
                          "lv_stop INT, is_active INT)",
                          NULL, NULL, &zErrMsg);
        if (rc != SQLITE_OK) {
            std::cerr << "exec error: " << zErrMsg << std::endl;
            sqlite3_free(zErrMsg);
        }
        // create request table
        rc = sqlite3_exec(
            db,
            "CREATE TABLE request(id INT, o_id INT, d_id INT, early "
            "INT, late INT, demand INT, online_time INT, matched INT)",
            NULL, NULL, &zErrMsg);
        if (rc != SQLITE_OK) {
            std::cerr << "exec error: " << zErrMsg << std::endl;
            sqlite3_free(zErrMsg);
        }
        // craete stop table
        rc = sqlite3_exec(db,
                          "CREATE TABLE stop(t_id INT, n_id INT, type INT "
                          ", visit_time INT, time_limit INT)",
                          NULL, NULL, &zErrMsg);
        if (rc != SQLITE_OK) {
            std::cerr << "exec error: " << zErrMsg << std::endl;
            sqlite3_free(zErrMsg);
        }
    }
}

DA::~DA() {
    std::cout << "DA destructor" << std::endl;
    // rc = sqlite3_exec(db, "DROP TABLE vehicle", NULL, NULL, &zErrMsg);
    // if (rc != SQLITE_OK) {
    //   std::cerr << "SQL error: " << zErrMsg << std::endl;
    //   sqlite3_free(zErrMsg);
    // }
    // rc = sqlite3_exec(db, "DROP TABLE request", NULL, NULL, &zErrMsg);
    // if (rc != SQLITE_OK) {
    //   std::cerr << "SQL error: " << zErrMsg << std::endl;
    //   sqlite3_free(zErrMsg);
    // }
    rc = sqlite3_close(db);
    if (rc != SQLITE_OK) {
        std::cerr << "Cannot close database: " << sqlite3_errmsg(db)
                  << std::endl;
    }
}

int DA::AddVehicle(Vehicle *vehicle) {
    sqlite3_stmt *stmt;
    rc = sqlite3_prepare_v2(
        db,
        "INSERT INTO vehicle(id, o_id, d_id, early, late, demand, "
        "load, nnd, route, sched, lv_node, lv_stop, is_active) "
        "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        std::cerr << "prepare error: " << sqlite3_errmsg(db) << std::endl;
        return -1;
    } else {
        sqlite3_bind_int(stmt, 1, vehicle->id);
        sqlite3_bind_int(stmt, 2, vehicle->oid);
        sqlite3_bind_int(stmt, 3, vehicle->did);
        sqlite3_bind_int(stmt, 4, vehicle->early);
        sqlite3_bind_int(stmt, 5, vehicle->late);
        sqlite3_bind_int(stmt, 6, vehicle->demand);
        sqlite3_bind_int(stmt, 7, vehicle->load);
        sqlite3_bind_int(stmt, 8, vehicle->nnd);

        // convert route into string
        std::string route = VectorToString(vehicle->route);
        sqlite3_bind_text(stmt, 9, route.c_str(), route.length(),
                          SQLITE_STATIC);

        // insert stops into stop table
        std::vector<long long int> stop_ids;
        sqlite3_stmt *insert_stop;
        rc = sqlite3_prepare_v2(db,
                                "INSERT INTO stop(t_id, n_id, type, "
                                "visit_time, time_limit) VALUES(?, ?, ?, ?, ?)",
                                -1, &insert_stop, NULL);
        for (auto &stop : vehicle->sched) {
            // std::cout << stop.trip_id << " in " << stop.node_id << std::endl;
            sqlite3_bind_int(insert_stop, 1, stop.trip_id);
            sqlite3_bind_int(insert_stop, 2, stop.node_id);
            sqlite3_bind_int(insert_stop, 3, (int)stop.type);
            sqlite3_bind_int(insert_stop, 4, stop.visit_time);
            sqlite3_bind_int(insert_stop, 5, stop.time_limit);
            rc = sqlite3_step(insert_stop);
            if (rc != SQLITE_DONE) {
                std::cerr << "step error: " << sqlite3_errmsg(db) << std::endl;
                return -1;
            }
            stop_ids.push_back(sqlite3_last_insert_rowid(db));
        }
        rc = sqlite3_finalize(insert_stop);
        if (rc != SQLITE_OK) {
            std::cerr << "finalize error: " << sqlite3_errmsg(db) << std::endl;
            return -1;
        }
        // not sure whether there will be concurrency problem
        std::string sched = VectorToString(stop_ids);
        // std::cout << "vehicle " << vehicle->id << " " << sched << std::endl;
        sqlite3_bind_text(stmt, 10, sched.c_str(), sched.length(),
                          SQLITE_STATIC);
        sqlite3_bind_int(stmt, 11, vehicle->lv_node);
        sqlite3_bind_int(stmt, 12, vehicle->lv_stop);
        sqlite3_bind_int(stmt, 13, vehicle->is_active);
    }
    rc = sqlite3_step(stmt);
    if (rc != SQLITE_DONE) {
        std::cerr << "step error: " << sqlite3_errmsg(db) << std::endl;
        return -1;
    }
    rc = sqlite3_finalize(stmt);
    if (rc != SQLITE_OK) {
        std::cerr << "finalize error: " << sqlite3_errmsg(db) << std::endl;
        return -1;
    }
    return 0;
}

int DA::UpdateLocation(VehicleId vid, int lv_node, int nnd) {
    sqlite3_stmt *stmt;
    rc = sqlite3_prepare_v2(
        db, "UPDATE vehicle SET lv_node = ?, nnd = ? WHERE id = ?", -1, &stmt,
        NULL);
    if (rc != SQLITE_OK) {
        std::cerr << "prepare error: " << sqlite3_errmsg(db) << std::endl;
        return -1;
    } else {
        sqlite3_bind_int(stmt, 1, lv_node);
        sqlite3_bind_int(stmt, 2, nnd);
        sqlite3_bind_int(stmt, 3, vid);
    }
    rc = sqlite3_step(stmt);
    if (rc != SQLITE_OK) {
        std::cerr << "step error: " << sqlite3_errmsg(db) << std::endl;
        return -1;
    }
    rc = sqlite3_finalize(stmt);
    if (rc != SQLITE_OK) {
        std::cerr << "finalize error: " << sqlite3_errmsg(db) << std::endl;
        return -1;
    }
    return 0;
}

int DA::InsertRequest(Trip *request) {
    sqlite3_stmt *stmt;
    rc = sqlite3_prepare_v2(
        db,
        "INSERT INTO request(id, o_id, d_id, early, late, "
        "demand, online_time, matched) VALUES(?, ?, ?, ?, ?, ?)",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        std::cerr << "prepare error: " << sqlite3_errmsg(db) << std::endl;
        return -1;
    }
    sqlite3_bind_int(stmt, 1, request->id);
    sqlite3_bind_int(stmt, 2, request->oid);
    sqlite3_bind_int(stmt, 3, request->did);
    sqlite3_bind_int(stmt, 4, request->early);
    sqlite3_bind_int(stmt, 5, request->late);
    sqlite3_bind_int(stmt, 6, request->demand);
    sqlite3_bind_int(stmt, 7, -1);
    sqlite3_bind_int(stmt, 8, 0);
    rc = sqlite3_step(stmt);
    if (rc != SQLITE_OK) {
        std::cerr << "step error: " << sqlite3_errmsg(db) << std::endl;
        return -1;
    }
    return 0;
}

static int OneRequestCallback(void *request, int count, char **data,
                              char **columns) {
    request = new Vehicle();
    for (int i = 0; i < count; ++i)
        std::cout << columns[i] << " is " << data[i] << std::endl;
    return 0;
}

int DA::GetOneRequest(Trip *request) {
    request = NULL;
    rc = sqlite3_exec(
        db,
        "SELECT * FROM request WHERE matched = 0 ORDER BY online_time "
        "ASC LIMIT 1",
        OneRequestCallback, (void *)request, &zErrMsg);
    if (rc != SQLITE_OK) {
        std::cerr << "exec error: " << zErrMsg << std::endl;
        sqlite3_free(zErrMsg);
        return -1;
    }
    if (request == NULL)
        return 0;
    else
        return 1;
}

static int NRequestCallback(void *requests, int count, char **data,
                            char **columns) {
    Trip *request = new Trip();
    ((std::vector<Trip *> *)requests)->push_back(request);
    for (int i = 0; i < count; ++i)
        std::cout << columns[i] << " is " << data[i] << std::endl;
    return 0;
}

int DA::GetNRequest(std::vector<Trip *> &requests, int n) {
    std::string sql =
        "SELECT * FROM request WHERE matched = 0 ORDER BY online_time ASC ";
    if (n >= 0)
        sql += "LIMIT " + std::to_string(n);
    rc = sqlite3_exec(db, sql.c_str(), NRequestCallback, (void *)(&requests),
                      &zErrMsg);
    if (rc != SQLITE_OK) {
        std::cerr << "exec error: " << zErrMsg << std::endl;
        sqlite3_free(zErrMsg);
        return -1;
    }
    return requests.size();
}

std::vector<long long int> DA::StringToVector(std::string list) {
    std::string delimiter = ",";
    std::string num;
    std::vector<long long int> result;
    size_t pos;
    while ((pos = list.find(delimiter)) != std::string::npos) {
        num = list.substr(0, pos);
        result.push_back(std::stoll(num));
        list.erase(0, pos + delimiter.length());
    }
    return result;
}

template <typename T> std::string DA::VectorToString(std::vector<T> &list) {
    std::string result = "";
    std::string delimiter = ",";
    for (auto i : list) {
        result.append(std::to_string(i));
        result.append(delimiter);
    }
    return result;
}
} // namespace cargo
