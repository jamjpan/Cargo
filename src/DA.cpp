#include "libcargo/DA.h"
#include <iostream>

namespace cargo
{
DA::DA()
{
  rc = sqlite3_open(":memory:", &db);
  if (rc != SQLITE_OK) {
    std::cerr << "Cannot open database: " << sqlite3_errmsg(db) << std::endl;
    sqlite3_close(db);
  } else {
    // create vehicle table
    rc = sqlite3_exec(
        db,
        "CREATE TABLE vehicle(id INT, o_id INT, d_id INT, early INT, late INT, "
        "demand INT, load INT, nnd INT, route TEXT, sched TEXT, lv_node INT, "
        "lv_stop INT, is_active INT)",
        NULL, NULL, &zErrMsg);
    if (rc != SQLITE_OK) {
      std::cerr << "SQL error: " << zErrMsg << std::endl;
      sqlite3_free(zErrMsg);
    }
    // create request table
    rc = sqlite3_exec(db,
                      "CREATE TABLE request(id INT, o_id INT, d_id INT, early "
                      "INT, late INT, demand INT)",
                      NULL, NULL, &zErrMsg);
    if (rc != SQLITE_OK) {
      std::cerr << "SQL error: " << zErrMsg << std::endl;
      sqlite3_free(zErrMsg);
    }
  }
}

DA::~DA()
{
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
    std::cerr << "Cannot close database: " << sqlite3_errmsg(db) << std::endl;
  }
}

int DA::AddVehicle(Vehicle* vehicle)
{
  sqlite3_stmt* stmt;
  rc = sqlite3_prepare_v2(
      db,
      "INSERT INTO vehicle(id, o_id, d_id, early, late, demand, "
      "load, nnd, route, sched, lv_node, lv_stop, is_active) "
      "VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
      -1, &stmt, NULL);
  if (rc != SQLITE_OK) {
    std::cerr << "prepare error: " << sqlite3_errmsg(db) << std::endl;
    return 1;
  } else {
    sqlite3_bind_int(stmt, 1, vehicle->id);
    sqlite3_bind_int(stmt, 2, vehicle->oid);
    sqlite3_bind_int(stmt, 3, vehicle->did);
    sqlite3_bind_int(stmt, 4, vehicle->early);
    sqlite3_bind_int(stmt, 5, vehicle->late);
    sqlite3_bind_int(stmt, 6, vehicle->demand);
    sqlite3_bind_int(stmt, 7, vehicle->load);
    sqlite3_bind_int(stmt, 8, vehicle->nnd);
    sqlite3_bind_text(stmt, 9, NULL, 0, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 10, NULL, 0, SQLITE_STATIC);
    sqlite3_bind_int(stmt, 11, vehicle->lv_node);
    sqlite3_bind_int(stmt, 12, vehicle->lv_stop);
    sqlite3_bind_int(stmt, 13, vehicle->is_active);
  }
  rc = sqlite3_step(stmt);
  if (rc != SQLITE_DONE) {
    std::cerr << "step error: " << sqlite3_errmsg(db) << std::endl;
    return 1;
  }
  rc = sqlite3_finalize(stmt);
  if (rc != SQLITE_OK) {
    std::cerr << "finalize error: " << sqlite3_errmsg(db) << std::endl;
    return 1;
  }
  return 0;
}

int DA::UpdateLocation(VehicleId vid, int lv_node, int nnd)
{
  sqlite3_stmt* stmt;
  rc = sqlite3_prepare_v2(db, "UPDATE vehicle SET lv_node = ?, nnd = ? WHERE id = ?", -1, &stmt, NULL);
  if (rc != SQLITE_OK) {
    std::cerr << "prepare error: " << sqlite3_errmsg(db) << std::endl;
    return 1;
  } else {
    sqlite3_bind_int(stmt, 1, lv_node);
    sqlite3_bind_int(stmt, 2, nnd);
    sqlite3_bind_int(stmt, 3, vid);
  }
  rc = sqlite3_step(stmt);
  if (rc != SQLITE_OK) {
    std::cerr << "step error: " << sqlite3_errmsg(db) << std::endl;
    return 1;
  }
  rc = sqlite3_finalize(stmt);
  if (rc != SQLITE_OK) {
    std::cerr << "finalize error: " << sqlite3_errmsg(db) << std::endl;
    return 1;
  }
  return 0;
}

}  // namespace cargo