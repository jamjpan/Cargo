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
    rc = sqlite3_exec(db, "CREATE TABLE t(x INTEGER);", callback, 0, &zErrMsg);
    rc = sqlite3_exec(db, "INSERT INTO t VALUES(42);", callback, 0, &zErrMsg);
    rc = sqlite3_exec(db, "SELECT * FROM t;", callback, 0, &zErrMsg);
    if (rc != SQLITE_OK) {
      fprintf(stderr, "SQL error: %s\n", zErrMsg);
      sqlite3_free(zErrMsg);
    }
  }
  sqlite3_close(db);
}
