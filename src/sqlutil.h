// *****************************************************************************
// This module is part of RfProbe, RF Signal Path Loss And Terrain Analysis Tool
// RfProbe is a fork of Splat!, project of J. A. Magliacane
// http://www.qsl.net/kd2bd/splat.html
// *****************************************************************************
//
//   This program is free software; you can redistribute it and/or modify it
//   under the terms of the GNU General Public License as published by the
//   Free Software Foundation; either version 2 of the License or any later
//   version.
//
//   This program is distributed in the hope that it will useful, but WITHOUT
//   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
//   for more details.
//
// *****************************************************************************
//
#ifndef SQLUTIL_H
#define SQLUTIL_H

#include <sqlite3.h>

#define	NCHR_QUERY	4096

void execute_sql(sqlite3 *db, char zSql[], sqlite3_callback xCallback, void *pArg);
void sql_cmd(sqlite3 *db, const char* stmt);
// Open database
void connect(sqlite3 **db, char *db_name);
// return 0 if the file is not an sqlite db
int isDbSqlite3(char *file_name);
// return 0 if the db has a problem
int DbCheckIntegrity(sqlite3 *db);
// Safe Open database
int DbSafeOpen(sqlite3 **db, char *db_name);
// return n rows in table. If return < 0, the table not exist
int DbTableNRows(sqlite3 *db, char *table_name);
// return only one double from the db.
double DbGet1Double(sqlite3 *db, char *table, char *str_cols);
// return the count of elements in db with conditions.
int DbGetCount(sqlite3 *db, char *table, char *str_condition);
// return only one integer from the db.
int DbGet1Int(sqlite3 *db, char *table, char *str_cols);
// return only one long from the db.
long DbGet1Long(sqlite3 *db, char *table, char *str_cols);
// return only one DATETIME from the db.
void DbGet1DateTime(char *RisDateTime, sqlite3 *db, char *table, char *str_cols);
// create table metadata
void DbMetadataCreate(sqlite3* db);
// get the last insert rowid
long GetLastInsertRowId(sqlite3* db);



#endif			// #ifndef SQLUTIL_H
