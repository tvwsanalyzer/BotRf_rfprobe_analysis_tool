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
// *****************************************************************************
// sqlite3 utility
// *****************************************************************************

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <sqlite3.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <stdarg.h>			// variable type
#include "sqlutil.h"

// write formatted string in general log
extern void logUsr(const char * format, ...);

//---------------------------
//
// using namespace std;
static int callback(void *NotUsed, int argc, char **argv, char **azColName){
	int i;
	printf("--------------------- function callback\n");
	for(i=0; i<argc; i++){
		printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
	}
	printf("\n");
	return 0;
}

/**
static int select_callback(void *data, int argc, char **argv, char **azColName){
	int i;
	for(i=0; i<argc; i++){
		printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
	}
	printf("\n");
	return 0;
}
**/

void execute_sql(sqlite3 *db, char zSql[], sqlite3_callback xCallback, void *pArg){

	int  rc;
	char *pzErrMsg = 0;
	rc = sqlite3_exec(db, zSql, xCallback, pArg, &pzErrMsg);
	if( rc != SQLITE_OK ){
		fprintf(stderr, "SQL  %s\n", pzErrMsg);
		sqlite3_free(pzErrMsg);
	}
}

// execute sql statement without parameters
void sql_cmd(sqlite3 *db, const char* stmt) {
	int   ret;
	char *errmsg = 0;

	ret = sqlite3_exec(db, stmt, 0, 0, &errmsg);

	if(ret != SQLITE_OK) {
		printf("SQL  %s [%s].\n", stmt, errmsg);
		sqlite3_free(errmsg);
	}
}

// Open database
void connect(sqlite3 **db, char *db_name)
{
	int  rc;
	rc = sqlite3_open(db_name, db);
	if( rc )
	{
		fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(*db));
		exit(0);
	}
	else
	{
		// fprintf(stdout, "Opened database successfully\n");
	}
}

// return 0 if the file is not an sqlite db
int isDbSqlite3(char *file_name)
{
	FILE *fp;
	char cmd[2048];
	char path[1035];

	if(file_name==NULL)
		return(0);

	// Open the command for reading.
	sprintf(cmd,
		"file -b %s",
		file_name);

	fp = popen(cmd, "r");
	if (fp == NULL)
	{
		// Failed to run command
		return(0);		// no sqlite 3 db
	}
	// Read the output a line at a time - output it.
	while (fgets(path, sizeof(path)-1, fp) != NULL)
	{
		if (strncmp(path,"SQLite 3.x database",19) == 0)
		{
			// is a sqlite db
			return(1);
		}
	}
	pclose(fp);		// close
	return(0);		// no sqlite 3 db
}

int fDbAnswerOk=0;	// 1 if the result answer is OK

// if return 1, callback want a query abort !!!
static int DbCheckOk_callback(void *data, int argc, char **argv, char **azColName)
{
	int i;
	char result[10];
	for(i=0; i<argc; i++)
	{
		if(i==0)
		{
			// get the result string
			strncpy(result, argv[i], 10);
			if(tolower(result[i])!='o')
				return(0);
			if(tolower(result[i+1])!='k')
				return(0);
			if((result[i+2])!=0)
				return(1);
			fDbAnswerOk=1;
			return(0);
		}
	}
	return 0;
}

// return 0 if the db has a problem
int DbCheckIntegrity(sqlite3 *db)
{
	char query[NCHR_QUERY];

	fDbAnswerOk=0;	// 1 if the result answer is OK
	sprintf(query, "pragma integrity_check");
	execute_sql(db, query, DbCheckOk_callback, 0);
	return(fDbAnswerOk);
}

// Safe Open database
int DbSafeOpen(sqlite3 **db, char *db_name)
{
	int  rc;
	int ris;

	*db = NULL;			// initialize the pointer to handler
	ris = 0;

	// check if file is really a sqlite3 db
	if(isDbSqlite3(db_name) == 0)
	{
		// ERROR: file is not a db sqlite
		logUsr("ERROR: file is not a db sqlite\n");
		return(ris);
	}
	// verified: the file is a db sqlite

	rc = sqlite3_open(db_name, db);
	if( rc )
	{
		// ERROR: can't open database
		logUsr("Can't open database: %s\n", sqlite3_errmsg(*db));
		return(ris);
	}
	// Opened database successfully
	if(DbCheckIntegrity(*db) == 0)
	{
		// ERROR: db integrity check
		logUsr("ERROR: db integrity check\n");
		sqlite3_close(*db);	// Close database
		*db = NULL;			// initialize the pointer to handler
		return(ris);
	}
	// DB INTEGRITY CHECK OK
	// -----------------------------------
	// the file has passed all tests
	ris = 1;
	return(ris);
}

int dbNRowsTable=-1;	// n. of rows in table
char dbNameTable[32];	// table name
static int DbTableNRows_callback(void *data, int argc, char **argv, char **azColName)
{
	if(argc==1)
	{
		sscanf( argv[0], "%i", &dbNRowsTable );
	}
	return 0;
}

// return n rows in table. If return < 0, the table not exist
int DbTableNRows(sqlite3 *db, char *table_name)
{
	char query[NCHR_QUERY];

	strncpy(dbNameTable, table_name, 32);
	dbNameTable[31]=0;
	dbNRowsTable=-1;	// init n. rows in table
	sprintf(query, "SELECT count(*) FROM sqlite_master WHERE type='table' AND name='%s'",
				dbNameTable);
	execute_sql(db, query, DbTableNRows_callback, 0);
	if(dbNRowsTable<=0)
	{
		dbNRowsTable=-1;	// table not exist
		return(dbNRowsTable);
	}
	sprintf(query, "SELECT count(*) FROM %s",
				dbNameTable);
	execute_sql(db, query, DbTableNRows_callback, 0);
	return(dbNRowsTable);
}

char str_number[256];
// get one number from db
static int DbGet1Num_callback(void *data, int argc, char **argv, char **azColName)
{
	str_number[0]=0;

	strncpy(str_number, argv[0], 255 );
	str_number[255] = 0;

	return 0;
}

// return only one double from the db.
// Used to execute query for coordinates
double DbGet1Double(sqlite3 *db, char *table, char *str_cols)
{
	char query[NCHR_QUERY];
	double ris;

	ris = 0.0;
	sprintf(query, "SELECT %s FROM %s", str_cols, table);
	execute_sql(db, query, DbGet1Num_callback, 0);
	sscanf( str_number, "%lf", &ris );
	return(ris);
}

// return the count of elements in db with conditions.
int DbGetCount(sqlite3 *db, char *table, char *str_condition)
{
	char query[NCHR_QUERY];
	int ris;

	ris = 0;
	sprintf(query, "SELECT COUNT(*) FROM %s	WHERE %s", table, str_condition);
	execute_sql(db, query, DbGet1Num_callback, 0);
	sscanf( str_number, "%i", &ris );
	return(ris);
}
// return only one integer from the db.
int DbGet1Int(sqlite3 *db, char *table, char *str_cols)
{
	char query[NCHR_QUERY];
	int ris;

	ris = 0;
	sprintf(query, "SELECT %s FROM %s", str_cols, table);
	execute_sql(db, query, DbGet1Num_callback, 0);
	sscanf( str_number, "%i", &ris );
	return(ris);
}
// return only one long from the db.
long DbGet1Long(sqlite3 *db, char *table, char *str_cols)
{
	char query[NCHR_QUERY];
	long ris;

	ris = 0;
	sprintf(query, "SELECT %s FROM %s", str_cols, table);
	execute_sql(db, query, DbGet1Num_callback, 0);
	sscanf( str_number, "%li", &ris );
	return(ris);
}
// return only one DATETIME from the db.
void DbGet1DateTime(char *RisDateTime, sqlite3 *db, char *table, char *str_cols)
{
	char query[NCHR_QUERY];

	sprintf(query, "SELECT %s FROM %s", str_cols, table);
	execute_sql(db, query, DbGet1Num_callback, 0);
	// Remove milliseconds. Only date and time with seconds
	strncpy(RisDateTime, str_number, 19);
	RisDateTime[19]=0;
	return;
}

// create table metadata
void DbMetadataCreate(sqlite3* db)
{
	sql_cmd(db, "begin");

	//----------------
	// TABLE metadata
	sql_cmd(db, "CREATE TABLE metadata(locale TEXT)");
	// insert directly the row unique value locale
	sql_cmd(db, "INSERT INTO metadata (locale) VALUES ('en_US')");
	//----------------

	sql_cmd(db, "commit");
}

//---------------------------------------------------
// get the last insert rowid
long dbLastInsertRowId=-1L;	// last inserted row id

static int DbLastInsertRowId_callback(void *data, int argc, char **argv, char **azColName)
{
	if(argc==1)
	{
		sscanf( argv[0], "%li", &dbLastInsertRowId );
	}
	return 0;
}

long GetLastInsertRowId(sqlite3* db)
{
	char query[NCHR_QUERY];

	sprintf(query, "SELECT last_insert_rowid();");
	execute_sql(db, query, DbLastInsertRowId_callback, 0);
	return(dbLastInsertRowId);
}

// end of sqlutil.cpp