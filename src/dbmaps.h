#ifndef DBMAPS_H
#define DBMAPS_H

#include <limits.h>
#include "sqlutil.h"

//---------------------------------------------------
// struct idx zipinfo table
typedef struct {
	int  id;						// INTEGER PRIMARY KEY
	char name[PATH_MAX];			// TEXT
	char ext[4];					// TEXT
	int size;						// INTEGER
	char sha1[41];					// TEXT
	int n_entries;					// INTEGER
} dbc_zip;

// struct idx mapinfo table
typedef struct {
	int  id;						// INTEGER PRIMARY KEY
	int  zip_id;					// INTEGER
	char filetime[32];				// DATETIME CREATION ID
	char path[PATH_MAX];			// TEXT
	char name[PATH_MAX];			// TEXT
	char ext[4];					// TEXT
	int  size;						// INTEGER
	char sha1[41];					// TEXT
	int  type;						// INTEGER
	int  rows_cols;					// INTEGER
	char lat_code[2];				// TEXT
	int  lat_value;					// INTEGER
	char lon_code[2];				// TEXT
	int  lon_value;					// INTEGER
	long n_holes;					// INTEGER
	int  min_h;						// INTEGER
	int  max_h;						// INTEGER
	double min_lat;					// REAL
	double min_lon;					// REAL
} dbc_map;

void initDbZipRecord(dbc_zip *record);
// insert row zip info in db
long DbZipInfoInsert(sqlite3* db, dbc_zip *record);

void initDbMapRecord(dbc_map *record);
// insert row zip info in db
long DbMapInfoInsert(sqlite3* db, dbc_map *record);

// create table zip info in db
void DbZipInfoCreate(sqlite3* db);
// create table map info in db
void DbMapInfoCreate(sqlite3* db);

// create an index db of RfTrack db
int DbMapsCreate(char *dbName);

// open the db index
sqlite3 *DbMapsOpen(char *dbName);

// decode the info
void DecodeZipInfo(sqlite3 *db, char *FileName);

// from the original SRTM map name, get the map record
int DbMapInfoReadFromMapName(sqlite3* db, char *srtm_map_name, dbc_map *record);

// unzip map and check entry
int UnzipMap(char *zipArchive, zi_entry_info *entry);

// remove map
void RemoveMap(zi_entry_info *entry);

int CheckMap(dbc_map *record);

// read row zip info stored in db with id=zip_id
int DbZipInfoRead(sqlite3* db, int zip_id, dbc_zip *record);

#endif			// #ifndef DBMAPS_H
