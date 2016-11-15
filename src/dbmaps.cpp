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
// sqlite3 map utility
// *****************************************************************************
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include "fileutil.h"
#include "dbmaps.h"

extern int MapHminmaxholes(char *mapName, int *min, int *max, long *n_holes);

//
//---------------------------
// database with map index
sqlite3 *dbidx;
// struct used to memorize db info
dbc_zip IdxZipRecord;
// struct used to memorize db info
dbc_map idxMap;

void initDbZipRecord(dbc_zip *record)
{
	record->id=0;					// INTEGER PRIMARY KEY
	record->name[0]=0;				// TEXT
	record->ext[0]=0;				// TEXT
	record->size=0;					// INTEGER
	record->sha1[0]=0;				// TEXT
	record->n_entries=0;			// INTEGER
}

// insert row zip info in db
long DbZipInfoInsert(sqlite3* db, dbc_zip *record)
{
	char sCmd[NCHR_QUERY];			// sCmd string
	int pos=0;

	sql_cmd(db, "begin");

	//----------------
	// insert into TABLE zipinfo
	pos=0;
	pos = pos + sprintf(&sCmd[pos], "INSERT INTO zipinfo(");
	pos = pos + sprintf(&sCmd[pos], "name,");				// archive name without ext
	pos = pos + sprintf(&sCmd[pos], "ext,");				// archive name extension
	pos = pos + sprintf(&sCmd[pos], "size,");				// archive file size in bytes (es: 34688134)
	pos = pos + sprintf(&sCmd[pos], "sha1,");				// SHA1 checksum of file
	pos = pos + sprintf(&sCmd[pos], "n_entries) ");			// n. entries (es: 25)

	pos = pos + sprintf(&sCmd[pos], "VALUES (");
	pos = pos + sprintf(&sCmd[pos], "'%s',", record->name);			// TEXT
	pos = pos + sprintf(&sCmd[pos], "'%s',", record->ext);			// TEXT
	pos = pos + sprintf(&sCmd[pos], "%i,", record->size);			// INTEGER
	pos = pos + sprintf(&sCmd[pos], "'%s',", record->sha1);			// TEXT
	pos = pos + sprintf(&sCmd[pos], "%i)", record->n_entries);		// INTEGER

	sql_cmd(db, sCmd);
	//----------------

	sql_cmd(db, "commit");
	return(GetLastInsertRowId(db));
}

//---------------------------------------------------

void initDbMapRecord(dbc_map *record)
{
	record->id=0;					// INTEGER PRIMARY KEY
	record->zip_id=0;				// INTEGER
	record->filetime[0]=0;			// DATETIME CREATION ID
	record->path[0]=0;				// TEXT
	record->name[0]=0;				// TEXT
	record->ext[0]=0;				// TEXT
	record->size=0;					// INTEGER
	record->sha1[0]=0;				// TEXT
	record->type=0;					// INTEGER
	record->rows_cols=0;			// INTEGER
	record->lat_code[0]=0;			// TEXT
	record->lat_value=0;			// INTEGER
	record->lon_code[0]=0;			// TEXT
	record->lon_value=0;			// INTEGER
	record->n_holes=0;				// INTEGER
	record->min_h=0;				// INTEGER
	record->max_h=0;				// INTEGER
	record->min_lat=0.0;			// REAL
	record->min_lon=0.0;			// REAL
}

// insert row zip info in db
long DbMapInfoInsert(sqlite3* db, dbc_map *record)
{
	char sCmd[NCHR_QUERY];			// sCmd string
	int pos=0;

	sql_cmd(db, "begin");

	//----------------
	// insert into TABLE zipinfo
	pos=0;
	pos = pos + sprintf(&sCmd[pos], "INSERT INTO mapinfo(");
	pos = pos + sprintf(&sCmd[pos], "zip_id,");				// id of archive that stores the map
	pos = pos + sprintf(&sCmd[pos], "filetime,");			// datetime creation id
	pos = pos + sprintf(&sCmd[pos], "path,");				// path name stored in zip
	pos = pos + sprintf(&sCmd[pos], "name,");				// archive name without ext
	pos = pos + sprintf(&sCmd[pos], "ext,");				// archive name extension
	pos = pos + sprintf(&sCmd[pos], "size,");				// archive file size in bytes (es: 34688134)
	pos = pos + sprintf(&sCmd[pos], "sha1,");				// SHA1 checksum of file
	pos = pos + sprintf(&sCmd[pos], "type,");				//  map type: 1 for SRTM 1 Arc-Second, 3 for SRTM 3 Arc-Second
	pos = pos + sprintf(&sCmd[pos], "rows_cols,");				//  map rows & cols number
	pos = pos + sprintf(&sCmd[pos], "lat_code,");			// map latitude code N nord, S sud
	pos = pos + sprintf(&sCmd[pos], "lat_value,");			// map latitude value (0..90 nord, or 0..90 sud)
	pos = pos + sprintf(&sCmd[pos], "lon_code,");			// map longitude code W west, E east
	pos = pos + sprintf(&sCmd[pos], "lon_value,");			// map longitude value (0..180 west or 0..180 east
	pos = pos + sprintf(&sCmd[pos], "n_holes,");			// map n. holes: H negative
	pos = pos + sprintf(&sCmd[pos], "min_h,");				// map min H value
	pos = pos + sprintf(&sCmd[pos], "max_h,");				// map max H value
	pos = pos + sprintf(&sCmd[pos], "min_lat,");			// map min latitude value with sign (180..0..-180 degrees
	pos = pos + sprintf(&sCmd[pos], "min_lon)");			// map min longitude value with sign (90..0..-90 degrees

	pos = pos + sprintf(&sCmd[pos], "VALUES (");
	pos = pos + sprintf(&sCmd[pos], "%i,", record->zip_id);			// INTEGER
	pos = pos + sprintf(&sCmd[pos], "'%s',", record->filetime);		// TEXT
	pos = pos + sprintf(&sCmd[pos], "'%s',", record->path);			// TEXT
	pos = pos + sprintf(&sCmd[pos], "'%s',", record->name);			// TEXT
	pos = pos + sprintf(&sCmd[pos], "'%s',", record->ext);			// TEXT
	pos = pos + sprintf(&sCmd[pos], "%i,", record->size);			// INTEGER
	pos = pos + sprintf(&sCmd[pos], "'%s',", record->sha1);			// TEXT
	pos = pos + sprintf(&sCmd[pos], "%i,", record->type);			// INTEGER
	pos = pos + sprintf(&sCmd[pos], "%i,", record->rows_cols);		// INTEGER
	pos = pos + sprintf(&sCmd[pos], "'%s',", record->lat_code);		// TEXT
	pos = pos + sprintf(&sCmd[pos], "%i,", record->lat_value);		// INTEGER
	pos = pos + sprintf(&sCmd[pos], "'%s',", record->lon_code);		// TEXT
	pos = pos + sprintf(&sCmd[pos], "%i,", record->lon_value);		// INTEGER
	pos = pos + sprintf(&sCmd[pos], "%li,", record->n_holes);		// INTEGER
	pos = pos + sprintf(&sCmd[pos], "%i,", record->min_h);			// INTEGER
	pos = pos + sprintf(&sCmd[pos], "%i,", record->max_h);			// INTEGER
	pos = pos + sprintf(&sCmd[pos], "%lf,", record->min_lat);		// REAL
	pos = pos + sprintf(&sCmd[pos], "%lf)", record->min_lon);		// REAL

	sql_cmd(db, sCmd);
	//----------------

	sql_cmd(db, "commit");
	return(GetLastInsertRowId(db));
}

//---------------------------------------------------
// create table zip info in db
void DbZipInfoCreate(sqlite3* db)
{
	char sCmd[NCHR_QUERY];			// sCmd string
	int pos=0;

	sql_cmd(db, "begin");

	//----------------
	// TABLE dbzip
	pos=0;
	pos = pos + sprintf(&sCmd[pos], "CREATE TABLE zipinfo(");
	pos = pos + sprintf(&sCmd[pos], "'id' INTEGER PRIMARY KEY,");	// id counter of record
	// pos = pos + sprintf(&sCmd[pos], "'filetime' DATETIME,");		// date zip created (es: 12-Sep-29 23:58)
	pos = pos + sprintf(&sCmd[pos], "'name' TEXT,");				// archive filename without extension
	pos = pos + sprintf(&sCmd[pos], "'ext' TEXT,");					// archive extension (es: zip)
	pos = pos + sprintf(&sCmd[pos], "'size' INTEGER,");				// archive file size in bytes (es: 34688134)
	pos = pos + sprintf(&sCmd[pos], "'sha1' TEXT,");				// archive SHA1 checksum
	pos = pos + sprintf(&sCmd[pos], "'n_entries' INTEGER)");		// n. entries (es: 25)
	// pos = pos + sprintf(&sCmd[pos], "'compression' REAL)");			// % compression (es: 49.9)
	sql_cmd(db, sCmd);
	//----------------

	sql_cmd(db, "commit");
}

// create table map info in db
void DbMapInfoCreate(sqlite3* db)
{
	char sCmd[NCHR_QUERY];			// sCmd string
	int pos=0;

	sql_cmd(db, "begin");

	//----------------
	// TABLE acqinfo
	pos=0;
	pos = pos + sprintf(&sCmd[pos], "CREATE TABLE mapinfo(");
	pos = pos + sprintf(&sCmd[pos], "'id' INTEGER PRIMARY KEY,");// id counter of record
	pos = pos + sprintf(&sCmd[pos], "'zip_id' INTEGER,");		// id of archive that stores the map
	pos = pos + sprintf(&sCmd[pos], "'filetime' DATETIME,");	// date map stored (es: 12-Sep-29 23:58)
	pos = pos + sprintf(&sCmd[pos], "'path' TEXT,");			// map filename path stored in zip
	pos = pos + sprintf(&sCmd[pos], "'name' TEXT,");			// map filename without extension (es: N00W073)
	pos = pos + sprintf(&sCmd[pos], "'ext' TEXT,");				// map extension (es: hgt)
	pos = pos + sprintf(&sCmd[pos], "'size' INTEGER,");			// map file size in bytes (es: 2884802)
	pos = pos + sprintf(&sCmd[pos], "'sha1' TEXT,");			// map SHA1 checksum
	pos = pos + sprintf(&sCmd[pos], "'type' INTEGER,");			// map type: 1 for SRTM 1 Arc-Second, 3 for SRTM 3 Arc-Second
	pos = pos + sprintf(&sCmd[pos], "'rows_cols' INTEGER,");			// map rows & cols number
	pos = pos + sprintf(&sCmd[pos], "'lat_code' TEXT,");		// map latitude code N nord, S sud
	pos = pos + sprintf(&sCmd[pos], "'lat_value' INTEGER,");	// map latitude value (0..90 nord, or 0..90 sud)
	pos = pos + sprintf(&sCmd[pos], "'lon_code' TEXT,");		// map longitude code W west, E east
	pos = pos + sprintf(&sCmd[pos], "'lon_value' INTEGER,");	// map longitude value (0..180 west or 0..180 east
	pos = pos + sprintf(&sCmd[pos], "'n_holes' INTEGER,");		// n. holes (H negative values)
	pos = pos + sprintf(&sCmd[pos], "'min_h' INTEGER,");		// min H value
	pos = pos + sprintf(&sCmd[pos], "'max_h' INTEGER,");		// max H value
	pos = pos + sprintf(&sCmd[pos], "'min_lat' REAL,");			// map min latitude value with sign (180..0..-180 degrees
	pos = pos + sprintf(&sCmd[pos], "'min_lon' REAL)");			// map min longitude value with sign (90..0..-90 degrees
	sql_cmd(db, sCmd);
	//----------------

	sql_cmd(db, "commit");
}

// create an index db of RfTrack db
int DbMapsCreate(char *dbName)
{
	sqlite3* db;
	char sCmd[NCHR_QUERY];			// sCmd string
	// int pos=0;
	// int i;

	// delete eventually an old db
	sprintf(sCmd, "rm -f %s", dbName);
	system(sCmd);

	sqlite3_open(dbName, &db);

	if(db == 0) {
		fprintf(stderr, "Can't open database [%s]: %s\n",
		dbName,
		sqlite3_errmsg(db));
		return 1;
	}
	//
	// ------------------------------------------
	// create tables
	DbMetadataCreate(db);			// create table metadata
	DbZipInfoCreate(db);			// create table zip info in db
	DbMapInfoCreate(db);			// create table map info in db
	// ------------------------------------------
	//
	sqlite3_close(db);
	return 0;
}

// open the db index
sqlite3 *DbMapsOpen(char *dbName)
{
	sqlite3 *db;

	db = NULL;
	// check if file exist
	if( access( dbName, F_OK ) == -1 )
	{
		// file doesn't exist, create it
		if(DbMapsCreate(dbName))
		{
			// problem creating db
			return(db);
		}
	}
	sqlite3_open(dbName, &db);

	if(db == 0)
	{
		fprintf(stderr, "Can't open database [%s]: %s\n",
		dbName,
		sqlite3_errmsg(db));
		db = NULL;
	}
	return(db);
}

// unzip map and check entry
int UnzipMap(char *zipArchive, zi_entry_info *entry)
{
	char mapname[PATH_MAX];

	// recreate full name to extract command
	if( (entry->path[0]==0)||( (entry->path[0]=='.')&&(entry->path[1]==0) ) )
	{
		sprintf(mapname, "%s.%s", entry->name, entry->ext);
	}
	else
	{
		sprintf(mapname, "%s/%s.%s", entry->path,
				entry->name, entry->ext);
	}
	extract7z_noPath(zipArchive, ".", mapname);
	// set map name with extension
	sprintf(mapname, "%s.%s", entry->name, entry->ext);
	sha1(entry->sha1, mapname);
	return(1);
}

// remove map
void RemoveMap(zi_entry_info *entry)
{
	char cmd[2048];
	sprintf(cmd, "rm %s.%s", entry->name, entry->ext);
	system(cmd);
}

// check all data of map
// http://vterrain.org/Elevation/SRTM/
// http://pcsupport.about.com/od/fileextensions/f/hgtfile.htm
// http://gis.stackexchange.com/questions/43743/how-to-extract-elevation-from-hgt-file
//
int CheckMap(dbc_map *record)
{
	long lon_degrees;			// longitudine verso est
	long lat_degrees;			// latitudine verso nord

	int mapOk;
	char mapname[PATH_MAX];
	char chr;

	mapOk=0;

	if(record->size == 2884802)
	{
		record->type = 3;
		record->rows_cols = 1201;
	}
	else
	{
		record->type = 1;
		record->rows_cols = 3601;
	}
	// get latitude from map name
	chr=record->name[0];
	chr=toupper(chr);
	if( (chr!='N')&&(chr!='S') )
		return(mapOk);
	record->lat_code[0] = chr;
	record->lat_code[1] = 0;

	if(!isdigit(record->name[1]))
		return(mapOk);
	lat_degrees = (long)(record->name[1]-'0');

	lat_degrees = lat_degrees * 10;
	if(!isdigit(record->name[2]))
		return(mapOk);
	lat_degrees = lat_degrees + (long)(record->name[2]-'0');
	if(lat_degrees > 90)
	{
		return(mapOk);
	}
	record->lat_value = lat_degrees;
	if(record->lat_code[0]!='N')
	{
		lat_degrees = -lat_degrees;
	}
	record->min_lat = lat_degrees;

	// get longitude
	chr=record->name[3];
	chr=toupper(chr);
	if( (chr!='E')&&(chr!='W') )
		return(mapOk);

	record->lon_code[0] = chr;
	record->lon_code[1] = 0;

	if(!isdigit(record->name[4]))
		return(mapOk);
	lon_degrees = (long)(record->name[4]-'0');

	lon_degrees = lon_degrees * 10;
	if(!isdigit(record->name[5]))
		return(mapOk);
	lon_degrees = lon_degrees + (long)(record->name[5]-'0');

	lon_degrees = lon_degrees * 10;
	if(!isdigit(record->name[6]))
		return(mapOk);
	lon_degrees = lon_degrees + (long)(record->name[6]-'0');
	if(lon_degrees > 180)
	{
		return(mapOk);
	}

	record->lon_value = lon_degrees;
	if(record->lon_code[0]!='E')
	{
		lon_degrees = -lon_degrees;
	}
	record->min_lon = lon_degrees;

	// get min, max, n_holes
	sprintf(mapname, "%s.%s", record->name, record->ext);
	MapHminmaxholes(mapname, &record->min_h, &record->max_h, &record->n_holes);

	mapOk=1;

	return(mapOk);
}

// decode the info
void DecodeZipInfo(sqlite3 *db, char *FileName)
{
	char line[1024];
	char* ptr;
	// char field[128];
	char ZipName[PATH_MAX];
	char name[128];
	char ext[128];
	int nLine;
	long size;
	int n_entries;
    FILE* file;
	int count_entry;
	int len;
	zi_entry_info ziEntry;
	dbc_zip ZipRecord;
	dbc_map MapRecord;
	long idZipRecord;
	long idRecord;

	if((file= fopen(FileName, "r"))==NULL)
		return;
	nLine = 0;
	while(!feof(file))
	{
		fgets(line, sizeof(line), file);
		ptr = strstr(line, "\n");
		if(ptr)
		{
			*ptr = 0;
		}

		if(nLine == 0)
		{
			// get Archive name
			zi_GetNameZip(line, ZipName, name, ext);
		}
		else if(nLine == 1)
		{
			// Zip file size, number of entries
			zi_GetSizeZip(line, &size, &n_entries);
			count_entry=0;
		}
		else
		{
			if(count_entry==0)
			{
				// insert the zip entry in db
				initDbZipRecord(&ZipRecord);
				sprintf(&ZipRecord.name[0], "%s", name);
				sprintf(&ZipRecord.ext[0], "%s", ext);
				ZipRecord.size = size;
				sha1(ZipRecord.sha1, FileName);
				ZipRecord.n_entries = n_entries;
				idZipRecord = DbZipInfoInsert(db, &ZipRecord);
			}
			// decode entries
			if(count_entry<n_entries)
			{
				if(zi_GetEntryZip(line, &ziEntry))
				{
					// insert the map entry in db
					initDbMapRecord(&MapRecord);
					MapRecord.zip_id = idZipRecord;
					sprintf(&MapRecord.filetime[0], "%s %s",
						ziEntry.date, ziEntry.time);
					sprintf(&MapRecord.path[0], "%s", ziEntry.path);
					sprintf(&MapRecord.name[0], "%s", ziEntry.name);
					sprintf(&MapRecord.ext[0], "%s", ziEntry.ext);
					MapRecord.size = ziEntry.size;
					if(UnzipMap(ZipName, &ziEntry))
					{
						sprintf(&MapRecord.sha1[0], "%s", ziEntry.sha1);
					}
					if(CheckMap(&MapRecord))
					{
						idRecord = DbMapInfoInsert(db, &MapRecord);
					}
					RemoveMap(&ziEntry);
				}
				count_entry++;
			}
		}
		nLine++;
	}
	fclose(file);
}

// ---------------------------------------------
// get map data from sqlite db

// pointer to record dbc_map
int flag_dbc_map_found = 0;				// != 0 if found map name in database
dbc_map *ptr_dbc_map = NULL;

// get one record dbc_map
static int DbMapInfoRead_callback(void *data, int argc, char **argv, char **azColName)
{
	char create_date[16];					// DATETIME is divided in DATE & TIME
	char create_time[16];					// DATETIME is divided in

	if(argc<=0)
	{
		// no elements
		flag_dbc_map_found = 0;
		return(0);
	}

	// read the map info
	//
	sscanf( argv[ 0], "%d", &(ptr_dbc_map->id));					// INTEGER PRIMARY KEY
	sscanf( argv[ 1], "%d",  &(ptr_dbc_map->zip_id));				// INTEGER
	sscanf( argv[ 2], "%s %s", &(create_date[0]), &(create_time[0]));		// DATETIME
	sprintf(&(ptr_dbc_map->filetime[0]), "%s %s", create_date, create_time);

	sscanf( argv[ 3], "%s", &(ptr_dbc_map->path[0]));			// TEXT
	sscanf( argv[ 4], "%s", &(ptr_dbc_map->name)[0]);			// TEXT
	sscanf( argv[ 5], "%s", &(ptr_dbc_map->ext)[0]);			// TEXT
	sscanf( argv[ 6], "%d", &(ptr_dbc_map->size));				// INTEGER
	sscanf( argv[ 7], "%s", &(ptr_dbc_map->sha1[0]));			// TEXT
	sscanf( argv[ 8], "%d", &(ptr_dbc_map->type));				// INTEGER
	sscanf( argv[ 9], "%d", &(ptr_dbc_map->rows_cols));			// INTEGER
	sscanf( argv[10], "%s", &(ptr_dbc_map->lat_code[0]));		// TEXT
	sscanf( argv[11], "%d", &(ptr_dbc_map->lat_value));			// INTEGER
	sscanf( argv[12], "%s", &(ptr_dbc_map->lon_code[0]));		// TEXT
	sscanf( argv[13], "%d", &(ptr_dbc_map->lon_value));			// INTEGER
	sscanf( argv[14], "%ld", &(ptr_dbc_map->n_holes));			// INTEGER
	sscanf( argv[15], "%d", &(ptr_dbc_map->min_h));				// INTEGER
	sscanf( argv[16], "%d", &(ptr_dbc_map->max_h));				// INTEGER
	sscanf( argv[17], "%lf", &(ptr_dbc_map->min_lat));			// REAL
	sscanf( argv[18], "%lf", &(ptr_dbc_map->min_lon));			// REAL

	flag_dbc_map_found = 1;
	return(0);
}

// from the original SRTM map name, get the map record
int DbMapInfoReadFromMapName(sqlite3* db, char *srtm_map_name, dbc_map *record)
{
	char query[NCHR_QUERY];				// query string
	// int pos=0;

	flag_dbc_map_found = 0;				// != 0 if found map name in database

	if(record==NULL)
	{
		return(flag_dbc_map_found);
	}

	sql_cmd(db, "begin");

	ptr_dbc_map = record;			// pass pointer to callback function
	//----------------
	// select record search case insensitive
	sprintf(query, "SELECT * FROM mapinfo WHERE name='%s'  COLLATE NOCASE", srtm_map_name);
	execute_sql(db, query, DbMapInfoRead_callback, 0);
	//----------------

	sql_cmd(db, "commit");

	return(flag_dbc_map_found);
}

// pointer to record dbc_idx
int flag_dbc_zip_found = 0;				// != 0 if found zip in database

dbc_zip *ptr_dbc_zip = NULL;

// get one record ZipInfo
static int DbZipInfoRead_callback(void *data, int argc, char **argv, char **azColName)
{
	if(argc<=0)
	{
		// no elements
		flag_dbc_zip_found = 0;
		return(0);
	}

	sscanf( argv[ 0], "%d", &(ptr_dbc_zip->id));				// INTEGER PRIMARY KEY
	sscanf( argv[ 1], "%s", &(ptr_dbc_zip->name)[0]);			// TEXT
	sscanf( argv[ 2], "%s", &(ptr_dbc_zip->ext)[0]);			// TEXT
	sscanf( argv[ 3], "%d", &(ptr_dbc_zip->size));				// INTEGER
	sscanf( argv[ 4], "%s", &(ptr_dbc_zip->sha1[0]));			// TEXT
	sscanf( argv[ 5], "%d", &(ptr_dbc_zip->n_entries));			// INTEGER

	flag_dbc_zip_found = 1;

	return 0;
}

// read row zip info stored in db with id=zip_id
int DbZipInfoRead(sqlite3* db, int zip_id, dbc_zip *record)
{
	char query[NCHR_QUERY];			// query string
	// int pos=0;

	flag_dbc_zip_found = 0;				// != 0 if found zip in database

	if(record==NULL)
	{
		return(flag_dbc_zip_found);
	}

	sql_cmd(db, "begin");

	ptr_dbc_zip = record;			// pass pointer to callback function
	//----------------
	// select record with MAX(id)
	sprintf(query, "SELECT * FROM ZipInfo WHERE id = %i",
				zip_id);
	execute_sql(db, query, DbZipInfoRead_callback, 0);
	//----------------

	sql_cmd(db, "commit");

	return(flag_dbc_zip_found);
}


//
// END SQLITE UTILITY
// ---------------------------------------------
