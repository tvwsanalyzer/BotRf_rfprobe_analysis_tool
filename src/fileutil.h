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
#ifndef FILEUTIL_H
#define FILEUTIL_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>
#include <bzlib.h>

#define BZBUFFER 65536

// entry info:
// example:
// directory
// drwx---     6.3 fat        0 bx        0 stor 12-Sep-29 23:58 A20/
// file entry
// -rw-a--     6.3 fat  2884802 bx  1064428 defN 12-Sep-29 23:58 A20/N00W061.hgt

typedef struct
{
	char attrib[8];
	long size;
	long compress;
	char date[10];
	char time[10];
	char fullname[PATH_MAX];
	char path[PATH_MAX];
	char name[PATH_MAX];
	char ext[32];
	char sha1[41];			// sha1 checksum
} zi_entry_info;

// ---------------------------------------------

int csplitpath(char *path,
			char *onlyPath,
			char *onlyFilename,		// file name and path
			char *onlyName,
			char *onlyExt);

// get the full path of file name
char *RealPath(const char *RelfName, char *FullfName);

// same as fopen, but create a full path before open the file
FILE *fopenFullPath(const char *filename, const char *mode);

// get absolute path of current program
void GetAbsolutePathProgram(char *path_currdir);

// calc sha1 checksum.
int sha1(char *chksum, char *file_name);
// verify sha1 code of file
int chksha1(char *chksum, char *file_name);

// create a bin file
void SaveBinFile(char *name, void *ptr, size_t n_bytes);

// archive utility using 7zip
int archive7zListfiles(char *name_arc, char *file_name_list);
// return a value != 0 if the archive is Ok
int check7zArchive(char *name_arc);
// extract utility using 7zip
int extract7z_noPath(char *name_arc, char *dest_dir, char *file_name);
int extract7z(char *name_arc, char *destination);

// ---------------------------------------------
// zip info management

int GetZipInfo(char *name_arc, char *name_infozip);

// decode first line zipinfo with zip name archive.
void zi_GetNameZip(char *line, char *fullname, char *name, char *ext);

// decode second line zipinfo with file size and entries.
void zi_GetSizeZip(char *line, long *size, int *n_entries);

// decode 3..n_entries line zipinfo with entry info.
// Return 0 if the entry is a directory 1, if the entry is file
//
int zi_GetEntryZip(char *line, zi_entry_info *entry);

// ---------------------------------------------
// bz2 file utilities
// ---------------------------------------------
// This function returns at most one less than 'length' number
// of characters from a bz2 compressed file whose file descriptor
// is pointed to by *bzfd.  In operation, a buffer is filled with
// uncompressed data (size = BZBUFFER), which is then parsed
// and doled out as NULL terminated character strings every time
// this function is invoked.  A NULL string indicates an EOF
// or error condition.
//
char *BZfgets(BZFILE *bzfd, unsigned length);

#endif			// #ifndef FILEUTIL_H
