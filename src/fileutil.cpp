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
// file and archive utility
// *****************************************************************************
//

#include <stdarg.h>			// variable type
#include "fileutil.h"

// write formatted string in general log
extern void logUsr(const char * format, ...);

extern char PathCurrDir[PATH_MAX];			// path current directory

char FullNameProgram[PATH_MAX];

//
int csplitpath(char *path,
			char *onlyPath,
			char *onlyFilename,		// file name and path
			char *onlyName,
			char *onlyExt)
{
	int len;
	char temp[PATH_MAX];
	char *ptr, *ptrName;

	const int separator = '/';			// dir separator

	// initialize path and filename
	if(onlyPath != NULL)
	{
		memset(onlyPath, 0, 1);			// Sets the first byte of the string to 0
	}
	if(onlyFilename != NULL)
	{
		memset(onlyFilename, 0, 1);		// Sets the first byte of the string to 0
	}
	if(onlyName != NULL)
	{
		memset(onlyName, 0, 1);			// Sets the first byte of the string to 0
	}
	if(onlyExt != NULL)
	{
		memset(onlyExt, 0, 1);			// Sets the first byte of the string to 0
	}
	if(path == NULL)
	{
		return(1);
	}
	len = strlen(path);					// get the length
	if(len == 0)
	{
		// the path string is ""
		return(0);
	}
	strncpy(temp, path, PATH_MAX);	// get temp path for analysis

	ptr = strrchr( temp, separator );

	if(ptr==NULL)
	{
		// NO DIRECTORY SEPARATOR
		if(onlyPath != NULL)
		{
			strcpy(onlyPath, ".");
		}
		if(onlyFilename != NULL)
		{
			strcpy(onlyFilename, temp);
		}
		ptr = strrchr( temp, '.' );
		if(ptr!=NULL)
		{
			// there is an extension
			*ptr = 0;
			if(onlyExt != NULL)
			{
				strcpy(onlyExt, ptr+1);
			}
		}
		if(onlyName != NULL)
		{
			strcpy(onlyName, temp);
		}
	}
	else
	{
		// there is a directory separator
		*ptr = 0;
		if(onlyPath != NULL)
		{
			strcpy(onlyPath, temp);
		}
		ptr++;
		ptrName=ptr;
		if(onlyFilename != NULL)
		{
			strcpy(onlyFilename, ptrName);
		}
		ptr = strrchr( ptrName, '.' );
		if(ptr!=NULL)
		{
			// there is an extension
			*ptr = 0;
			if(onlyExt != NULL)
			{
				strcpy(onlyExt, ptr+1);
			}
		}
		if(onlyName != NULL)
		{
			strcpy(onlyName, ptrName);
		}
	}
	return 0;
}

// get the full path of file name
//
char *RealPath(const char *RelfName, char *FullfName)
{
	char *pchr;
	int fCurrDir;			// flag current dir

	fCurrDir = 0;
	if(FullfName == NULL)
	{
		return(FullfName);
	}
	if(RelfName == NULL)
	{
		FullfName[0] = 0;
		return(FullfName);
	}
	pchr=(char *)RelfName;
	if(pchr[0] == 0)
	{
		FullfName[0] = 0;
		return(0);
	}
	if(pchr[0]=='.')
	{
		if(pchr[1]=='/')
		{
			// pchr relative to the pchr current dir
			pchr=pchr+2;
			fCurrDir = 1;			// pchr relative to current directory
		}
		else if(pchr[1]!='.')
		{
			// hidden file relative to the pchr current dir
			fCurrDir = 1;			// pchr relative to current directory
		}
		if(fCurrDir)
		{
			snprintf(FullfName, PATH_MAX, "%s/%s",	PathCurrDir, pchr);
			return(FullfName);
		}
	}
	else if(pchr[0]!='/')
	{
		snprintf(FullfName, PATH_MAX, "%s/%s",	PathCurrDir, RelfName);
		return(FullfName);
	}
	// for all other cases use realpath
	return(realpath(RelfName, FullfName));
}

// same as fopen, but create a full path before open the file
FILE *fopenFullPath(const char *filename, const char *mode)
{
	char FullfName[PATH_MAX];

	RealPath(filename, FullfName);

	logUsr("fopenFullPath=[%s]\n", filename);
	logUsr("fopenFullPath=full path[%s]\n", FullfName);

	return( fopen(FullfName, mode) );
}

// ---------------------------------------------
// get absolute path of current program
void GetAbsolutePathProgram(char *path_currdir)
{
	int i, len;
	// get program name with full path
	readlink("/proc/self/exe", FullNameProgram, PATH_MAX);
	// get the path of the program
	len = strlen(FullNameProgram);		// get the length
	if(len == 0)
	{
		// the path string is ""
		path_currdir[0] = 0;
		return;
	}
	strcpy(path_currdir, FullNameProgram);		// get path for analysis
	// check the file name
	for (i = (len-1); i>=0; i--)
	{
		if (path_currdir[i] == '/')
		{
			path_currdir[i] = 0;
			break;
		}
	}
}
			
// ---------------------------------------------
// SHA1 utility
// ---------------------------------------------
//

// -------------------------------------
// calc sha1 checksum.
// example:
// sha1 "t.txt"
// invoke sha1() and store its result to $out variable
// out=$(sha1 ${dbinput})
int sha1(char *chksum, char *file_name)
{
	FILE *fp;
	char cmd[2048];
	char path[1035];

	if(chksum==NULL)
		return(0);
	if(file_name==NULL)
		return(0);

	*chksum = 0;				// init chksum

	// Open the command for reading.
	sprintf(cmd,
		"sha1sum -b %s",
		file_name);

	fp = popen(cmd, "r");
	if (fp == NULL)
	{
		// Failed to run command
		return(0);
	}
	// Read the output a line at a time - output it.
	while (fgets(path, sizeof(path)-1, fp) != NULL)
	{
		path[40]=0;			// sha1 is substring of 40 char from left to right
		strcpy(chksum, path);
		break;
	}
	pclose(fp);		// close
	return(1);
}

// -------------------------------------
// verify sha1 code of file
// calling example:
//	if [ $(chksha1 ${dbinput} ${dbchksum}) -eq "1" ]; then
//		# different sha1. Do something
//	fi
int chksha1(char *chksum, char *file_name)
{
	char key[40];
	if(chksum==NULL)
		return(0);
	if(file_name==NULL)
		return(0);

	if(sha1(key, file_name) == 0)
	{
		// error
		return(0);
	}
	if(strcmp(key,chksum) != 0)
	{
		// different strings.
		return(0);
	}
	return(1);
}

//
// end SHA1 utility
// ---------------------------------------------
//

// ---------------------------------------------
// file utilities
// ---------------------------------------------

// create a bin file
void SaveBinFile(char *name, void *ptr, size_t n_bytes)
{
	FILE *fp;

	printf("SaveBinFile - name[%s] (%li)\n", name, (int)n_bytes);

	fp = fopen( name , "wb" );
	fwrite(ptr , sizeof(char) , n_bytes , fp );
	fflush(fp);
	fclose(fp);
}

// ---------------------------------------------
// 7zip: archive utilities
// ---------------------------------------------
//
// compress files named a list
// return: -1 on error, status of the command otherwise.
int archive7zListfiles(char *name_arc, char *list_of_filenames)
{
	char cmd[2048];
	// redirect output of the command to null
	sprintf(cmd,
		"7z a %s %s > /dev/null 2>&1",
		name_arc, list_of_filenames);
    return(system(cmd));
}

// check a 7z archive
// return a value != 0 if the archive is Ok
int check7zArchive(char *name_arc)
{
	FILE *fp;
	char cmd[2048];
	char path[1035];
	int ris;

	if(name_arc==NULL)
		return(0);

	// Open the command for reading.
	sprintf(cmd,
		"7z t %s > tst7z.txt 2>&1",
		name_arc);
    system(cmd);

	// read the result
	ris = 0;
	fp = fopen("tst7z.txt", "r");
	if (fp == NULL)
	{
		// Failed to run command
		return(0);
	}
	// Read the output a line at a time - output it.
	while (fgets(path, sizeof(path)-1, fp) != NULL)
	{
		if(strcmp(path, "Everything is Ok\n") == 0)
		{
			ris = 1;
			break;
		}
	}
	fclose(fp);		// close
	return(ris);
}

// ---------------------------------------------
// extract utility using 7zip
// ---------------------------------------------
// extract a file without path
// return: -1 on error, status of the command otherwise.
int extract7z_noPath(char *name_arc, char *dest_dir, char *file_name)
{
	char cmd[2048];
	// redirect output of the command to null
	sprintf(cmd,
		"7z e -y %s %s -o\"%s\" > /dev/null 2>&1",
		name_arc, file_name, dest_dir);
	// printf("extract_no_path[%s]\n",cmd);
    return(system(cmd));
}

// return: -1 on error, status of the command otherwise.
int extract7z(char *name_arc, char *destination)
{
	char cmd[2048];
	// redirect output of the command to null
	sprintf(cmd,
		"7z e %s -o\"%s\" -y > /dev/null 2>&1",
		name_arc, destination);
    return(system(cmd));
}

// ---------------------------------------------
// get zip info
// ---------------------------------------------
//
int GetZipInfo(char *name_arc, char *name_infozip)
{
	char cmd[2048];
	// redirect output of the command to null
	sprintf(cmd,
		"zipinfo -l %s > %s",
		name_arc, name_infozip);
    return(system(cmd));
}

// decode first line zipinfo with zip name archive.
void zi_GetNameZip(char *line, char *fullname, char *name, char *ext)
{
	int len;

	len=strlen(line);
	strncpy(fullname, &line[10], len - 10 + 1);
	csplitpath(fullname,
			NULL,
			NULL,		// file name and path
			name,
			ext);
}

// decode second line zipinfo with file size and entries.
void zi_GetSizeZip(char *line, long *size, int *n_entries)
{
	// char fullname[PATH_MAX];
	// int len;
	char *pchr;
	const char delimiters[] = " \t";
	int i;

	// len=strlen(line);

	// skip 3 token "Zip file size:" and get file size
	pchr = strtok(line, delimiters);
	for(i=0;i<3;i++)
	{
		pchr = strtok(NULL, delimiters);
	}
	sscanf(pchr, "%li", size);
	// skip 5 token "bytes, number of entries:" and get n_entries
	pchr = strtok(NULL, delimiters);
	for(i=0;i<4;i++)
	{
		pchr = strtok(NULL, delimiters);
	}
	sscanf(pchr, "%i", n_entries);
}


// decode 3..n_entries line zipinfo with entry info.
// Return 0 if the entry is a directory 1, if the entry is file
//
int zi_GetEntryZip(char *line, zi_entry_info *entry)
{
	// int len;
	char *pchr;
	char ext[32];
	const char delimiters[] = " \t";
	int i;

	// len=strlen(line);

	// get token file attrib
	pchr = strtok(line, delimiters);
	strncpy(&entry->attrib[0], pchr, 8);
	// skip 2 tokens and get the file size
	for(i=0;i<3;i++)
		pchr = strtok(NULL, delimiters);
	sscanf(pchr, "%li", &entry->size);
	if(entry->size == 0)
	{
		// file size 0 or directory
		return 0;
	}
	// skip 1 token "bx" and get compressed
	for(i=0;i<2;i++)
		pchr = strtok(NULL, delimiters);
	sscanf(pchr, "%li", &entry->compress);
	// skip 1 token and get date
	for(i=0;i<2;i++)
		pchr = strtok(NULL, delimiters);
	sscanf(pchr, "%s", &entry->date[0]);
	// get time
	pchr = strtok(NULL, delimiters);
	sscanf(pchr, "%s", &entry->time[0]);
	// get full name
	pchr = strtok(NULL, delimiters);
	sscanf(pchr, "%s", &entry->fullname[0]);
	// decode name
	csplitpath(&entry->fullname[0],
			&entry->path[0],
			NULL,		// file name and path
			&entry->name[0],
			&entry->ext[0]);
	// if ext is hgt, the entry is ok
	strncpy(ext, entry->ext, 32);
	for(i = 0; ext[i]; i++)
	{
		ext[i] = tolower(ext[i]);
	}
	if(strcmp(ext, "hgt") == 0)
	{
		return(1);
	}
	return(0);
}

// ---------------------------------------------
// bz2 file utilities
// ---------------------------------------------

char opened=0;
int bzerror;

// This function returns at most one less than 'length' number
// of characters from a bz2 compressed file whose file descriptor
// is pointed to by *bzfd.  In operation, a buffer is filled with
// uncompressed data (size = BZBUFFER), which is then parsed
// and doled out as NULL terminated character strings every time
// this function is invoked.  A NULL string indicates an EOF
// or error condition.
//
char *BZfgets(BZFILE *bzfd, unsigned length)
{

	static int x, y, nBuf;
	static char buffer[BZBUFFER+1], output[BZBUFFER+1];
	char done=0;

	if (opened!=1 && bzerror==BZ_OK)
	{
		// First time through.  Initialize everything!
		x=0;
		y=0;
		nBuf=0;
		opened=1;
		output[0]=0;
	}
	do
	{
		if (x==nBuf && bzerror!=BZ_STREAM_END && bzerror==BZ_OK && opened)
		{
			// Uncompress data into a static buffer

			nBuf=BZ2_bzRead(&bzerror, bzfd, buffer, BZBUFFER);
			buffer[nBuf]=0;
			x=0;
		}

		// Build a string from buffer contents

		output[y]=buffer[x];

		if (output[y]=='\n' || output[y]==0 || y==(int)length-1)
		{
			output[y+1]=0;
			done=1;
			y=0;
		}

		else
		y++;
		x++;

	} while (done==0);

	if (output[0]==0)
	opened=0;

	return (output);
}


// end of fileutil.cpp