// *****************************************************************************
// 	   rfprobe: An RF Signal Path Loss And Terrain Analysis Tool
// *****************************************************************************
// RfProbe is a fork of Splat!, project started in 1997 by John A. Magliacane
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
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <sqlite3.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <ctype.h>
#include <time.h>
#include <stdarg.h>			// variable type
#include "fontdata.h"
#include "conversion.h"
#include "fileutil.h"
#include "dbmaps.h"
#include "maps.h"
#include "languages.h"
#include "dbgitwom.h"		// for debugging itwom3.0.cpp
//--------------------------------------------------------------------
// conditional compilation


// define ORI_STARTSITE_ANALYSIS if you want to start analysis from RxSite to TxSite (original)
// #define ORI_STARTSITE_ANALYSIS
#ifdef ORI_STARTSITE_ANALYSIS
	//----------------------------------------
	// graph start with RxSite position (original)
#define	GrpStartSite	RxSite
#define	GrpEndSite		TxSite
	//----------------------------------------
#else
	//----------------------------------------
	// graph start with TxSite position (mr version)
#define	GrpStartSite	TxSite
#define	GrpEndSite		RxSite
	//----------------------------------------
#endif

//--------------------------------------------------------------------
// mr: conditional compilation commands

// -----------------------------------
// Commands to enable functions in reports
//
// 08/07: disabled draw first fresnel zone
// define EN_DRAW_FRESNEL if you want to enable draw fresnel zone
// #define EN_DRAW_FRESNEL

// modify 07/07:
// define ORI_STARTSITE_ANALYSIS if you want to define the start with RxSite position (original in rfprobe)
// If not defined, the graph start with TxSite position
// #define ORI_STARTSITE_ANALYSIS

// added 04/07/2016:
// this define enable two rows in report.
// Actually is disabled because generate different results.
// For example:
// ...
// N. of obstructions detected: 1
// Antenna at marmolada must be raised to at least 31 m above groud level to clear all obstructions detected by BotRf .
// Antenna at marmolada must be raised to at least 33 m above groud level to clear the first Fresnel zone.
// ...
// #define	EN_PRINTREP_ANTENNARAISE

// if not defined EN_PRINTREP_ANTENNARAISE, the CHK_FRESNEL_CLEAR enable the check fresnel is clear
// #define CHK_FRESNEL_CLEAR

// added 10/07/2016:
// enable to print Antenna Height calculated with haat in report
// #define	EN_PRINTREP_HAAT

//--------------------------------------------------------------------

//-------------------------------------------------------------------------
// MR defines sostituzione costanti
//
// type of propagation models
#define	PROP_MODEL_ITWOM			0			// model ITWOM_Ver3.0
#define	PROP_MODEL_LRICE			1			// model Longley-Rice

// type of polarization
#define	POL_HORIZONTAL				0			// polarization horizontal
#define	POL_VERTICAL				1			// polarization vertical

// type of clima
#define	CLIMA_EQUATORIAL				1
#define	CLIMA_CONTINENTAL_SUBTROPICAL	2
#define	CLIMA_MARITIME_SUBTROPICAL		3
#define	CLIMA_DESERT					4
#define	CLIMA_CONTINENTAL_TEMPERATE		5
#define	CLIMA_MARITIME_TEMPERATE_LAND	6
#define	CLIMA_MARITIME_TEMPERATE_SEA	7

// -----------------------------------
// copy filename
#define COPY_FILENAME(destName, srcName)	{							\
		for (x=0; (srcName)[x]!='.' && (srcName)[x]!=0 && x<250; x++)       \
		(destName)[x]=(srcName)[x];                                     \
	}

#define CleanFileName(name)					\
	for (x=0; name[x]!=0; x++)       \
	{                                       \
		if (name[x]==' ' ||          \
			name[x]==17 ||           \
			name[x]=='\\' ||         \
			name[x]=='*' ||          \
			name[x]=='/')            \
		{                                   \
			name[x]='_';	            \
		}                                   \
	}


// -----------------------------------

// -------------------
// speed of light in air, foot/seconds
// speed of light = 299792458 metres per second
// speed of light in materials = (speed of light) / (refractive index)
//          (refractive index air) = 1.000293
// speed of light air = (speed of light) / (refractive index air) = 299704644.53915002904149084318295
// 1 meter = 0.3048 foot
// speed of light air foot/sec = speed of light air / 0.3048 = 983282954.52477043648783085033775
#define SpeedLightAir_foot_sec	9.8425e8

//-------------------------------------------------------------------------
// Modify Marco Rainone
// - created a zip archive with all the .sdf maps
// - before to analize, the maps are uncompressed
//-------------------------------------------------------------------------

char string[255];
char gpsav=0;
char program_name[10];						// name of program
char program_version[6];					// version of program
char dashes[80];
char PropagationModel;

double max_range=0.0;
double forced_EffRadiatedPower=-1.0;
double fzone_clearance=0.6;
double forced_freq;
double clutter;

// ----------------------------------------
int contour_threshold;

unsigned char got_elevation_pattern;
unsigned char got_azimuth_pattern;
unsigned char metric=0;
unsigned char dbm=0;
unsigned char smooth_contours=0;

site_data tx_site[32];
site_data rx_site;
site_data dbgTxsite = ZeroSiteData;
site_data dbgRxsite = ZeroSiteData;

struct path
{
	double lat[ARRAYSIZE];
	double lon[ARRAYSIZE];
	double elevation[ARRAYSIZE];
	double distance[ARRAYSIZE];
	int length;
} path;

//
typedef struct
{
	double EarthDielectricConst;	// Earth's Dielectric Constant
	double EarthConductivity; 		// Earth Conductivity (Siemens per meter)
	double AtmBendingCons;			// Atmospheric Bending Constant (N-Units)
	double frq_mhz;
	// Fraction of situations and Fraction of Time reflect how the Longley-Rice
	// calculations are to be carried out.
	// For example, the program will calculate the maximum path loss experienced
	// 50% of the time in 50% of the situations.
	double FractionSituations; 		// Fraction of Situations
	double FractionTime;			// Fraction of Time
	double EffRadiatedPower;		// Transmitter Effective Radiated Power in Watts or dBm, https://en.wikipedia.org/wiki/Effective_radiated_power
	int radio_climate;
	int polarization;				// polarization, 0:Horizontal 1:Vertical
	float antenna_pattern[361][1001];
}	data_LR;

data_LR LR;

typedef struct
{
	unsigned char color[32][3];
	int level[32];
	int levels;
} region_data;

region_data region;
#define	SetRegionData(id, lev, col0, col1, col2)	{		\
	region.level[id]=(lev);                                 \
	region.color[id][0]=col0;                               \
	region.color[id][1]=col1;                               \
	region.color[id][2]=col2;                               \
}

double elev[ARRAYSIZE+10];


//=======================================================
// used to calc power margin

// data used to calc margin about power link
typedef struct
{
	double TxPower;					// Tx transmission power
	double TxCableLoss;				// Tx cable loss
	double TxAntGain;				// Tx antenna gain

	double RxAntGain;				// Rx antenna gain
	double RxCableLoss;				// Rx cable loss
	double RxSensitivity;			// Rx sensitivity
} data_pow_link;

data_pow_link dtPowerLink;						// power link data

void ReadDataPowerLink(char *dtFileName)
{
	FILE *fd;
	char string[50];

	fd=fopen(dtFileName,"r");
	if (fd==NULL)
	{
		return;
	}
	fgets(string,49,fd);
	sscanf(string,"%lf",&dtPowerLink.TxPower);				// Tx transmission power
	fgets(string,49,fd);
	sscanf(string,"%lf",&dtPowerLink.TxCableLoss);			// Tx cable loss
	fgets(string,49,fd);
	sscanf(string,"%lf",&dtPowerLink.TxAntGain);			// Tx antenna gain
	fgets(string,49,fd);
	sscanf(string,"%lf",&dtPowerLink.RxAntGain);			// Rx antenna gain
	fgets(string,49,fd);
	sscanf(string,"%lf",&dtPowerLink.RxCableLoss);			// Rx cable loss
	fgets(string,49,fd);
	sscanf(string,"%lf",&dtPowerLink.RxSensitivity);		// Rx sensitivity
	fclose(fd);
}

//=======================================================
// MR: new functions inserted in rfprobe
//
#ifdef DBG_ITWOM
// date: 15:05/2016
FILE *fMr=NULL;
#endif

// ---------------------------------------------
// log utility
// ---------------------------------------------

// get actual time in format yyyy-mm-dd hh:mm:ss
void GetStrTime(char *buffer)
{
	time_t rawtime;
	struct tm *timeinfo;

	// print date & time
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer,21,"%F %T",timeinfo);
}

// general type log
char LogGen[PATH_MAX];				// full path log
FILE *fpLogGen=NULL;				// file pointer general log

// user type log
char LogUsr[PATH_MAX];				// full path log
FILE *fpLogUsr=NULL;				// file pointer user log

// general log utility
void ini_logGen(char *logName, char *mode)
{
	fpLogGen=fopen(logName, mode);
}
void end_logGen(void)
{
	fclose(fpLogGen);
	fpLogGen=NULL;
}

// write formatted string in general log
void logGen(const char * format, ...)
{
	char buffer[32];
	va_list args;

	if(fpLogGen==NULL)
	{
		return;
	}
	// print date & time
	GetStrTime(buffer);
	fprintf(fpLogGen, "%s|",buffer);

	// print the arguments:
	va_start (args, format);
	vfprintf (fpLogGen, format, args);
	va_end (args);
	fflush(fpLogGen);
}

// user log utility
void ini_logUsr(char *logName, char *mode)
{
	fpLogUsr=fopen(logName, mode);
	if(fpLogUsr==NULL)
	return;
	fprintf(fpLogUsr, ">>>>>>>>> ini_logUsr\n");
	fflush(fpLogUsr);
	// printf("ini_logUsr\n");
}
void end_logUsr(void)
{
	if(fpLogUsr==NULL)
	return;
	fprintf(fpLogUsr, ">>>>>>>>> end_logUsr\n");
	fflush(fpLogUsr);

	fclose(fpLogUsr);
	fpLogUsr=NULL;
	// printf("end_logUsr\n");
}

// write formatted string in general log
void logUsr(const char * format, ...)
{
	char buffer[128];
	va_list args;

	if(fpLogUsr==NULL)
	{
		return;
	}

	// print date & time
	GetStrTime(buffer);
	fprintf(fpLogUsr, "%s|",buffer);

	// print the arguments:
	va_start (args, format);
	vfprintf (fpLogUsr, format, args);
	va_end (args);
	fflush(fpLogUsr);
	// printf("\nlogUsr:");
	// vprintf (format, args);			// dbg
}

// ---------------------------------------------
// file path utility
// ---------------------------------------------
//

// filename length for different file systems
#define	LENFNAME_EXT2		255	// in bytes
#define	LENFNAME_EXT3		255	// in bytes
#define	LENFNAME_EXT3COW	255	// in bytes
#define	LENFNAME_EXT4		255	// in bytes
#define	LENFNAME_FAT32		255	// in bytes
#define	LENFNAME_NTFS		255	// in characters

#define MAX_LENFNAME		255

// http://stackoverflow.com/questions/298510/how-to-get-the-current-directory-in-a-c-program
char PathCurrDir[PATH_MAX];			// path current directory

// add 25/06/2016
char PathOutputDir[PATH_MAX];		// path output directory
char PathFileName[PATH_MAX];		// file name with extension

char *GetCurrentDirectory(void)
{
	char *ptr;

	ptr = (getcwd(PathCurrDir, sizeof(PathCurrDir)));

	return(ptr);
}

// this version of InitDirUtility uses GetAbsolutePathProgram().
// This is made to use the program with cron.
// If this program is called inside cron, the function getcwd return
// the home directory of user (in this case root) and not
// the path where is stored the program
//
int InitDirUtility(void)
{
	GetAbsolutePathProgram(PathCurrDir);

	// initialize PathOutputDir, where are stored out files
	strncpy(PathOutputDir, PathCurrDir, PATH_MAX);
	return(1);
}

// transform pathname in full pathname
int FullPathName(char *full_path, char *path)
{
	int len;
	if(full_path == NULL)
	{
		return(0);
	}
	full_path[0]=0;
	if(path == NULL)
	{
	fprintf(stdout, "path[NULL]\n");
	fflush(stdout);
		return(0);
	}
	if(path[0] == 0)
	{
	fprintf(stdout, "if(path[0] == 0)\n");
	fflush(stdout);
		return(0);
	}
	if(path[0]=='/')
	{
		// is an absolute path
	fprintf(stdout, "if(path[0]=='/')\n");
	fflush(stdout);
		strcpy(full_path, path);
	}
	else if(path[0]=='.')
	{
	fprintf(stdout, "if(path[0]=='.')\n");
	fflush(stdout);
		if(path[1]=='/')
		{
	fprintf(stdout, "if(path[1]=='/')\n");
	fflush(stdout);
			// add actual location path
			strcpy(full_path, PathCurrDir);
			len = strlen(full_path);
			strcpy(&full_path[len], &path[1]);
		}
		else
		{
			// path is relative to the actual location path
			sprintf(full_path, "%s/%s", PathCurrDir, path);
		}
	}
	else
	{
	fprintf(stdout, ">>>>>>>>>>>>> FullPathName else :\n");
	fflush(stdout);
		// path is relative to the actual location path
		sprintf(full_path, "%s/%s", PathCurrDir, path);
	}
	fprintf(stdout, "full path[%s]\n", full_path);
	fprintf(stdout, "path[%s]\n", path);
	fflush(stdout);

	return(strlen(full_path));
}

// substitute of the original rfprobe code to form file name:
void SetFileName(char *DestFileName, char *SrcFileName, char *Ext)
{
	int x;

	if(SrcFileName == NULL)
	{
		if(DestFileName!=NULL)
		{
			DestFileName[0]=0;
		}
		return;
	}
	for (x=0; SrcFileName[x]!='.' && SrcFileName[x]!=0 && x<250; x++)
	{
		DestFileName[x]=SrcFileName[x];
	}
	DestFileName[x]=0;
	if(Ext != NULL)
	{
		DestFileName[x]='.';
		DestFileName[x+1]=Ext[0];
		DestFileName[x+2]=Ext[1];
		DestFileName[x+3]=Ext[2];
		DestFileName[x+4]=0;
	}
}

//
// end file path utility
// ---------------------------------------------
//

//---------------------------------------------------
// geo conversion utility
//---------------------------------------------------

double stdLatitude(double lat)
{
	return(lat);
}
double stdLongitude(double lon)
{
	if((lon>0)&&(lon<180))
	{
		// longitude is negative to west greenwitch
		lon = -lon;
	}
	else
	{
		// longitude is positive to east greenwitch
		lon = 360.0 -lon;
	}
	return(lon);
}

//======================================================

//
// string with absolute path of rfprobe file names
char fn_clutter_gp[PATH_MAX];
char fn_curvature_gp[PATH_MAX];
char fn_fresnel_gp[PATH_MAX];
char fn_fresnel_pt_6_gp[PATH_MAX];
char fn_profile_gp[PATH_MAX];
// char fn_profile_clean_gp[PATH_MAX];		// used to create clean profile
char fn_reference_gp[PATH_MAX];
char fn_splat_dcf[PATH_MAX];
char fn_splat_gp[PATH_MAX];

char fn_power_gp[PATH_MAX];					// mr power plot
char fn_power_bck_pos[PATH_MAX];			// mr power plot background for positive margin
char fn_power_bck_neg[PATH_MAX];			// mr power plot background for negative margin
char graph_pow_name[PATH_MAX];
char graph_pow_file[PATH_MAX];

char fn_splat_lcf[PATH_MAX];
char fn_splat_lrp[PATH_MAX];
char fn_splat_scf[PATH_MAX];
// mr:
char fn_splatmaps_zip[PATH_MAX];		// absolute path zip maps
char fn_zippath_srtmmaps[PATH_MAX];		// absolute path of srtm zip maps
char fn_dbidx[PATH_MAX];				// absolute path sqlite db with map index
// added 25/06
char fn_pathreport[PATH_MAX];			// absolute path rfprobe path Report
char fn_pathreport_red[PATH_MAX];		// absolute path rfprobe path Report reduced

//---------------------------
// database with map index
extern sqlite3 *dbidx;

// create the absolute path filenames of rfprobe files
// list of files
// clutter.gp		: The height of ground clutter specified using the -gc switch
//
// Configuration files:
// splat.lrp		: irregular terrain model parameter (lrp) configuration file
// splat.dcf		: DBM  Signal  Level  Color  Definition
// splat.lcf		: path loss color definition file
// splat.scf		: field  strength  color  definition  file
//
// GnuPlot files to generate graph
// splat.gp			: gnuplot script to generate elevation graph
// fresnel.gp		: used in splat.gp. axes x1y1. First Fresnel Zone
// curvature.gp		: used in splat.gp. axes x1y2. Earth's Curvature Contour
// fresnel_pt_6.gp	: used in splat.gp. % of First Fresnel Zone
// profile.gp		: used in splat.gp. Point-to-Point Profile
// reference.gp		: used in splat.gp. Optical Line of Sight Path
//
// splatmaps.zip	:

void SetFilesWithAbsPath(void)
{
	// full path user log
	snprintf(LogUsr				, PATH_MAX	, "%s/mrlog.txt",				PathCurrDir);

	//
	// full path rfprobe files
	snprintf(fn_clutter_gp		, PATH_MAX	, "%s/clutter.gp",						PathCurrDir);
	snprintf(fn_curvature_gp	, PATH_MAX	, "%s/curvature.gp",					PathCurrDir);
	snprintf(fn_fresnel_gp		, PATH_MAX	, "%s/fresnel.gp",						PathCurrDir);
	snprintf(fn_fresnel_pt_6_gp	, PATH_MAX	, "%s/fresnel_pt_6.gp",					PathCurrDir);
	snprintf(fn_profile_gp		, PATH_MAX	, "%s/profile.gp",						PathCurrDir);

	snprintf(fn_reference_gp	, PATH_MAX	, "%s/reference.gp",					PathCurrDir);
	snprintf(fn_splat_dcf		, PATH_MAX	, "%s/splat.dcf",						PathCurrDir);
	snprintf(fn_splat_gp		, PATH_MAX	, "%s/splat.gp",						PathCurrDir);
	snprintf(fn_power_gp		, PATH_MAX	, "%s/power.gp",						PathCurrDir);
	snprintf(fn_power_bck_pos	, PATH_MAX	, "%s/grppowbackground_posmarg.png",	PathCurrDir);
	snprintf(fn_power_bck_neg	, PATH_MAX	, "%s/grppowbackground_negmarg.png",	PathCurrDir);
	snprintf(fn_splat_lcf		, PATH_MAX	, "%s/splat.lcf",						PathCurrDir);
	snprintf(fn_splat_lrp		, PATH_MAX	, "%s/splat.lrp",						PathCurrDir);
	snprintf(fn_splat_scf		, PATH_MAX	, "%s/splat.scf",						PathCurrDir);
	snprintf(fn_splatmaps_zip	, PATH_MAX	, "%s/zipmaps/splatmaps.zip",			PathCurrDir);

	snprintf(fn_zippath_srtmmaps	, PATH_MAX	, "%s/dem3",	PathCurrDir);

	// full path sqlite3 db maps index
	snprintf(fn_dbidx			, PATH_MAX	, "%s/dbmaps.sqlite",	PathCurrDir);
}

// substitute the instruction to generate graph with absolute path
// system("gnuplot splat.gp")
int GenerateGraphGnuplot(void)
{
	char cmd[PATH_MAX];
	sprintf(cmd, "gnuplot %s", fn_splat_gp);
	return(system(cmd));
}

//---------------------------------------------------
// MR MACRO VARIE

// calculate dB
#define CALC_DB(value)		( 20.0*log10( (value) ) )


//=======================================================
// per progress indicator
#define UpdateProgressIndicator()	{				\
	idxSymbol++;                                    \
	if (idxSymbol==z)                               \
	{                                               \
		fprintf(stdout, "%c",symbol[x]);            \
		fflush(stdout);                             \
		idxSymbol=0;                                \
		if (x==3)                                   \
			x=0;                                    \
		else                                        \
			x++;                                    \
	}                                               \
}

// continue original code...
//=======================================================


void point_to_point(double elev[], double tht_m, double rht_m,
					double eps_dielect, double EarthConductivity, double AtmBendingCons,
					double frq_mhz, int radio_climate, int pol, double conf,
					double rel, double &dbloss, char *strmode, int &errnum);

void point_to_point_ITM(double elev[], double tht_m, double rht_m,
						double eps_dielect, double EarthConductivity, double AtmBendingCons,
						double frq_mhz, int radio_climate, int pol, double conf,
						double rel, double &dbloss, char *strmode, int &errnum);

double ITWOMVersion();


double ElevationAngle(site_data source, site_data destination)
{
	// This function returns the angle of elevation (in degrees)
	// of the destination as seen from the source location.
	// A positive result represents an angle of elevation (uptilt),
	// while a negative result represents an angle of depression
	// (downtilt), as referenced to a normal to the center of
	// the earth.

	register double a, b, dx;

	a=GetElevation(destination)+destination.alt+earthRadiusFoot;
	b=GetElevation(source)+source.alt+earthRadiusFoot;

	dx=mi2Foot( Distance(source,destination) );

	// Apply the Law of Cosines
	return ((180.0*(acos(((b*b)+(dx*dx)-(a*a))/(2.0*b*dx)))/PI)-90.0);
}

int interpolate(int y0, int y1, int x0, int x1, int n)
{
	// Perform linear interpolation between quantized contour
	// levels displayed in field strength and path loss maps.
	// If signal level x0 corresponds to color level y0, signal
	// level x1 corresponds to color level y1, and signal level
	// n falls somewhere between x0 and x1, determine what
	// color value n corresponds to between y0 and y1.

	int result=0;
	double delta_x, delta_y;

	if (n<=x0)
	return y0;

	if (n>=x1)
	return y1;

	if (y0==y1)
	return y0;

	if (x0==x1)
	return y0;

	delta_y=(double)(y0-y1);
	delta_x=(double)(x0-x1);

	result=y0+(int)ceil((delta_y/delta_x)*(n-x0));

	return result;
}


// print a (lat,lon) position in degrees
void fprint_deg(FILE *fp, double lat, double lon)
{
	fprintf(fp,"(%+.4f,%+.4f)", stdLatitude(lat), stdLongitude(lon) );
//	fprintf(fp,"(%.4f,%.4f)", stdLatitude(lat), stdLongitude(lon) );
}

// print a (lat,lon) position converted in dms
void fprint_dms(FILE *fp, double lat, double lon)
{
	fprintf(fp, "(%s", dec2dms(stdLatitude(lat)));
	fprintf(fp, "/");
	fprintf(fp, "%s)", dec2dms(stdLongitude(lon)));
}

void ReadPath(site_data source, site_data destination)
{
	// This function generates a sequence of latitude and
	// longitude positions between source and destination
	// locations along a great circle path, and stores
	// elevation and distance information for points
	// along that path in the "path" structure.

	int	c;
	double azimuth;
	double distance;
	double lat1;
	double lon1;
	double beta;
	double den;
	double num;
	double lat2;
	double lon2;
	double total_distance;
	double MilesPerPoint;
	double NumPointsRadian;

	site_data tempsite=ZeroSiteData;

	// convert degree in radians
	lat1=RADIANS(source.lat);
	lon1=RADIANS(source.lon);
	lat2=RADIANS(destination.lat);
	lon2=RADIANS(destination.lon);

	// NumPointsRadian = NumPointsDegree * 180 / pigreco
	if(NumPointsDegree==3600)
	{
		NumPointsRadian=206265.0;
	}
	else
	{
		// for (NumPointsDegree==1200)
		NumPointsRadian=68755.0;
	}

	azimuth=RADIANS(AzimuthSites(source,destination));
	total_distance=Distance(source,destination);		// distance in miles from source to dest

	if (total_distance>(30.0/ppd))		// > 0.5 point distance
	{
		double nPointsLon;
		double nPointsLat;
		double nPointsPath;

		nPointsLon=NumPointsRadian*acos(cos(lon1-lon2));
		nPointsLat=NumPointsRadian*acos(cos(lat1-lat2));
		nPointsPath=sqrt((nPointsLon*nPointsLon)+(nPointsLat*nPointsLat));		// Total number of samples
		MilesPerPoint=total_distance/nPointsPath;	// Miles per point
	}
	else
	{
		// total_distance <= 0.5 point distance
		total_distance=0.0;
		c=0;

		// nPointsLon=0.0;
		// nPointsLat=0.0;
		// nPointsPath=0.0;
		MilesPerPoint=0.0;

		lat1=DEGREES(lat1);
		lon1=DEGREES(lon1);

		path.lat[c]=lat1;
		path.lon[c]=lon1;
		path.elevation[c]=GetElevation(source);
		path.distance[c]=0.0;
	}

	for (distance=0.0, c=0;
			(total_distance!=0.0 && distance<=total_distance && c<ARRAYSIZE);
			c++, distance=MilesPerPoint*(double)c)
	{
		beta = DISTANCE_2_ARC(distance);
		lat2=asin(sin(lat1)*cos(beta)+cos(azimuth)*sin(beta)*cos(lat1));
		num=cos(beta)-(sin(lat1)*sin(lat2));
		den=cos(lat1)*cos(lat2);

		if (azimuth==0.0 && (beta>HALFPI-lat1))
		{
			lon2=lon1+PI;
		}
		else if (azimuth==HALFPI && (beta>HALFPI+lat1))
		{
			lon2=lon1+PI;
		}
		else if (fabs(num/den)>1.0)
		{
			lon2=lon1;
		}
		else
		{
			if ((PI-azimuth)>=0.0)
			{
				lon2=lon1-ArcCos(num,den);
			}
			else
			{
				lon2=lon1+ArcCos(num,den);
			}
		}

		while (lon2<0.0)
		{
			lon2+=TWOPI;
		}
		while (lon2>TWOPI)
		{
			lon2-=TWOPI;
		}

		lat2=DEGREES(lat2);
		lon2=DEGREES(lon2);

		path.lat[c]=lat2;
		path.lon[c]=lon2;
		tempsite.lat=lat2;
		tempsite.lon=lon2;
		path.elevation[c]=GetElevation(tempsite);
		path.distance[c]=distance;
	}

	// Make sure exact destination point is recorded at path.length-1
	if (c<ARRAYSIZE)
	{
		path.lat[c]=destination.lat;
		path.lon[c]=destination.lon;
		path.elevation[c]=GetElevation(destination);
		path.distance[c]=total_distance;
		c++;
	}
	if (c<ARRAYSIZE)
	{
		path.length=c;
	}
	else
	{
		path.length=ARRAYSIZE-1;
	}
}

double ElevationAngleUntilObstruction(site_data source, site_data destination, double earth_radius)
{
	// This function returns the angle of elevation (in degrees)
	// of the destination as seen from the source location, UNLESS
	// the path between the sites is obstructed, in which case, the
	// elevation angle to the first obstruction is returned instead.

	int	x;
	char FoundObstruction=0;
	double source_alt;
	double destination_alt;
	double cos_xmtr_angle;
	double cos_test_angle;
	double test_alt;
	double elevation;
	double distance;
	double source_alt2;
	double first_obstruction_angle=0.0;

	struct	path temp;

	temp=path;

	ReadPath(source,destination);

	distance=mi2Foot( Distance(source,destination) );

	source_alt=earth_radius+source.alt+GetElevation(source);
	destination_alt=earth_radius+destination.alt+GetElevation(destination);
	source_alt2=source_alt*source_alt;

	// Calculate the cosine of the elevation angle of the
	// destination (receiver) as seen by the source (transmitter).

	cos_xmtr_angle=((source_alt2)+(distance*distance)-(destination_alt*destination_alt))/(2.0*source_alt*distance);

	// Test all points in between source and destination locations to
	// see if the angle to a topographic feature generates a higher
	// elevation angle than that produced by the destination.  Begin
	// at the source since we're interested in identifying the FIRST
	// obstruction along the path between source and destination.

	for (x=2, FoundObstruction=0; x<path.length && FoundObstruction==0; x++)
	{
		distance=mi2Foot( path.distance[x] );

		test_alt=earthRadiusFoot+(path.elevation[x]==0.0?path.elevation[x]:path.elevation[x]+clutter);

		cos_test_angle=((source_alt2)+(distance*distance)-(test_alt*test_alt))/(2.0*source_alt*distance);

		// Compare these two angles to determine if
		// an obstruction exists.  Since we're comparing
		// the cosines of these angles rather than
		// the angles themselves, the sense of the
		// following "if" statement is reversed from
		// what it would be if the angles themselves
		// were compared.

		if (cos_xmtr_angle>=cos_test_angle)
		{
			FoundObstruction=1;
			first_obstruction_angle=(DEGREES(acos(cos_test_angle)))-90.0;
		}
	}

	if (FoundObstruction)
	{
		elevation=first_obstruction_angle;
	}
	else
	{
		elevation=(DEGREES(acos(cos_xmtr_angle)))-90.0;
	}

	path=temp;

	return elevation;
}

double AverageTerrain(site_data source, double azimuthx, double start_distance, double end_distance)
{
	// This function returns the average terrain calculated in
	// the direction of "azimuth" (degrees) between "start_distance"
	// and "end_distance" (miles) from the source location.  If
	// the terrain is all water (non-critical error), NO_LAND_HEIGHT is
	// returned.  If not enough SDF data has been loaded into
	// memory to complete the survey (critical error), then
	// -9999.0 is returned.

	int	c, samples, endpoint;
	double beta;
	double lat1;
	double lon1;
	double lat2;
	double lon2;
	double num;
	double den;
	double azimuth;
	double terrain=0.0;

	site_data destination=ZeroSiteData;

	lat1=RADIANS(source.lat);
	lon1=RADIANS(source.lon);

	// Generate a path of elevations between the source
	// location and the remote location provided.

	beta=DISTANCE_2_ARC(end_distance);

	azimuth=RADIANS(azimuthx);

	lat2=asin(sin(lat1)*cos(beta)+cos(azimuth)*sin(beta)*cos(lat1));

	num=cos(beta)-(sin(lat1)*sin(lat2));
	den=cos(lat1)*cos(lat2);
	if (azimuth==0.0 && (beta>HALFPI-lat1))
	{
		lon2=lon1+PI;
	}
	else if (azimuth==HALFPI && (beta>HALFPI+lat1))
	{
		lon2=lon1+PI;
	}
	else if (fabs(num/den)>1.0)
	{
		lon2=lon1;
	}
	else
	{
		if ((PI-azimuth)>=0.0)
		lon2=lon1-ArcCos(num,den);
		else
		lon2=lon1+ArcCos(num,den);
	}

	while (lon2<0.0)
	{
		lon2+=TWOPI;
	}

	while (lon2>TWOPI)
	{
		lon2-=TWOPI;
	}

	lat2=DEGREES(lat2);
	lon2=DEGREES(lon2);

	destination.lat=lat2;
	destination.lon=lon2;

	// If SDF data is missing for the endpoint of
	// the radial, then the average terrain cannot
	// be accurately calculated.  Return -9999.0

	if (GetElevation(destination)<NO_LAND_HEIGHT)
	{
		return (-9999.0);
	}
	else
	{
		ReadPath(source,destination);

		endpoint=path.length;

		// Shrink the length of the radial if the
		// outermost portion is not over U.S. land.

		for (c=endpoint-1; c>=0 && path.elevation[c]==0.0; c--);

		endpoint=c+1;

		for (c=0, samples=0; c<endpoint; c++)
		{
			if (path.distance[c]>=start_distance)
			{
				terrain+=(path.elevation[c]==0.0?path.elevation[c]:path.elevation[c]+clutter);
				samples++;
			}
		}

		if (samples==0)
		{
			terrain=NO_LAND_HEIGHT;  // No land
		}
		else
		{
			terrain=(terrain/(double)samples);
		}

		return terrain;
	}
}

// Height Above Average Terrain Calculation
//
double haat(site_data antenna)
{
	// This function returns the antenna's Height Above Average
	// Terrain (HAAT) based on FCC Part 73.313(d).  If a critical
	// error occurs, such as a lack of SDF data to complete the
	// survey, NO_LAND_HEIGHT is returned.


	int	azi, c;
	char error=0;
	double terrain;
	double avg_terrain;
	double haat;
	double sum=0.0;

	// Calculate the average terrain between 2 and 10 miles
	// from the antenna site at azimuths of 0, 45, 90, 135,
	// 180, 225, 270, and 315 degrees.

	for (c=0, azi=0; azi<=315 && error==0; azi+=45)
	{
		terrain=AverageTerrain(antenna, (double)azi, 2.0, 10.0);

		if (terrain<-9998.0)  // SDF data is missing
		error=1;

		if(terrain>NO_LAND_HEIGHT)  // It's land, not water
		{
			sum+=terrain;  // Sum of averages
			c++;
		}

		if(terrain < 0.0)
		{
			pmsg(stdout, ( minus_haat_terrain ), terrain, c);
			fflush(stdout);
		}
		else
		{
			pmsg(stdout, ( plus_haat_terrain_sum ), terrain, c, sum);
			fflush(stdout);
		}

	}

	pmsg(stdout, ( haat_sum_c ), sum, c);
	fflush(stdout);

	if (error)
	{
		return NO_LAND_HEIGHT;
	}
	else
	{
		if(c==0)
		{
			logUsr("haat divisione per 0\n");
			return NO_LAND_HEIGHT;
		}
		avg_terrain=(sum/(double)c);

		haat=(antenna.alt+GetElevation(antenna))-avg_terrain;

		pmsg(stdout, ( haat_avg_terrain_alt_elevation_haat ),
			avg_terrain, antenna.alt, GetElevation(antenna), haat);
		fflush(stdout);

		return haat;
	}
}

void PlaceMarker(site_data location)
{
	// This function places text and marker data in the mask array
	// for illustration on topographic maps generated by rfprobe.
	// By default, rfprobe centers text information BELOW the marker,
	// but may move it above, to the left, or to the right of the
	// marker depending on how much room is available on the map,
	// or depending on whether the area is already occupied by
	// another marker or label.  If no room or clear space is
	// available on the map to place the marker and its associated
	// text, then the marker and text are not written to the map.

	int	a, b, c, byte;
	char ok2print;
	char occupied;
	double x;
	double y;
	double lat;
	double lon;
	double textx=0.0;
	double texty=0.0;
	double xmin;
	double xmax;
	double ymin;
	double ymax;
	double p1;
	double p3;
	double p6;
	double p8;
	double p12;
	double p16;
	double p24;
	double label_length;

	xmin=(double)min_lat;
	xmax=(double)max_lat;
	ymin=(double)min_lon;
	ymax=(double)max_lon;
	lat=location.lat;
	lon=location.lon;

	if ( lat<xmax && lat>=xmin &&
			(DeltaLongitude(lon,ymax)<=0.0) &&
			(DeltaLongitude(lon,ymin)>=DegreesPerPoint) )
	{
		p1=1.0/ppd;
		p3=3.0/ppd;
		p6=6.0/ppd;
		p8=8.0/ppd;
		p12=12.0/ppd;
		p16=16.0/ppd;
		p24=24.0/ppd;

		ok2print=0;
		occupied=0;

		// Is Marker Position Clear Of Text Or Other Markers?

		for (a=0, x=lat-p3; (x<=xmax && x>=xmin && a<7); x+=p1, a++)
		for (b=0, y=lon-p3; (DeltaLongitude(y,ymax)<=0.0) && (DeltaLongitude(y,ymin)>=DegreesPerPoint) && b<7; y+=p1, b++)
		occupied|=(GetMask(x,y)&2);

		if (occupied==0)
		{
			// Determine Where Text Can Be Positioned

			// label_length=length in points.
			// Each character is 8 n. points wide.

			label_length=p1*(double)(strlen(location.name)<<3);

			if ((DeltaLongitude(lon+label_length,ymax)<=0.0) && (DeltaLongitude(lon-label_length,ymin)>=DegreesPerPoint))
			{
				// Default: Centered Text

				texty=lon+label_length/2.0;

				if ((lat-p8)>=p16)
				{
					// Position Text Below The Marker

					textx=lat-p8;

					x=textx;
					y=texty;

					// Is This Position Clear Of Text Or Other Markers?

					for (a=0, occupied=0; a<16; a++)
					{
						for (b=0; b<(int)strlen(location.name); b++)
						for (c=0; c<8; c++, y-=p1)
						occupied|=(GetMask(x,y)&2);
						x-=p1;
						y=texty;
					}

					x=textx;
					y=texty;

					if (occupied==0)
					ok2print=1;
				}

				else
				{
					// Position Text Above The Marker

					textx=lat+p24;

					x=textx;
					y=texty;

					// Is This Position Clear Of Text Or Other Markers?

					for (a=0, occupied=0; a<16; a++)
					{
						for (b=0; b<(int)strlen(location.name); b++)
						for (c=0; c<8; c++, y-=p1)
						occupied|=(GetMask(x,y)&2);
						x-=p1;
						y=texty;
					}

					x=textx;
					y=texty;

					if (occupied==0)
					ok2print=1;
				}
			}

			if (ok2print==0)
			{
				if (DeltaLongitude(lon-label_length,ymin)>=DegreesPerPoint)
				{
					// Position Text To The	Right Of The Marker

					textx=lat+p6;
					texty=lon-p12;

					x=textx;
					y=texty;

					// Is This Position Clear Of Text Or Other Markers?

					for (a=0, occupied=0; a<16; a++)
					{
						for (b=0; b<(int)strlen(location.name); b++)
						for (c=0; c<8; c++, y-=p1)
						occupied|=(GetMask(x,y)&2);
						x-=p1;
						y=texty;
					}

					x=textx;
					y=texty;

					if (occupied==0)
					ok2print=1;
				}

				else
				{
					// Position Text To The	Left Of The Marker

					textx=lat+p6;
					texty=lon+p8+(label_length);

					x=textx;
					y=texty;

					// Is This Position Clear Of Text Or Other Markers?

					for (a=0, occupied=0; a<16; a++)
					{
						for (b=0; b<(int)strlen(location.name); b++)
						for (c=0; c<8; c++, y-=p1)
						occupied|=(GetMask(x,y)&2);
						x-=p1;
						y=texty;
					}

					x=textx;
					y=texty;

					if (occupied==0)
					ok2print=1;
				}
			}

			// textx and texty contain the latitude and longitude
			// coordinates that describe the placement of the text
			// on the map.

			if (ok2print)
			{
				// Draw Text

				x=textx;
				y=texty;

				for (a=0; a<16; a++)
				{
					for (b=0; b<(int)strlen(location.name); b++)
					{
						byte=fontdata[16*(location.name[b])+a];

						for (c=128; c>0; c=c>>1, y-=p1)
						if (byte&c)
						OrMask(x,y,2);
					}

					x-=p1;
					y=texty;
				}

				// Draw Square Marker Centered On Location Specified

				for (a=0, x=lat-p3; (x<=xmax && x>=xmin && a<7); x+=p1, a++)
				for (b=0, y=lon-p3; (DeltaLongitude(y,ymax)<=0.0) && (DeltaLongitude(y,ymin)>=DegreesPerPoint) && b<7; y+=p1, b++)
				OrMask(x,y,2);
			}
		}
	}
}

double ReadBearing(char *input)
{
	// This function takes numeric input in the form of a character
	// string, and returns an equivalent bearing in degrees as a
	// decimal number (double).  The input may either be expressed
	// in decimal format (40.139722) or degree, minute, second
	// format (40 08 23).  This function also safely handles
	// extra spaces found either leading, trailing, or
	// embedded within the numbers expressed in the
	// input string.  Decimal seconds are permitted.

	double seconds;
	double bearing=0.0;
	char string[20];
	int	a;
	int b;
	int length;
	int degrees;
	int minutes;

	// Copy "input" to "string", and ignore any extra
	// spaces that might be present in the process.

	string[0]=0;
	length=strlen(input);

	for (a=0, b=0; a<length && a<18; a++)
	{
		if ((input[a]!=' ' && input[a]!='\n') || (input[a]==' ' && input[a+1]!=' ' && input[a+1]!='\n' && b!=0))
		{
			string[b]=input[a];
			b++;
		}
	}

	string[b]=0;

	// Count number of spaces in the clean string.

	length=strlen(string);

	for (a=0, b=0; a<length; a++)
	if (string[a]==' ')
	b++;

	if (b==0)  // Decimal Format (40.139722)
	sscanf(string,"%lf",&bearing);

	if (b==2)  // Degree, Minute, Second Format (40 08 23.xx)
	{
		sscanf(string,"%d %d %lf",&degrees, &minutes, &seconds);

		bearing=fabs((double)degrees);
		bearing+=fabs(((double)minutes)/60.0);
		bearing+=fabs(seconds/3600.0);

		if ((degrees<0) || (minutes<0) || (seconds<0.0))
		bearing=-bearing;
	}

	// Anything else returns a 0.0

	if (bearing>360.0 || bearing<-360.0)
	bearing=0.0;

	return bearing;
}

site_data LoadQTH(char *filename)
{
	// This function reads rfprobe .qth (site location) files.
	// The latitude and longitude may be expressed either in
	// decimal degrees, or in degree, minute, second format.
	// Antenna height is assumed to be expressed in feet above
	// ground level (AGL), unless followed by the letter 'M',
	// or 'm', or by the word "meters" or "Meters", in which
	// case meters is assumed, and is handled accordingly.

	int	x;
	char	string[50], qthfile[255];
	site_data tempsite=ZeroSiteData;
	FILE	*fd=NULL;

	/***
	original:
	x=strlen(filename);
	strncpy(qthfile, filename, 254);
	if (qthfile[x-3]!='q' || qthfile[x-2]!='t' || qthfile[x-1]!='h')
	{
		if (x>249)
			qthfile[249]=0;

		strncat(qthfile,".qth\0",5);
	}
	***/
	// inserted mr 15/05/2016. Disabled using fopenFullPath
	logUsr("LoadQTH(filename)=[%s]\n", filename);

	snprintf(qthfile, 254, "%s", filename);

	tempsite.lat=91.0;
	tempsite.lon=361.0;
	tempsite.alt=0.0;
	tempsite.name[0]=0;
	tempsite.filename[0]=0;

	fd=fopenFullPath(qthfile,"r");

	if (fd!=NULL)
	{
		// Site Name
		fgets(string,49,fd);

		// Strip <CR> and/or <LF> from end of site name
		for (x=0; string[x]!='\r' && string[x]!='\n' && string[x]!=0; tempsite.name[x]=string[x], x++);

		tempsite.name[x]=0;

		// Site Latitude
		fgets(string,49,fd);
		tempsite.lat=ReadBearing(string);

		// Site Longitude
		fgets(string,49,fd);
		tempsite.lon=ReadBearing(string);

		NormalLongitude(tempsite.lon);

		// Antenna Height
		fgets(string,49,fd);
		fclose(fd);

		// Remove <CR> and/or <LF> from antenna height string
		for (x=0; string[x]!='\r' && string[x]!='\n' && string[x]!=0; x++);

		string[x]=0;

		// Antenna height may either be in feet or meters.
		// If the letter 'M' or 'm' is discovered in
		// the string, then this is an indication that
		// the value given is expressed in meters, and
		// must be converted to feet before exiting.

		for (x=0; string[x]!='M' && string[x]!='m' && string[x]!=0 && x<48; x++);

		if (string[x]=='M' || string[x]=='m')
		{
			string[x]=0;
			sscanf(string,"%f",&tempsite.alt);
			tempsite.alt*=3.28084;
		}

		else
		{
			string[x]=0;
			sscanf(string,"%f",&tempsite.alt);
		}

		for (x=0; x<254 && qthfile[x]!=0; x++)
		tempsite.filename[x]=qthfile[x];

		tempsite.filename[x]=0;
	}

	return tempsite;
}

void LoadPAT(char *filename)
{
	// This function reads and processes antenna pattern (.az and .el) files that correspond
	// in name to previously loaded rfprobe .lrp files.

	int	a;
	int b;
	int w;
	int x;
	int y;
	int z;
	int last_index;
	int next_index;
	int span;

	char string[255];
	char azfile[255];
	char elfile[255];
	char *pointer=NULL;

	float az;
	float xx;
	float elevation;
	float amplitude;
	float rotation;
	float valid1;
	float valid2;
	float delta;
	float azimuth[361];
	float azimuth_pattern[361];
	float el_pattern[10001];
	float elevation_pattern[361][1001];
	float slant_angle[361];
	float tilt;
	float mechanical_tilt=0.0;
	float tilt_azimuth;
	float tilt_increment;
	float sum;

	FILE	*fd=NULL;
	unsigned char read_count[10001];

	SetFileName(azfile, filename, (char *)"az");
	SetFileName(elfile, filename, (char *)"el");

	rotation=0.0;

	got_azimuth_pattern=0;
	got_elevation_pattern=0;

	// Load .az antenna pattern file

	fd=fopenFullPath(azfile,"r");

	if (fd!=NULL)
	{
		// Clear azimuth pattern array

		for (x=0; x<=360; x++)
		{
			azimuth[x]=0.0;
			read_count[x]=0;
		}


		// Read azimuth pattern rotation
		// in degrees measured clockwise
		// from true North.

		fgets(string,254,fd);
		pointer=strchr(string,';');

		if (pointer!=NULL)
		*pointer=0;

		sscanf(string,"%f",&rotation);


		// Read azimuth (degrees) and corresponding
		// normalized field radiation pattern amplitude
		// (0.0 to 1.0) until EOF is reached.

		fgets(string,254,fd);
		pointer=strchr(string,';');

		if (pointer!=NULL)
		*pointer=0;

		sscanf(string,"%f %f",&az, &amplitude);

		do
		{
			x=(int)rintf(az);

			if (x>=0 && x<=360 && fd!=NULL)
			{
				azimuth[x]+=amplitude;
				read_count[x]++;
			}

			fgets(string,254,fd);
			pointer=strchr(string,';');

			if (pointer!=NULL)
			*pointer=0;

			sscanf(string,"%f %f",&az, &amplitude);

		} while (feof(fd)==0);

		fclose(fd);


		// Handle 0=360 degree ambiguity

		if ((read_count[0]==0) && (read_count[360]!=0))
		{
			read_count[0]=read_count[360];
			azimuth[0]=azimuth[360];
		}

		if ((read_count[0]!=0) && (read_count[360]==0))
		{
			read_count[360]=read_count[0];
			azimuth[360]=azimuth[0];
		}

		// Average pattern values in case more than	one was read for each degree of azimuth.

		for (x=0; x<=360; x++)
		{
			if (read_count[x]>1)
				azimuth[x] = azimuth[x] / (float)read_count[x];
		}

		// Interpolate missing azimuths to completely fill the array

		last_index=-1;
		next_index=-1;

		for (x=0; x<=360; x++)
		{
			if (read_count[x]!=0)
			{
				if (last_index==-1)
				last_index=x;
				else
				next_index=x;
			}

			if (last_index!=-1 && next_index!=-1)
			{
				valid1=azimuth[last_index];
				valid2=azimuth[next_index];

				span=next_index-last_index;
				delta=(valid2-valid1)/(float)span;

				for (y=last_index+1; y<next_index; y++)
				azimuth[y]=azimuth[y-1]+delta;

				last_index=y;
				next_index=-1;
			}
		}

		// Perform azimuth pattern rotation
		// and load azimuth_pattern[361] with
		// azimuth pattern data in its final form.

		for (x=0; x<360; x++)
		{
			y=x+(int)rintf(rotation);

			if (y>=360)
			y-=360;

			azimuth_pattern[y]=azimuth[x];
		}

		azimuth_pattern[360]=azimuth_pattern[0];

		got_azimuth_pattern=255;
	}

	// Read and process .el file

	fd=fopenFullPath(elfile,"r");

	if (fd!=NULL)
	{
		for (x=0; x<=10000; x++)
		{
			el_pattern[x]=0.0;
			read_count[x]=0;
		}

		// Read mechanical tilt (degrees) and
		// tilt azimuth in degrees measured
		// clockwise from true North.

		fgets(string,254,fd);
		pointer=strchr(string,';');

		if (pointer!=NULL)
		*pointer=0;

		sscanf(string,"%f %f",&mechanical_tilt, &tilt_azimuth);

		// Read elevation (degrees) and corresponding
		// normalized field radiation pattern amplitude
		// (0.0 to 1.0) until EOF is reached.

		fgets(string,254,fd);
		pointer=strchr(string,';');

		if (pointer!=NULL)
		*pointer=0;

		sscanf(string,"%f %f", &elevation, &amplitude);

		while (feof(fd)==0)
		{
			// Read in normalized radiated field values
			// for every 0.01 degrees of elevation between
			// -10.0 and +90.0 degrees

			x=(int)rintf(100.0*(elevation+10.0));

			if (x>=0 && x<=10000)
			{
				el_pattern[x]+=amplitude;
				read_count[x]++;
			}

			fgets(string,254,fd);
			pointer=strchr(string,';');

			if (pointer!=NULL)
			{
				*pointer=0;
			}
			sscanf(string,"%f %f", &elevation, &amplitude);
		}

		fclose(fd);

		// Average the field values in case more than one was read for each 0.01 degrees of elevation.

		for (x=0; x<=10000; x++)
		{
			if (read_count[x]>1)
			el_pattern[x]/=(float)read_count[x];
		}

		// Interpolate between missing elevations (if any)
		// to completely fill the array and provide
		// radiated field values for every 0.01 degrees of
		// elevation.

		last_index=-1;
		next_index=-1;

		for (x=0; x<=10000; x++)
		{
			if (read_count[x]!=0)
			{
				if (last_index==-1)
				{
					last_index=x;
				}
				else
				{
					next_index=x;
				}
			}

			if (last_index!=-1 && next_index!=-1)
			{
				valid1=el_pattern[last_index];
				valid2=el_pattern[next_index];

				span=next_index-last_index;
				delta=(valid2-valid1)/(float)span;

				for (y=last_index+1; y<next_index; y++)
				el_pattern[y]=el_pattern[y-1]+delta;

				last_index=y;
				next_index=-1;
			}
		}

		// Fill slant_angle[] array with offset angles based
		// on the antenna's mechanical beam tilt (if any)
		// and tilt direction (azimuth).

		if (mechanical_tilt==0.0)
		{
			for (x=0; x<=360; x++)
			{
				slant_angle[x]=0.0;
			}
		}

		else
		{
			tilt_increment=mechanical_tilt/90.0;

			for (x=0; x<=360; x++)
			{
				xx=(float)x;
				y=(int)rintf(tilt_azimuth+xx);

				/**
				while (y<0)
				y+=360;
				while (y>=360)
				y-=360;
				**/
				Degrees2FirstRound(y);

				if (x<=180)
				slant_angle[y]=-(tilt_increment*(90.0-xx));

				if (x>180)
				slant_angle[y]=-(tilt_increment*(xx-270.0));
			}
		}

		slant_angle[360]=slant_angle[0];   // 360 degree wrap-around

		for (w=0; w<=360; w++)
		{
			tilt=slant_angle[w];

			// Convert tilt angle to an array index offset

			y=(int)rintf(100.0*tilt);

			// Copy shifted el_pattern[10001] field
			// values into elevation_pattern[361][1001]
			// at the corresponding azimuth, downsampling
			// (averaging) along the way in chunks of 10.

			for (x=y, z=0; z<=1000; x+=10, z++)
			{
				for (sum=0.0, a=0; a<10; a++)
				{
					b=a+x;

					if (b>=0 && b<=10000)
					sum+=el_pattern[b];
					if (b<0)
					sum+=el_pattern[0];
					if (b>10000)
					sum+=el_pattern[10000];
				}

				elevation_pattern[w][z]=sum/10.0;
			}
		}

		got_elevation_pattern=255;
	}

	for (x=0; x<=360; x++)
	{
		for (y=0; y<=1000; y++)
		{
			if (got_elevation_pattern)
			elevation=elevation_pattern[x][y];
			else
			elevation=1.0;

			if (got_azimuth_pattern)
			az=azimuth_pattern[x];
			else
			az=1.0;

			LR.antenna_pattern[x][y]=az*elevation;
		}
	}
}

//-------------------------------------------------------
// UnzipMap:
// function introduced from MR
// This function extract map from splatmaps.zip and
// save it
//-------------------------------------------------------
int UnzipMap(char * name)
{
	char command[1024];

	// flags used to unzip
	// -o: overwrite file
	// -d: output directory

	// -------------------------
	// log tst
	// logUsr("UnzipMap path=[%s]\n", sdf_path);
	// logUsr("UnzipMap name=[%s]\n", name);

	// -------------------------

	sprintf( command,
	"unzip -o %s %s.sdf -d %s",
	fn_splatmaps_zip,
	name,
	sdf_path
	);

	// logUsr("UnzipMap cmd=[%s]\n", command);

	system(command);

	return(0);
}

// clear all data in array
void ClearArray(void *ptrArray, size_t n_bytes)
{
	memset ( ptrArray, 0x00, n_bytes );
}

void LoadCities(char *filename)
{
	// This function reads rfprobe city/site files, and plots
	// the locations and names of the cities and site locations
	// read on topographic maps generated by rfprobe

	int	x, y, z;
	char	input[80], str[3][80];
	site_data city_site=ZeroSiteData;
	FILE	*fd=NULL;

	fd=fopenFullPath(filename,"r");

	if (fd!=NULL)
	{
		fgets(input,78,fd);

		pmsg( stdout, (reading_file), filename);
		fflush(stdout);

		while (fd!=NULL && feof(fd)==0)
		{
			// Parse line for name, latitude, and longitude

			for (x=0, y=0, z=0; x<78 && input[x]!=0 && z<3; x++)
			{
				if (input[x]!=',' && y<78)
				{
					str[z][y]=input[x];
					y++;
				}

				else
				{
					str[z][y]=0;
					z++;
					y=0;
				}
			}

			strncpy(city_site.name,str[0],49);
			city_site.lat=ReadBearing(str[1]);
			city_site.lon=ReadBearing(str[2]);
			city_site.alt=0.0;

			NormalLongitude(city_site.lon);

			PlaceMarker(city_site);

			fgets(input,78,fd);
		}

		fclose(fd);
		pmsg( stdout, (sdf_done) );
		fflush(stdout);
	}

	else
	pmsg( stderr, (error_file_not_found), filename);
}		// LoadCities(char *filename)


void LoadUDT(char *filename)
{
	// This function reads a file containing User-Defined Terrain
	// features for their addition to the digital elevation model
	// data used by rfprobe.  Elevations in the UDT file are evaluated
	// and then copied into a temporary file under /tmp.  Then the
	// contents of the temp file are scanned, and if found to be unique,
	// are added to the ground elevations described by the digital
	// elevation data already loaded into memory.

	int	i;
	int x;
	int y;
	int z;
	int ypix;
	int xpix;
	int tempxpix;
	int tempypix;
	int fd=0;

	char input[80];
	char str[3][80];
	char tempname[15];
	char *pointer=NULL;
	double latitude;
	double longitude;
	double height;
	double tempheight;

	FILE *fd1=NULL;
	FILE *fd2=NULL;

	strcpy(tempname,"/tmp/XXXXXX\0");

	fd1=fopenFullPath(filename,"r");

	if (fd1!=NULL)
	{
		fd=mkstemp(tempname);
		fd2=fopenFullPath(tempname,"w");

		fgets(input,78,fd1);

		pointer=strchr(input,';');

		if (pointer!=NULL)
		*pointer=0;

		pmsg( stdout, (reading_file), filename);
		fflush(stdout);

		while (feof(fd1)==0)
		{
			// Parse line for latitude, longitude, height

			for (x=0, y=0, z=0; x<78 && input[x]!=0 && z<3; x++)
			{
				if (input[x]!=',' && y<78)
				{
					str[z][y]=input[x];
					y++;
				}

				else
				{
					str[z][y]=0;
					z++;
					y=0;
				}
			}

			latitude=ReadBearing(str[0]);
			longitude=ReadBearing(str[1]);

			NormalLongitude(longitude);

			// Remove <CR> and/or <LF> from antenna height string

			for (i=0; str[2][i]!='\r' && str[2][i]!='\n' && str[2][i]!=0; i++);

			str[2][i]=0;

			// The terrain feature may be expressed in either
			// feet or meters.  If the letter 'M' or 'm' is
			// discovered in the string, then this is an
			// indication that the value given is expressed
			// in meters.  Otherwise the height is interpreted
			// as being expressed in feet.

			for (i=0; str[2][i]!='M' && str[2][i]!='m' && str[2][i]!=0 && i<48; i++);

			if (str[2][i]=='M' || str[2][i]=='m')
			{
				str[2][i]=0;
				height=rint(atof(str[2]));
			}

			else
			{
				str[2][i]=0;
				height=rint(fo2Meters(atof(str[2])));
			}

			if (height>0.0)
			fprintf(fd2,"%d, %d, %f\n",(int)rint(latitude/DegreesPerPoint), (int)rint(longitude/DegreesPerPoint), height);

			fgets(input,78,fd1);

			pointer=strchr(input,';');

			if (pointer!=NULL)
			*pointer=0;
		}

		fclose(fd1);
		fclose(fd2);
		close(fd);

		pmsg( stdout, (sdf_done) );
		fflush(stdout);

		fd1=fopenFullPath(tempname,"r");
		fd2=fopenFullPath(tempname,"r");

		y=0;

		fscanf(fd1,"%d, %d, %lf", &xpix, &ypix, &height);

		do
		{
			x=0;
			z=0;

			fscanf(fd2,"%d, %d, %lf", &tempxpix, &tempypix, &tempheight);

			do
			{
				if (x>y && xpix==tempxpix && ypix==tempypix)
				{
					z=1;  // Dupe!

					if (tempheight>height)
					height=tempheight;
				}

				else
				{
					fscanf(fd2,"%d, %d, %lf", &tempxpix, &tempypix, &tempheight);
					x++;
				}

			} while (feof(fd2)==0 && z==0);

			if (z==0)  // No duplicate found
			AddHeightToElevation(xpix*DegreesPerPoint, ypix*DegreesPerPoint, height);

			fscanf(fd1,"%d, %d, %lf", &xpix, &ypix, &height);
			y++;

			rewind(fd2);

		} while (feof(fd1)==0);

		fclose(fd1);
		fclose(fd2);
		unlink(tempname);
	}

	else
	pmsg( stderr, (error_file_not_found), filename);

	fprintf(stdout, "\n");
}

void LoadBoundaries(char *filename)
{
	// This function reads Cartographic Boundary Files available from
	// the U.S. Census Bureau, and plots the data contained in those
	// files on the PPM Map generated by rfprobe.  Such files contain
	// the coordinates that describe the boundaries of cities,
	// counties, and states.

	int	x;
	double	lat0, lon0, lat1, lon1;
	char	string[80];
	site_data source=ZeroSiteData;
	site_data destination=ZeroSiteData;
	FILE	*fd=NULL;

	fd=fopenFullPath(filename,"r");

	if (fd!=NULL)
	{
		fgets(string,78,fd);

		pmsg( stdout, (reading_file), filename);
		fflush(stdout);

		do
		{
			fgets(string,78,fd);
			sscanf(string,"%lf %lf", &lon0, &lat0);
			fgets(string,78,fd);

			do
			{
				sscanf(string,"%lf %lf", &lon1, &lat1);

				source.lat=lat0;
				source.lon=(lon0>0.0 ? 360.0-lon0 : -lon0);
				destination.lat=lat1;
				destination.lon=(lon1>0.0 ? 360.0-lon1 : -lon1);

				ReadPath(source,destination);

				for (x=0; x<path.length; x++)
				OrMask(path.lat[x],path.lon[x],4);

				lat0=lat1;
				lon0=lon1;

				fgets(string,78,fd);

			} while (strncmp(string,"END",3)!=0 && feof(fd)==0);

			fgets(string,78,fd);

		} while (strncmp(string,"END",3)!=0 && feof(fd)==0);

		fclose(fd);

		pmsg( stdout, (sdf_done) );
		fflush(stdout);
	}

	else
	pmsg( stderr, (error_file_not_found), filename);
}

char ReadLRParm(site_data txsite, char forced_read)
{
	// This function reads ITM parameter data for the transmitter
	// site.  The file name is the same as the txsite, except the
	// filename extension is .lrp.  If the needed file is not found,
	// then the file fn_splat_lrp is read from the current working
	// directory.  Failure to load this file under a forced_read
	// condition will result in the default parameters hard coded
	// into this function to be used and written to fn_splat_lrp.

	double din;
	char filename[PATH_MAX];
	char string[80];
	char *pointer=NULL;
	char return_value=0;
	int iin;
	int ok=0;
	// int x;
	FILE *fd=NULL;
	FILE *outfile=NULL;

	// Default parameters

	LR.EarthDielectricConst=0.0;
	LR.EarthConductivity=0.0;
	LR.AtmBendingCons=0.0;
	LR.frq_mhz=0.0;
	LR.radio_climate=0;
	LR.polarization=POL_HORIZONTAL;
	LR.FractionSituations=0.0;
	LR.FractionTime=0.0;
	LR.EffRadiatedPower=0.0;

	// Generate .lrp filename from txsite filename.

	//for (x=0; txsite.filename[x]!='.' && txsite.filename[x]!=0 && x<250; x++)
	//	filename[x]=txsite.filename[x];
	SetFileName(filename, txsite.filename,"lrp");

	fd=fopenFullPath(filename,"r");

	if (fd==NULL)
	{
		// Load default fn_splat_lrp file

		// -------------------------
		// ORI: strncpy(filename,"splat.lrp\0",10);
		// fd=fopenFullPath(filename,"r");
		// -------------------------
		// MR: 16/05: inserted to solve path problems
		fd=fopen(fn_splat_lrp,"r");
	}

	if (fd!=NULL)
	{
		fgets(string,80,fd);

		pointer=strchr(string,';');

		if (pointer!=NULL)
		*pointer=0;

		ok=sscanf(string,"%lf", &din);

		if (ok)
		{
			LR.EarthDielectricConst=din;

			fgets(string,80,fd);

			pointer=strchr(string,';');

			if (pointer!=NULL)
			*pointer=0;

			ok=sscanf(string,"%lf", &din);
		}

		if (ok)
		{
			LR.EarthConductivity=din;

			fgets(string,80,fd);

			pointer=strchr(string,';');

			if (pointer!=NULL)
			*pointer=0;

			ok=sscanf(string,"%lf", &din);
		}

		if (ok)
		{
			LR.AtmBendingCons=din;

			fgets(string,80,fd);

			pointer=strchr(string,';');

			if (pointer!=NULL)
			*pointer=0;

			ok=sscanf(string,"%lf", &din);
		}

		if (ok)
		{
			LR.frq_mhz=din;

			fgets(string,80,fd);

			pointer=strchr(string,';');
			if (pointer!=NULL)
			{
				*pointer=0;
			}

			ok=sscanf(string,"%d", &iin);
		}

		if (ok)
		{
			LR.radio_climate=iin;

			fgets(string,80,fd);

			pointer=strchr(string,';');
			if (pointer!=NULL)
			{
				*pointer=0;
			}

			ok=sscanf(string,"%d", &iin);
		}

		if (ok)
		{
			LR.polarization=iin;

			fgets(string,80,fd);

			pointer=strchr(string,';');
			if (pointer!=NULL)
			{
				*pointer=0;
			}

			ok=sscanf(string,"%lf", &din);
		}

		if (ok)
		{
			LR.FractionSituations=din;

			fgets(string,80,fd);

			pointer=strchr(string,';');
			if (pointer!=NULL)
			{
				*pointer=0;
			}

			ok=sscanf(string,"%lf", &din);
		}

		if (ok)
		{
			LR.FractionTime=din;
			din=0.0;
			return_value=1;

			if (fgets(string,80,fd)!=NULL)
			{
				pointer=strchr(string,';');

				if (pointer!=NULL)
				*pointer=0;

				if (sscanf(string,"%lf", &din))
				LR.EffRadiatedPower=din;

				// ERP in rfprobe is referenced to 1 Watt
				// into a dipole (0 dBd).  If ERP is
				// expressed in dBm (referenced to a
				// 0 dBi radiator), convert dBm in EIRP
				// to ERP.

				if ((strstr(string, "dBm")!=NULL) || (strstr(string,"dbm")!=NULL))
				LR.EffRadiatedPower=(pow(10.0,(LR.EffRadiatedPower-32.14)/10.0));
			}
		}

		fclose(fd);

		if (forced_EffRadiatedPower!=-1.0)
		{
			LR.EffRadiatedPower=forced_EffRadiatedPower;
		}

		if (forced_freq>=20.0 && forced_freq<=20000.0)
		{
			LR.frq_mhz=forced_freq;
		}

		if (ok)
		LoadPAT(filename);
	}

	if (fd==NULL && forced_read)
	{
		// Assign some default parameters for use in this run.

		LR.EarthDielectricConst=15.0;
		LR.EarthConductivity=0.005;
		LR.AtmBendingCons=301.0;
		LR.frq_mhz=300.0;
		LR.radio_climate=CLIMA_CONTINENTAL_TEMPERATE;
		LR.polarization=POL_HORIZONTAL;
		LR.FractionSituations=0.50;
		LR.FractionTime=0.50;
		LR.EffRadiatedPower=0.0;

		// Write them to a fn_splat_lrp file.

		outfile=fopen(fn_splat_lrp,"w");

		fprintf(outfile,"%.3f\t; Earth Dielectric Constant (Relative permittivity)\n",LR.EarthDielectricConst);
		fprintf(outfile,"%.3f\t; Earth Conductivity (Siemens per meter)\n", LR.EarthConductivity);
		fprintf(outfile,"%.3f\t; Atmospheric Bending Constant (N-Units)\n",LR.AtmBendingCons);
		fprintf(outfile,"%.3f\t; Frequency in MHz (20 MHz to 20 GHz)\n", LR.frq_mhz);
		fprintf(outfile,"%d\t; Radio Climate\n",LR.radio_climate);
		fprintf(outfile,"%d\t; Polarization (0 = Horizontal, 1 = Vertical)\n", LR.polarization);
		fprintf(outfile,"%.2f\t; Fraction of Situations\n",LR.FractionSituations);
		fprintf(outfile,"%.2f\t; Fraction of Time\n",LR.FractionTime);
		fprintf(outfile,"%.2f\t; Transmitter Effective Radiated Power in Watts or dBm (optional)\n",LR.EffRadiatedPower);
		fprintf(outfile,"\nPlease consult rfprobe documentation for the meaning and use of this data.\n");

		fclose(outfile);

		return_value=1;

		fprintf(stderr,"\n\n%c*** There were problems reading your \"%s\" file! ***\nA \"splat.lrp\" file was written to your directory with default data.\n",7,filename);
	}

	else if (forced_read==0)
	return_value=0;

	if (forced_read && (fd==NULL || ok==0))
	{
		LR.EarthDielectricConst=15.0;
		LR.EarthConductivity=0.005;
		LR.AtmBendingCons=301.0;
		LR.frq_mhz=300.0;
		LR.radio_climate=CLIMA_CONTINENTAL_TEMPERATE;
		LR.polarization=POL_HORIZONTAL;
		LR.FractionSituations=0.50;
		LR.FractionTime=0.50;
		LR.EffRadiatedPower=0.0;

		fprintf(stderr,"Default parameters have been assumed for this analysis.\n");

		return_value=1;
	}

	return (return_value);
}

void PlotPath(site_data source, site_data destination, char mask_value)
{
	// This function analyzes the path between the source and
	// destination locations.  It determines which points along
	// the path have line-of-sight visibility to the source.
	// Points along with path having line-of-sight visibility
	// to the source at an AGL altitude equal to that of the
	// destination location are stored by setting bit 1 in the
	// mask[][] array, which are displayed in green when PPM
	// maps are later generated by rfprobe.

	char FoundObstruction;
	int x;
	int y;
	register double cos_xmtr_angle;
	register double cos_test_angle;
	register double test_alt;
	double distance;
	double rx_alt;
	double tx_alt;

	ReadPath(source,destination);

	for (y=0; y<path.length; y++)
	{
		// Test this point only if it hasn't been already tested and found to be free of obstructions.

		if ((GetMask(path.lat[y],path.lon[y])&mask_value)==0)
		{
			distance=mi2Foot(path.distance[y]);
			tx_alt=earthRadiusFoot+source.alt+path.elevation[0];
			rx_alt=earthRadiusFoot+destination.alt+path.elevation[y];

			// Calculate the cosine of the elevation of the	transmitter as seen at the temp rx point.

			cos_xmtr_angle=((rx_alt*rx_alt)+(distance*distance)-(tx_alt*tx_alt))/(2.0*rx_alt*distance);

			for (x=y, FoundObstruction=0; x>=0 && FoundObstruction==0; x--)
			{
				distance=mi2Foot( (path.distance[y]-path.distance[x]) );
				test_alt=earthRadiusFoot+(path.elevation[x]==0.0?path.elevation[x]:path.elevation[x]+clutter);

				cos_test_angle=((rx_alt*rx_alt)+(distance*distance)-(test_alt*test_alt))/(2.0*rx_alt*distance);

				// Compare these two angles to determine if
				// an obstruction exists.  Since we're comparing
				// the cosines of these angles rather than
				// the angles themselves, the following "if"
				// statement is reversed from what it would
				// be if the actual angles were compared.

				if (cos_xmtr_angle>=cos_test_angle)
				{
					FoundObstruction=1;
				}
			}

			if (FoundObstruction==0)
			{
				OrMask(path.lat[y],path.lon[y],mask_value);
			}
		}
	}
}

///////////////////////////////////////////////////////////////////
// analysis specific site data
//
typedef struct {
	char   SiteName[50];			// site name
	double lat;
	double lon;
	float alt;						// antenna height
	double Elevation;				// site elevation
	double AntennaHground;			// antenna height ground
	double AntennaHSea;				// antenna height above sea
	double HAAT_AntHeightAboveAverageTerrain;	// haat: Height Antenna above Average Terrain
	// parameters calculated from site to other End Site
	double azimuth;					// azimuth from actual site to other end site
	double ElevationAngle;			// elevation angle in degrees of the end site as seen from this site
	double AngleObstruction1;		// angle to the first obstruction
	double pattern;
	double patterndB;
} site_aninfo;

// analysis specific results
typedef struct {
	char model[50];
	char PropagationMode[50];
	double path_loss;
	double total_loss;
	char strmode[50];				// mode type msg
	char extmode[50];				// extended mode type msg
	int ErrorCode;
	char ErrorMsg[200];
} analysis_info;

// obstruction info
typedef struct {
	double lat;
	double lon;
	double distance;				// distance (km)
	double height;					// height (m) above sea level
} obstruction_info;

#define MAX_OBSTRUCTIONS	40		// n. max obstructions

// struct with the result of path report
typedef struct {
	int PropagationModel;
	char AnalysisModel[50];

	char units;						// if 1 metric units
	// parameters data_LR used in analysis (all)
	data_LR lrPar;					// data_LR used in analysis
	double WaveLength;				// wavelen calculated with LR.frq_mhz

	// parameters of lrPar converted in string
	char RadioClimateType[50];
	char PolarizationType[50];

	char   txERP_W[30];				// Transmitter Effective Radiated Power in Watts
	double txERP_dBm;				// Transmitter Effective Radiated Power in dBm
	char   txEIRP_W[30];			// EIRP in W: equivalent isotropically radiated power (effective isotropically radiated power)
	double txEIRP_dBm;				// EIRP in dBm

	site_aninfo Tx;					// tx site data
	site_aninfo Rx;					// rx site data

	double TxRxDistance;			// distance from tx and rx
	double TxRxAntennaPattern;
	double TxRxAntennaPattern_dB;

	// model analysis
	analysis_info lrice;			// result of longley rice model
	analysis_info itwom;			// result of itwom model

	double free_space_loss;

	double path_loss;
	double patterndB;
	char strmode[50];				// mode type msg
	char extmode[50];				// extended mode type msg
	int ErrorCode;
	char ErrorMsg[200];
	double total_loss;
	double maxloss;
	double minloss;
	//-----------------

	double TerrainShieldingAttenuation;
	double PathLossWithTxAntennaPattern;
	double RaiseTerrainForGroundClutter;

	double field_strength;
	double power_density;

	// parameters
	double FieldStrength;
	double SignalPowerLevel;
	double SignalPowerDensity;
	double VoltageAcross50OhmDipole_uV;
	double VoltageAcross50OhmDipole_dBuV;
	double VoltageAcross75OhmDipole_uV;
	double VoltageAcross75OhmDipole_dBuV;

	// measurement units
	char umFieldStrength[20];
	char umSignalPowerLevel[20];
	char umSignalPowerDensity[20];
	char umVoltageAcross50OhmDipole_uV[20];
	char umVoltageAcross50OhmDipole_dBuV[20];
	char umVoltageAcross75OhmDipole_uV[20];
	char umVoltageAcross75OhmDipole_dBuV[20];

	// path obstructions
	int NumObstructions;
	obstruction_info obstruction[MAX_OBSTRUCTIONS];

	double RxAntennaRaiseAboveGround;
	double RxAntennaRaiseAboveGroundPercFresnel;

} analysis_data_path;

analysis_data_path anPath;			// data path report

// =============================================================================

// added 04/10/2016
// inserted to substitute different calls to:
// void point_to_point
// void point_to_point_ITM
// defined in itwom3.0.cpp.
void Point2PointModel(
		double elev[], 
		double source_altitude,			// source (Transceiver) above ground level		
		double destination_altitude,	// destination (Receiver) above groud level
		data_LR *parameters,			// parameters used in model calc
		double &result_path_loss_db,	// path loss result value in db
		char *result_strmode,			// result of string mode
		int &result_err					// result error
)
{
	double src_alt_m;					// source altitude in m
	double dst_alt_m;					// dest altitude in m
	
	// convert altitude in foot to m
	src_alt_m = fo2Meters(source_altitude);
	dst_alt_m = fo2Meters(destination_altitude);
	
	// calc using propagation model
	if(PropagationModel==PROP_MODEL_LRICE)
	{
		// point to point using longley rice model
		point_to_point_ITM(
			elev,
			src_alt_m,							// source (Transceiver) above ground level	
			dst_alt_m,							// destination (Receiver) above groud level
			parameters->EarthDielectricConst,
			parameters->EarthConductivity,
			parameters->AtmBendingCons,
			parameters->frq_mhz,
			parameters->radio_climate,
			parameters->polarization,
			parameters->FractionSituations,
			parameters->FractionTime,
			result_path_loss_db,
			result_strmode,
			result_err
		);
	}
	else
	{
		// point to point using ITWOM_Ver3.0 model
		point_to_point(
			elev,
			src_alt_m,							// source (Transceiver) above ground level	
			dst_alt_m,							// destination (Receiver) above groud level
			parameters->EarthDielectricConst,
			parameters->EarthConductivity,
			parameters->AtmBendingCons,
			parameters->frq_mhz,
			parameters->radio_climate,
			parameters->polarization,
			parameters->FractionSituations,
			parameters->FractionTime,
			result_path_loss_db,
			result_strmode,
			result_err
		);
	}
}		// end Point2PointModel

// used to save model results
void SaveModelResults(
		double path_loss,				// path loss result value in db
		char *strmode,					// result of string mode
		int errnum						// result error
)
{
	if(PropagationModel==PROP_MODEL_LRICE)
	{
		// point to point using longley rice model
		// set analysis results
		anPath.path_loss = path_loss;
		anPath.ErrorCode = errnum;
		snprintf(anPath.strmode, 50, "%s", strmode);
		snprintf(anPath.extmode, 50, "%s", strmode);

		anPath.lrice.path_loss = path_loss;
		anPath.lrice.ErrorCode = errnum;
		snprintf(anPath.lrice.strmode, 50, "%s", strmode);
		snprintf(anPath.lrice.extmode, 50, "%s", strmode);
	}
	else
	{
		// point to point using ITWOM_Ver3.0 model
		// set analysis results
		anPath.path_loss = path_loss;
		anPath.ErrorCode = errnum;
		snprintf(anPath.strmode, 50, "%s", strmode);
		snprintf(anPath.extmode, 50, "%s", strmode);

		anPath.itwom.path_loss = path_loss;
		anPath.itwom.ErrorCode = errnum;
		snprintf(anPath.itwom.strmode, 50, "%s", strmode);
		snprintf(anPath.itwom.extmode, 50, "%s", strmode);
	}
}
		
///////////////////////////////////////////////////////////////////

void PlotLRPath(site_data source, site_data destination, unsigned char mask_value, FILE *fd)
{
	// This function plots the RF path loss between source and
	// destination points based on the ITWOM propagation model,
	// taking into account antenna pattern data, if available.

	int	x;
	int y;
	int ifs;
	int ofs;
	int errnum;
	char FoundObstruction=0;
	char strmode[100];
	double loss;
	double azimuth;
	double pattern=0.0;
	double xmtr_alt;
	double dest_alt;
	double xmtr_alt2;
	double dest_alt2;
	double cos_rcvr_angle;
	double cos_test_angle=0.0;
	double test_alt;
	double elevation=0.0;
	double distance=0.0;
	// double four_thirds_earthRadiusFoot;
	double rxp;
	double dBm;
	double field_strength=0.0;

	site_data temp=ZeroSiteData;

	ReadPath(source,destination);

	// four_thirds_earthRadiusFoot=FOUR_THIRDS*EARTHRADIUS_FOOT;

	// Copy elevations plus clutter along path into the elev[] array.
	COPY_ELEVATIONS_PLUS_CLUTTER();

	// Since the only energy the propagation model considers
	// reaching the destination is based on what is scattered
	// or deflected from the first obstruction along the path,
	// we first need to find the location and elevation angle
	// of that first obstruction (if it exists).  This is done
	// using a 4/3rds Earth radius to match the radius used by
	// the irregular terrain propagation model.  This information
	// is required for properly integrating the antenna's elevation
	// pattern into the calculation for overall path loss.

	for (y=2; (y<(path.length-1) && path.distance[y]<=max_range); y++)
	{
		// Process this point only if it has not already been processed.

		if ((GetMask(path.lat[y],path.lon[y])&248)!=(mask_value<<3))
		{
			distance=mi2Foot( path.distance[y] );
			xmtr_alt=four_thirds_earthRadiusFoot+source.alt+path.elevation[0];
			dest_alt=four_thirds_earthRadiusFoot+destination.alt+path.elevation[y];
			dest_alt2=dest_alt*dest_alt;
			xmtr_alt2=xmtr_alt*xmtr_alt;

			// Calculate the cosine of the elevation of	the receiver as seen by the transmitter.

			cos_rcvr_angle=((xmtr_alt2)+(distance*distance)-(dest_alt2))/(2.0*xmtr_alt*distance);

			if (cos_rcvr_angle>1.0)
			{
				cos_rcvr_angle=1.0;
			}
			if (cos_rcvr_angle<-1.0)
			{
				cos_rcvr_angle=-1.0;
			}

			if (got_elevation_pattern || fd!=NULL)
			{
				// Determine the elevation angle to the first obstruction
				// along the path IF elevation pattern data is available
				// or an output (.ano) file has been designated.

				for (x=2, FoundObstruction=0; (x<y && FoundObstruction==0); x++)
				{
					distance=mi2Foot( path.distance[x] );

					test_alt=four_thirds_earthRadiusFoot+(path.elevation[x]==0.0?path.elevation[x]:path.elevation[x]+clutter);

					// Calculate the cosine of the elevation
					// angle of the terrain (test point)
					// as seen by the transmitter.

					cos_test_angle=((xmtr_alt2)+(distance*distance)-(test_alt*test_alt))/(2.0*xmtr_alt*distance);

					if (cos_test_angle>1.0)
					{
						cos_test_angle=1.0;
					}
					if (cos_test_angle<-1.0)
					{
						cos_test_angle=-1.0;
					}

					// Compare these two angles to determine if
					// an obstruction exists.  Since we're comparing
					// the cosines of these angles rather than
					// the angles themselves, the sense of the
					// following "if" statement is reversed from
					// what it would be if the angles themselves
					// were compared.

					if (cos_rcvr_angle>=cos_test_angle)
					{
						FoundObstruction=1;
					}
				}

				if (FoundObstruction)
				{
					elevation=(DEGREES(acos(cos_test_angle)))-90.0;
				}
				else
				{
					elevation=(DEGREES(acos(cos_rcvr_angle)))-90.0;
				}
			}

			// Determine attenuation for each point along
			// the path using ITWOM's point_to_point mode
			// starting at y=2 (number_of_points = 1), the
			// shortest distance terrain can play a role in
			// path loss.

			elev[0]=y-1;  // (number of points - 1)

			// Distance between elevation samples

			elev[1]=METERS_PER_MILE*(path.distance[y]-path.distance[y-1]);

			// in base of model, calc path loss
			Point2PointModel(
				elev,
				(source.alt),
				(destination.alt),
				&LR,
				loss,
				strmode,
				errnum
			);

			temp.lat=path.lat[y];
			temp.lon=path.lon[y];

			azimuth=(AzimuthSites(source,temp));

			if (fd!=NULL)
			{
				fprintf(fd,"%.7f, %.7f, %.3f, %.3f, ",path.lat[y], path.lon[y], azimuth, elevation);
			}

			// If ERP==0, write path loss to alphanumeric output file.
			// Otherwise, write field strength
			// or received power level (below), as appropriate.

			if (fd!=NULL && LR.EffRadiatedPower==0.0)
			{
				fprintf(fd,"%.2f",loss);
			}

			// Integrate the antenna's radiation pattern into the overall path loss.

			x=(int)rint(10.0*(10.0-elevation));

			if (x>=0 && x<=1000)
			{
				azimuth=rint(azimuth);

				pattern=(double)LR.antenna_pattern[(int)azimuth][x];

				if (pattern!=0.0)
				{
					pattern=20.0*log10(pattern);
					loss-=pattern;
				}
			}

			if (LR.EffRadiatedPower!=0.0)
			{
				if (dbm)
				{
					// dBm is based on EIRP (ERP + 2.14)

					rxp=LR.EffRadiatedPower/(pow(10.0,(loss-2.14)/10.0));

					dBm=10.0*(log10(rxp*1000.0));

					if (fd!=NULL)
					{
						fprintf(fd,"%.3f",dBm);
					}

					// Scale roughly between 0 and 255

					ifs=200+(int)rint(dBm);

					if (ifs<0)
					{
						ifs=0;
					}
					if (ifs>255)
					{
						ifs=255;
					}

					// reads the signal level (0-255) at the specified location
					// that was previously written by the complimentary PutSignal() function
					ofs=GetSignal(path.lat[y],path.lon[y]);
					if (ofs>ifs)
					{
						ifs=ofs;
					}

					PutSignal(path.lat[y],path.lon[y],(unsigned char)ifs);
				}

				else
				{
					field_strength=(139.4+(20.0*log10(LR.frq_mhz))-loss)+(10.0*log10(LR.EffRadiatedPower/1000.0));

					ifs=100+(int)rint(field_strength);

					if (ifs<0)
					{
						ifs=0;
					}
					if (ifs>255)
					{
						ifs=255;
					}
					// reads the signal level (0-255) at the specified location
					// that was previously written by the complimentary PutSignal() function
					ofs=GetSignal(path.lat[y],path.lon[y]);

					if (ofs>ifs)
					{
						ifs=ofs;
					}
					PutSignal(path.lat[y],path.lon[y],(unsigned char)ifs);

					if (fd!=NULL)
					{
						fprintf(fd,"%.3f",field_strength);
					}
				}
			}

			else
			{
				if (loss>255)
				{
					ifs=255;
				}
				else
				{
					ifs=(int)rint(loss);
				}
				ofs=GetSignal(path.lat[y],path.lon[y]);
				if (ofs<ifs && ofs!=0)
				{
					ifs=ofs;
				}
				PutSignal(path.lat[y],path.lon[y],(unsigned char)ifs);
			}

			if (fd!=NULL)
			{
				if (FoundObstruction)
				{
					fprintf(fd," *");
				}
				fprintf(fd,"\n");
			}

			// Mark this point as having been analyzed

			PutMask(path.lat[y],path.lon[y],(GetMask(path.lat[y],path.lon[y])&7)+(mask_value<<3));
		}
	}
}

void PlotLOSMap(site_data source, double altitude)
{
	// This function performs a 360 degree sweep around the
	// transmitter site (source location), and plots the
	// line-of-sight coverage of the transmitter on the rfprobe
	// generated topographic map based on a receiver located
	// at the specified altitude (in feet AGL).  Results
	// are stored in memory, and written out in the form
	// of a topographic map when the WritePPM() function
	// is later invoked.

	int y;
	int z;
	int idxSymbol;
	site_data edge=ZeroSiteData;
	unsigned char symbol[4];
	unsigned char x;
	double lat;
	double lon;
	double minLon;
	double maxLat;
	double th;
	static unsigned char mask_value=1;

	symbol[0]='.';
	symbol[1]='o';
	symbol[2]='O';
	symbol[3]='o';

	fprintf(stdout, "\n");
	pmsg( stdout, (Computing_lineofsight_coverage) , source.name,metric?fo2Meters(altitude):altitude,metric?"meters":"feet");

	if (clutter>0.0)
	pmsg( stdout, (and_ground_clutter) , metric?fo2Meters(clutter):clutter,metric?"meters":"feet");

	fprintf(stdout, "...\n\n");
	pmsg( stdout, (perc_00_to_25), 37,37);
	fflush(stdout);

	// th=points/degree divided by 64 loops per	progress indicator symbol (.oOo) printed.
	th=ppd/64.0;
	idxSymbol=0;
	z=(int)(th*ReduceAngle(max_lon-min_lon));

	minLon=DegreesPerPoint+(double)min_lon;
	maxLat=(double)max_lat-DegreesPerPoint;

	for (lon=minLon, x=0, y=0; (DeltaLongitude(lon,(double)max_lon)<=0.0); y++, lon=SET_LONGITUDE_TOUP(y) )
	{
		if (lon>=360.0)
		lon-=360.0;

		edge.lat=max_lat;
		edge.lon=lon;
		edge.alt=altitude;

		PlotPath(source,edge,mask_value);
		UpdateProgressIndicator();
	}

	fprintf(stdout, "\n");
	pmsg( stdout, (perc_25_to_50), 37,37);
	fflush(stdout);

	idxSymbol=0;
	z=(int)(th*(double)(max_lat-min_lat));

	for (lat=maxLat, x=0, y=0; lat>=(double)min_lat; y++, lat=SET_LATITUDE_TODOWN(y) )
	{
		edge.lat=lat;
		edge.lon=min_lon;
		edge.alt=altitude;

		PlotPath(source,edge,mask_value);
		UpdateProgressIndicator();
	}

	fprintf(stdout, "\n");
	pmsg( stdout, (perc_50_to_75), 37,37);
	fflush(stdout);

	idxSymbol=0;
	z=(int)(th*ReduceAngle(max_lon-min_lon));

	for (lon=minLon, x=0, y=0; (DeltaLongitude(lon,(double)max_lon)<=0.0); y++, lon=SET_LONGITUDE_TOUP(y) )
	{
		if (lon>=360.0)
		{
			lon-=360.0;
		}
		edge.lat=min_lat;
		edge.lon=lon;
		edge.alt=altitude;

		PlotPath(source,edge,mask_value);
		UpdateProgressIndicator();
	}

	fprintf(stdout, "\n");
	pmsg( stdout, (perc_75_to_100), 37,37);
	fflush(stdout);

	idxSymbol=0;
	z=(int)(th*(double)(max_lat-min_lat));

	for (lat=(double)min_lat, x=0, y=0; lat<(double)max_lat; y++, lat=SET_LATITUDE_TOUP(y) )
	{
		edge.lat=lat;
		edge.lon=max_lon;
		edge.alt=altitude;

		PlotPath(source,edge,mask_value);
		UpdateProgressIndicator();
	}

	fprintf(stdout, "\n");
	pmsg( stdout, (sdf_done) );
	fflush(stdout);

	// Assign next mask value

	switch (mask_value)
	{
	case 1:
		mask_value=8;
		break;

	case 8:
		mask_value=16;
		break;

	case 16:
		mask_value=32;
	}
}		// void PlotLOSMap()

void PlotLRMap(site_data source, double altitude, char *plo_filename)
{
	// This function performs a 360 degree sweep around the
	// transmitter site (source location), and plots the
	// Irregular Terrain Model attenuation on the rfprobe
	// generated topographic map based on a receiver located
	// at the specified altitude (in feet AGL).  Results
	// are stored in memory, and written out in the form
	// of a topographic map when the WritePPMLR() or
	// WritePPMSS() functions are later invoked.

	int y;
	int z;
	int idxSymbol;
	site_data edge=ZeroSiteData;
	double lat;
	double lon;
	double minLon;
	double maxLat;
	double th;
	unsigned char x;
	unsigned char symbol[4];
	static unsigned char mask_value=1;
	FILE *fd=NULL;

	minLon=DegreesPerPoint+(double)min_lon;
	maxLat=(double)max_lat-DegreesPerPoint;

	symbol[0]='.';
	symbol[1]='o';
	symbol[2]='O';
	symbol[3]='o';

	fprintf(stdout, "\n");
	if(PropagationModel==PROP_MODEL_LRICE)
	fprintf(stdout, "Computing ITM ");
	else
	fprintf(stdout, "Computing ITWOM ");

	if (LR.EffRadiatedPower==0.0)
	fprintf(stdout, "path loss");
	else
	{
		if (dbm)
		fprintf(stdout, "signal power level");
		else
		fprintf(stdout, "field strength");
	}

	fprintf(stdout, " contours of \"%s\"\nout to a radius of %.2f %s with an RX antenna at %.2f %s AGL",source.name,metric?max_range*KM_PER_MILE:max_range,metric?"km":"miles",metric?fo2Meters(altitude):altitude,metric?"meters":"feet");

	if (clutter>0.0)
	{
		fprintf(stdout, "\n");
		fprintf(stdout, "and %.2f %s of ground clutter",metric?fo2Meters(clutter):clutter,metric?"meters":"feet");
	}
	fprintf(stdout, "...\n\n");
	pmsg( stdout, (perc_00_to_25), 37,37);
	fflush(stdout);

	if (plo_filename[0]!=0)
	fd=fopenFullPath(plo_filename,"wb");

	if (fd!=NULL)
	{
		// Write header information to output file

		fprintf(fd,"%d, %d\t; max_west, min_west\n%d, %d\t; max_north, min_north\n",
				max_lon, min_lon, max_lat, min_lat);
	}

	// th=points/degree divided by 64 loops per	progress indicator symbol (.oOo) printed.
	th=ppd/64.0;
	idxSymbol=0;
	z=(int)(th*ReduceAngle(max_lon-min_lon));

	for (lon=minLon, x=0, y=0; (DeltaLongitude(lon,(double)max_lon)<=0.0); y++, lon=SET_LONGITUDE_TOUP(y) )
	{
		if (lon>=360.0)
		lon-=360.0;

		edge.lat=max_lat;
		edge.lon=lon;
		edge.alt=altitude;

		PlotLRPath(source,edge,mask_value,fd);
		UpdateProgressIndicator();
	}

	fprintf(stdout, "\n");
	pmsg( stdout, (perc_25_to_50), 37,37);
	fflush(stdout);

	idxSymbol=0;
	z=(int)(th*(double)(max_lat-min_lat));

	for (lat=maxLat, x=0, y=0; lat>=(double)min_lat; y++, lat=SET_LATITUDE_TODOWN(y) )
	{
		edge.lat=lat;
		edge.lon=min_lon;
		edge.alt=altitude;

		PlotLRPath(source,edge,mask_value,fd);
		UpdateProgressIndicator();
	}

	fprintf(stdout, "\n");
	pmsg( stdout, (perc_50_to_75), 37,37);
	fflush(stdout);

	idxSymbol=0;
	z=(int)(th*ReduceAngle(max_lon-min_lon));

	for (lon=minLon, x=0, y=0; (DeltaLongitude(lon,(double)max_lon)<=0.0); y++, lon=SET_LONGITUDE_TOUP(y) )
	{
		if (lon>=360.0)
		lon-=360.0;

		edge.lat=min_lat;
		edge.lon=lon;
		edge.alt=altitude;

		PlotLRPath(source,edge,mask_value,fd);

		UpdateProgressIndicator();
	}

	fprintf(stdout, "\n");
	pmsg( stdout, (perc_75_to_100), 37,37);
	fflush(stdout);

	idxSymbol=0;
	z=(int)(th*(double)(max_lat-min_lat));

	for (lat=(double)min_lat, x=0, y=0; lat<(double)max_lat; y++, lat=SET_LATITUDE_TOUP(y) )
	{
		edge.lat=lat;
		edge.lon=max_lon;
		edge.alt=altitude;

		PlotLRPath(source,edge,mask_value,fd);
		UpdateProgressIndicator();
	}

	if (fd!=NULL)
	fclose(fd);

	fprintf(stdout, "\n");
	pmsg( stdout, (sdf_done) );
	fflush(stdout);

	if (mask_value<30)
	mask_value++;
}

void LoadSignalColors(site_data xmtr)
{
	int x;
	int y;
	int ok;
	int val[4];
	char filename[255];
	char string[80];
	char *pointer=NULL;
	FILE *fd=NULL;

	// for (x=0; xmtr.filename[x]!='.' && xmtr.filename[x]!=0 && x<250; x++)
	// 	filename[x]=xmtr.filename[x];
	SetFileName(filename, xmtr.filename, (char *)"scf");

	// filename[x]='.';
	// filename[x+1]='s';
	// filename[x+2]='c';
	// filename[x+3]='f';
	// filename[x+4]=0;

	// Default values
	SetRegionData(0 , 128, 255 ,   0, 0);
	SetRegionData(1 , 118, 255 , 165, 0);
	SetRegionData(2 , 108, 255 , 206, 0);
	SetRegionData(3 ,  98, 255 , 255, 0);
	SetRegionData(4 ,  88, 184 , 255, 0);
	SetRegionData(5 ,  78,   0 , 255, 0);
	SetRegionData(6 ,  68,   0 , 208, 0);
	SetRegionData(7 ,  58,   0 , 196, 196);
	SetRegionData(8 ,  48,   0 , 148, 255);
	SetRegionData(9 ,  38,  80 ,  80, 255);
	SetRegionData(10 , 28,   0 ,  38, 255);
	SetRegionData(11 , 18, 142 ,  63, 255);
	SetRegionData(12 ,  8, 140 ,   0, 128);

	region.levels=13;

	fd=fopen(fn_splat_scf,"r");

	if (fd==NULL)
	fd=fopenFullPath(filename,"r");

	if (fd==NULL)
	{
		fd=fopenFullPath(filename,"w");

		fprintf(fd,"; rfprobe Auto-generated Signal Color Definition (\"%s\") File\n",filename);
		fprintf(fd,";\n; Format for the parameters held in this file is as follows:\n;\n");
		fprintf(fd,";    dBuV/m: red, green, blue\n;\n");
		fprintf(fd,"; ...where \"dBuV/m\" is the signal strength (in dBuV/m) and\n");
		fprintf(fd,"; \"red\", \"green\", and \"blue\" are the corresponding RGB color\n");
		fprintf(fd,"; definitions ranging from 0 to 255 for the region specified.\n");
		fprintf(fd,";\n; The following parameters may be edited and/or expanded\n");
		fprintf(fd,"; for future runs of rfprobe  A total of 32 contour regions\n");
		fprintf(fd,"; may be defined in this file.\n;\n;\n");

		for (x=0; x<region.levels; x++)
		fprintf(fd,"%3d: %3d, %3d, %3d\n",region.level[x], region.color[x][0], region.color[x][1], region.color[x][2]);

		fclose(fd);
	}

	else
	{
		x=0;
		fgets(string,80,fd);

		while (x<32 && feof(fd)==0)
		{
			pointer=strchr(string,';');

			if (pointer!=NULL)
			*pointer=0;

			ok=sscanf(string,"%d: %d, %d, %d", &val[0], &val[1], &val[2], &val[3]);

			if (ok==4)
			{
				for (y=0; y<4; y++)
				{
					if (val[y]>255)
					val[y]=255;

					if (val[y]<0)
					val[y]=0;
				}

				region.level[x]=val[0];
				region.color[x][0]=val[1];
				region.color[x][1]=val[2];
				region.color[x][2]=val[3];
				x++;
			}

			fgets(string,80,fd);
		}

		fclose(fd);
		region.levels=x;
	}
}

void LoadLossColors(site_data xmtr)
{
	int x;
	int y;
	int ok;
	int val[4];
	char filename[255];
	char string[80];
	char *pointer=NULL;
	FILE *fd=NULL;

	// for (x=0; xmtr.filename[x]!='.' && xmtr.filename[x]!=0 && x<250; x++)
	// 	filename[x]=xmtr.filename[x];
	SetFileName(filename, xmtr.filename, (char *)"lcf");

	// filename[x]='.';
	// filename[x+1]='l';
	// filename[x+2]='c';
	// filename[x+3]='f';
	// filename[x+4]=0;

	// Default values

	SetRegionData(0	, 80,	255,	0,		0); 
	SetRegionData(1	, 90,	255,	128,	0); 
	SetRegionData(2	, 100,	255,	165,	0); 
	SetRegionData(3	, 110,	255,	206,	0); 
	SetRegionData(4	, 120,	255,	255,	0); 
	SetRegionData(5	, 130,	184,	255,	0); 
	SetRegionData(6	, 140,	0,		255,	0); 
	SetRegionData(7	, 150,	0,		208,	0); 
	SetRegionData(8	, 160,	0,		196,	196); 
	SetRegionData(9	, 170,	0,		148,	255); 
	SetRegionData(10, 180,	80,		80,		255); 
	SetRegionData(11, 190,	0,		38,		255); 
	SetRegionData(12, 200,	142,	63,		255); 
	SetRegionData(13, 210,	196,	54,		255); 
	SetRegionData(14, 220,	255,	0,		255); 
	SetRegionData(15, 230,	255,	194,	204); 
	region.levels=16;

	fd=fopen(fn_splat_lcf,"r");

	if (fd==NULL)
	fd=fopenFullPath(filename,"r");

	if (fd==NULL)
	{
		fd=fopenFullPath(filename,"w");

		fprintf(fd,"; rfprobe Auto-generated Path-Loss Color Definition (\"%s\") File\n",filename);
		fprintf(fd,";\n; Format for the parameters held in this file is as follows:\n;\n");
		fprintf(fd,";    dB: red, green, blue\n;\n");
		fprintf(fd,"; ...where \"dB\" is the path loss (in dB) and\n");
		fprintf(fd,"; \"red\", \"green\", and \"blue\" are the corresponding RGB color\n");
		fprintf(fd,"; definitions ranging from 0 to 255 for the region specified.\n");
		fprintf(fd,";\n; The following parameters may be edited and/or expanded\n");
		fprintf(fd,"; for future runs of rfprobe  A total of 32 contour regions\n");
		fprintf(fd,"; may be defined in this file.\n;\n;\n");

		for (x=0; x<region.levels; x++)
		fprintf(fd,"%3d: %3d, %3d, %3d\n",region.level[x], region.color[x][0], region.color[x][1], region.color[x][2]);

		fclose(fd);
	}

	else
	{
		x=0;
		fgets(string,80,fd);

		while (x<32 && feof(fd)==0)
		{
			pointer=strchr(string,';');

			if (pointer!=NULL)
			*pointer=0;

			ok=sscanf(string,"%d: %d, %d, %d", &val[0], &val[1], &val[2], &val[3]);

			if (ok==4)
			{
				for (y=0; y<4; y++)
				{
					if (val[y]>255)
					val[y]=255;

					if (val[y]<0)
					val[y]=0;
				}

				SetRegionData(x, val[0], val[1], val[2], val[3]);
				x++;
			}

			fgets(string,80,fd);
		}

		fclose(fd);
		region.levels=x;
	}
}

void LoadDBMColors(site_data xmtr)
{
	int x;
	int y;
	int ok;
	int val[4];
	char filename[255];
	char string[80];
	char *pointer=NULL;
	FILE *fd=NULL;

	// for (x=0; xmtr.filename[x]!='.' && xmtr.filename[x]!=0 && x<250; x++)
	// 	filename[x]=xmtr.filename[x];
	SetFileName(filename, xmtr.filename, (char *)"dcf");

	// filename[x]='.';
	// filename[x+1]='d';
	// filename[x+2]='c';
	// filename[x+3]='f';
	// filename[x+4]=0;

	// Default values

	SetRegionData(0	 ,	0,		255,	0,		0);
	SetRegionData(1	 ,	-10,	255,	128,	0);
	SetRegionData(2	 ,	-20,	255,	165,	0);
	SetRegionData(3	 ,	-30,	255,	206,	0);
	SetRegionData(4	 ,	-40,	255,	255,	0);
	SetRegionData(5	 ,	-50,	184,	255,	0);
	SetRegionData(6	 ,	-60,	0,		255,	0);
	SetRegionData(7	 ,	-70,	0,		208,	0);
	SetRegionData(8	 ,	-80,	0,		196,	196);
	SetRegionData(9	 ,	-90,	0,		148,	255);
	SetRegionData(10 ,	-100,	80,		80,		255);
	SetRegionData(11 ,	-110,	0,		38,		255);
	SetRegionData(12 ,	-120,	142,	63,		255);
	SetRegionData(13 ,	-130,	196,	54,		255);
	SetRegionData(14 ,	-140,	255,	0,		255);
	SetRegionData(15 ,	-150,	255,	194,	204);
	region.levels=16;

	fd=fopen(fn_splat_dcf,"r");

	if (fd==NULL)
	fd=fopenFullPath(filename,"r");

	if (fd==NULL)
	{
		fd=fopenFullPath(filename,"w");

		fprintf(fd,"; rfprobe Auto-generated DBM Signal Level Color Definition (\"%s\") File\n",filename);
		fprintf(fd,";\n; Format for the parameters held in this file is as follows:\n;\n");
		fprintf(fd,";    dBm: red, green, blue\n;\n");
		fprintf(fd,"; ...where \"dBm\" is the received signal power level between +40 dBm\n");
		fprintf(fd,"; and -200 dBm, and \"red\", \"green\", and \"blue\" are the corresponding\n");
		fprintf(fd,"; RGB color definitions ranging from 0 to 255 for the region specified.\n");
		fprintf(fd,";\n; The following parameters may be edited and/or expanded\n");
		fprintf(fd,"; for future runs of rfprobe  A total of 32 contour regions\n");
		fprintf(fd,"; may be defined in this file.\n;\n;\n");

		for (x=0; x<region.levels; x++)
		fprintf(fd,"%+4d: %3d, %3d, %3d\n",region.level[x], region.color[x][0], region.color[x][1], region.color[x][2]);

		fclose(fd);
	}

	else
	{
		x=0;
		fgets(string,80,fd);

		while (x<32 && feof(fd)==0)
		{
			pointer=strchr(string,';');

			if (pointer!=NULL)
			*pointer=0;

			ok=sscanf(string,"%d: %d, %d, %d", &val[0], &val[1], &val[2], &val[3]);

			if (ok==4)
			{
				if (val[0]<-200)
				val[0]=-200;

				if (val[0]>+40)
				val[0]=+40;

				region.level[x]=val[0];

				for (y=1; y<4; y++)
				{
					if (val[y]>255)
					val[y]=255;

					if (val[y]<0)
					val[y]=0;
				}

				region.color[x][0]=val[1];
				region.color[x][1]=val[2];
				region.color[x][2]=val[3];
				x++;
			}

			fgets(string,80,fd);
		}

		fclose(fd);
		region.levels=x;
	}
}

void WritePPM(char *filename, unsigned char geo, unsigned char kml, unsigned char ngs, site_data *xmtr, unsigned char txsites)
{
	// This function generates a topographic map in Portable Pix Map
	// (PPM) format based on logarithmically scaled topology data,
	// as well as the content of flags held in the mask[][] array.
	// The image created is rotated counter-clockwise 90 degrees
	// from its representation in dem[][] so that north points
	// up and east points right in the image generated.

	char mapfile[255];
	char geofile[255];
	char kmlfile[255];
	unsigned char found;
	unsigned char mask;
	unsigned width;
	unsigned height;
	int indx;
	int x;
	int y;
	int x0=0;
	int y0=0;
	double lat;
	double lon;
	double north;
	double south;
	double east;
	double west;
	double minLon;

	FILE *fd;

	IniGreyLevelPoints();	// initialize the grey level points

	width=NPointsLongitude();
	height=NPointsLatitude();

	if (filename[0]==0)
	{
		strncpy(filename, xmtr[0].filename,254);
		filename[strlen(filename)-4]=0;  // Remove .qth
	}

	y=strlen(filename);

	if (y>4)
	{
		if (filename[y-1]=='m' && filename[y-2]=='p' && filename[y-3]=='p' && filename[y-4]=='.')
		y-=4;
	}

	for (x=0; x<y; x++)
	{
		mapfile[x]=filename[x];
		geofile[x]=filename[x];
		kmlfile[x]=filename[x];
	}

	mapfile[x]='.';
	geofile[x]='.';
	kmlfile[x]='.';
	mapfile[x+1]='p';
	geofile[x+1]='g';
	kmlfile[x+1]='k';
	mapfile[x+2]='p';
	geofile[x+2]='e';
	kmlfile[x+2]='m';
	mapfile[x+3]='m';
	geofile[x+3]='o';
	kmlfile[x+3]='l';
	mapfile[x+4]=0;
	geofile[x+4]=0;
	kmlfile[x+4]=0;

	minLon=((double)min_lon)+DegreesPerPoint;
	if (minLon>360.0)
	{
		minLon-=360.0;
	}

	north=(double)max_lat-DegreesPerPoint;
	south=(double)min_lat;
	east=(minLon<180.0?-minLon:360.0-min_lon);
	west=(double)(max_lon<180?-max_lon:360-max_lon);

	if (kml==0 && geo)
	{
		fd=fopenFullPath(geofile,"wb");

		fprintf(fd,"FILENAME\t%s\n",mapfile);
		fprintf(fd,"#\t\tX\tY\tLong\t\tLat\n");
		fprintf(fd,"TIEPOINT\t0\t0\t%.3f\t\t%.3f\n",west,north);
		fprintf(fd,"TIEPOINT\t%u\t%u\t%.3f\t\t%.3f\n",width-1,height-1,east,south);
		fprintf(fd,"IMAGESIZE\t%u\t%u\n",width,height);
		fprintf(fd,"#\n# Auto Generated by %s v%s\n#\n",program_name,program_version);

		fclose(fd);
	}

	if (kml && geo==0)
	{
		fd=fopenFullPath(kmlfile,"wb");

		fprintf(fd,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		fprintf(fd,"<kml xmlns=\"http://earth.google.com/kml/2.1\">\n");
		fprintf(fd,"  <Folder>\n");
		fprintf(fd,"   <name>%s</name>\n",program_name);
		fprintf(fd,"     <description>Line-of-Sight Contour</description>\n");
		fprintf(fd,"       <GroundOverlay>\n");
		fprintf(fd,"         <name>%s Line-of-Sight Contour</name>\n",program_name);
		fprintf(fd,"           <description>rfprobe Coverage</description>\n");
		fprintf(fd,"		<Icon>\n");
		fprintf(fd,"              <href>%s</href>\n",mapfile);
		fprintf(fd,"		</Icon>\n");
		// fprintf(fd,"            <opacity>128</opacity>\n");
		fprintf(fd,"            <LatLonBox>\n");
		fprintf(fd,"               <north>%.5f</north>\n",north);
		fprintf(fd,"               <south>%.5f</south>\n",south);
		fprintf(fd,"               <east>%.5f</east>\n",east);
		fprintf(fd,"               <west>%.5f</west>\n",west);
		fprintf(fd,"               <rotation>0.0</rotation>\n");
		fprintf(fd,"            </LatLonBox>\n");
		fprintf(fd,"       </GroundOverlay>\n");

		for (x=0; x<txsites; x++)
		{
			fprintf(fd,"     <Placemark>\n");
			fprintf(fd,"       <name>%s</name>\n",xmtr[x].name);
			fprintf(fd,"       <visibility>1</visibility>\n");
			fprintf(fd,"       <Style>\n");
			fprintf(fd,"       <IconStyle>\n");
			fprintf(fd,"        <Icon>\n");
			fprintf(fd,"          <href>root://icons/palette-5.png</href>\n");
			fprintf(fd,"          <x>224</x>\n");
			fprintf(fd,"          <y>224</y>\n");
			fprintf(fd,"          <w>32</w>\n");
			fprintf(fd,"          <h>32</h>\n");
			fprintf(fd,"        </Icon>\n");
			fprintf(fd,"       </IconStyle>\n");
			fprintf(fd,"       </Style>\n");
			fprintf(fd,"      <Point>\n");
			fprintf(fd,"        <extrude>1</extrude>\n");
			fprintf(fd,"        <altitudeMode>relativeToGround</altitudeMode>\n");
			fprintf(fd,"        <coordinates>%f,%f,%f</coordinates>\n",(xmtr[x].lon<180.0?-xmtr[x].lon:360.0-xmtr[x].lon), xmtr[x].lat, xmtr[x].alt);
			fprintf(fd,"      </Point>\n");
			fprintf(fd,"     </Placemark>\n");
		}

		fprintf(fd,"  </Folder>\n");
		fprintf(fd,"</kml>\n");

		fclose(fd);
	}

	fd=fopenFullPath(mapfile,"wb");

	fprintf(fd,"P6\n%u %u\n255\n",width,height);
	fprintf(stdout, "\n");
	pmsg( stdout, (writing_pixmap_image), mapfile,width,height);
	fflush(stdout);

	for (y=0, lat=north; y<(int)height; y++, lat=north-(DegreesPerPoint*(double)y))
	{
		for (x=0, lon=max_lon; x<(int)width; x++, lon=SET_LONGITUDE_TODOWN(x) )
		{
			NormalLongitude(lon);

			// SEARCH_POINTMAP( lat, lon );
			found = SearchPointMap( lat, lon , indx, x0, y0);
			if (found)
			{
				mask=dem[indx].mask[x0][y0];

				if (mask&2)
				{
					// Text Labels: Red
					fprintf(fd,"%c%c%c",255,0,0);
				}
				else if (mask&4)
				{
					// County Boundaries: Light Cyan
					fprintf(fd,"%c%c%c",128,128,255);
				}
				else switch (mask&57)
				{
				case 1:
					// TX1: Green
					fprintf(fd,"%c%c%c",0,255,0);
					break;

				case 8:
					// TX2: Cyan
					fprintf(fd,"%c%c%c",0,255,255);
					break;

				case 9:
					// TX1 + TX2: Yellow
					fprintf(fd,"%c%c%c",255,255,0);
					break;

				case 16:
					// TX3: Medium Violet
					fprintf(fd,"%c%c%c",147,112,219);
					break;

				case 17:
					// TX1 + TX3: Pink
					fprintf(fd,"%c%c%c",255,192,203);
					break;

				case 24:
					// TX2 + TX3: Orange
					fprintf(fd,"%c%c%c",255,165,0);
					break;

				case 25:
					// TX1 + TX2 + TX3: Dark Green
					fprintf(fd,"%c%c%c",0,100,0);
					break;

				case 32:
					// TX4: Sienna 1
					fprintf(fd,"%c%c%c",255,130,71);
					break;

				case 33:
					// TX1 + TX4: Green Yellow
					fprintf(fd,"%c%c%c",173,255,47);
					break;

				case 40:
					// TX2 + TX4: Dark Sea Green 1
					fprintf(fd,"%c%c%c",193,255,193);
					break;

				case 41:
					// TX1 + TX2 + TX4: Blanched Almond
					fprintf(fd,"%c%c%c",255,235,205);
					break;

				case 48:
					// TX3 + TX4: Dark Turquoise
					fprintf(fd,"%c%c%c",0,206,209);
					break;

				case 49:
					// TX1 + TX3 + TX4: Medium Spring Green
					fprintf(fd,"%c%c%c",0,250,154);
					break;

				case 56:
					// TX2 + TX3 + TX4: Tan
					fprintf(fd,"%c%c%c",210,180,140);
					break;

				case 57:
					// TX1 + TX2 + TX3 + TX4: Gold2
					fprintf(fd,"%c%c%c",238,201,0);
					break;

				default:
					if (ngs)
					{						// No terrainGreyLevel
						fprintf(fd,"%c%c%c",255,255,255);
					}
					else
					{
						// Sea-level: Medium Blue
						if (isPointSeaLevel(indx, x0, y0))
						fprintf(fd,"%c%c%c",0,0,170);
						else
						{
							// Elevation: Greyscale
							SetTerrainGreyLevel(indx, x0, y0);
							fprintf(fd,"%c%c%c",terrainGreyLevel,terrainGreyLevel,terrainGreyLevel);
						}
					}
				}
			}

			else
			{
				// We should never get here, but if we do, display the region as black

				fprintf(fd,"%c%c%c",0,0,0);
			}
		}
	}
	fclose(fd);
	pmsg( stdout, (sdf_done) );
	fflush(stdout);
}		// void WritePPM()

void WritePPMLR(char *filename, unsigned char geo, unsigned char kml, unsigned char ngs, site_data *xmtr, unsigned char txsites)
{
	// This function generates a topographic map in Portable Pix Map
	// (PPM) format based on the content of flags held in the mask[][]
	// array (only).  The image created is rotated counter-clockwise
	// 90 degrees from its representation in dem[][] so that north
	// points up and east points right in the image generated.

	char mapfile[255];
	char geofile[255];
	char kmlfile[255];
	char ckfile[255];
	unsigned width;
	unsigned height;
	unsigned red;
	unsigned green;
	unsigned blue;
	unsigned char found;
	unsigned char mask;
	unsigned char cityorcounty;
	int indx;
	int x;
	int y;
	int z;
	int colorwidth;
	int x0;
	int y0;
	int loss;
	int level;
	int hundreds;
	int tens;
	int units;
	int match;
	double lat;
	double lon;
	double north;
	double south;
	double east;
	double west;
	double minLon;
	FILE *fd;

	IniGreyLevelPoints();	// initialize the grey level points

	width=NPointsLongitude();
	height=NPointsLatitude();

	LoadLossColors(xmtr[0]);

	if (filename[0]==0)
	{
		strncpy(filename, xmtr[0].filename,254);
		filename[strlen(filename)-4]=0;  // Remove .qth
	}

	y=strlen(filename);

	if (y>240)
	y=240;


	if (y>4)
	{
		if (filename[y-1]=='m' && filename[y-2]=='p' && filename[y-3]=='p' && filename[y-4]=='.')
		y-=4;
	}

	for (x=0; x<y; x++)
	{
		mapfile[x]=filename[x];
		geofile[x]=filename[x];
		kmlfile[x]=filename[x];
		ckfile[x]=filename[x];
	}

	mapfile[x]='.';
	geofile[x]='.';
	kmlfile[x]='.';
	mapfile[x+1]='p';
	geofile[x+1]='g';
	kmlfile[x+1]='k';
	mapfile[x+2]='p';
	geofile[x+2]='e';
	kmlfile[x+2]='m';
	mapfile[x+3]='m';
	geofile[x+3]='o';
	kmlfile[x+3]='l';
	mapfile[x+4]=0;
	geofile[x+4]=0;
	kmlfile[x+4]=0;

	ckfile[x]='-';
	ckfile[x+1]='c';
	ckfile[x+2]='k';
	ckfile[x+3]='.';
	ckfile[x+4]='p';
	ckfile[x+5]='p';
	ckfile[x+6]='m';
	ckfile[x+7]=0;

	minLon=((double)min_lon)+DegreesPerPoint;

	if (minLon>360.0)
	minLon-=360.0;

	north=(double)max_lat-DegreesPerPoint;

	if (kml || geo)
	south=(double)min_lat;	// No bottom legend
	else
	south=(double)min_lat-(30.0/ppd); // 30 n. points for bottom legend

	east=(minLon<180.0?-minLon:360.0-min_lon);
	west=(double)(max_lon<180?-max_lon:360-max_lon);

	if (kml==0 && geo)
	{
		fd=fopenFullPath(geofile,"wb");

		fprintf(fd,"FILENAME\t%s\n",mapfile);
		fprintf(fd,"#\t\tX\tY\tLong\t\tLat\n");
		fprintf(fd,"TIEPOINT\t0\t0\t%.3f\t\t%.3f\n",west,north);

		fprintf(fd,"TIEPOINT\t%u\t%u\t%.3f\t\t%.3f\n",width-1,height-1,east,south);
		fprintf(fd,"IMAGESIZE\t%u\t%u\n",width,height);

		fprintf(fd,"#\n# Auto Generated by %s v%s\n#\n",program_name,program_version);

		fclose(fd);
	}

	if (kml && geo==0)
	{
		fd=fopenFullPath(kmlfile,"wb");

		fprintf(fd,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		fprintf(fd,"<kml xmlns=\"http://earth.google.com/kml/2.1\">\n");
		fprintf(fd,"<!-- Generated by %s Version %s -->\n",program_name,program_version);
		fprintf(fd,"  <Folder>\n");
		fprintf(fd,"   <name>%s</name>\n",program_name);
		fprintf(fd,"     <description>%s Transmitter Path Loss Overlay</description>\n",xmtr[0].name);
		fprintf(fd,"       <GroundOverlay>\n");
		fprintf(fd,"         <name>rfprobe Path Loss Overlay</name>\n");
		fprintf(fd,"           <description>rfprobe Coverage</description>\n");
		fprintf(fd,"		<Icon>\n");
		fprintf(fd,"              <href>%s</href>\n",mapfile);
		fprintf(fd,"		</Icon>\n");
		// fprintf(fd,"            <opacity>128</opacity>\n");
		fprintf(fd,"            <LatLonBox>\n");
		fprintf(fd,"               <north>%.5f</north>\n",north);
		fprintf(fd,"               <south>%.5f</south>\n",south);
		fprintf(fd,"               <east>%.5f</east>\n",east);
		fprintf(fd,"               <west>%.5f</west>\n",west);
		fprintf(fd,"               <rotation>0.0</rotation>\n");
		fprintf(fd,"            </LatLonBox>\n");
		fprintf(fd,"       </GroundOverlay>\n");
		fprintf(fd,"       <ScreenOverlay>\n");
		fprintf(fd,"          <name>Color Key</name>\n");
		fprintf(fd,"		<description>Contour Color Key</description>\n");
		fprintf(fd,"          <Icon>\n");
		fprintf(fd,"            <href>%s</href>\n",ckfile);
		fprintf(fd,"          </Icon>\n");
		fprintf(fd,"          <overlayXY x=\"0\" y=\"1\" xunits=\"fraction\" yunits=\"fraction\"/>\n");
		fprintf(fd,"          <screenXY x=\"0\" y=\"1\" xunits=\"fraction\" yunits=\"fraction\"/>\n");
		fprintf(fd,"          <rotationXY x=\"0\" y=\"0\" xunits=\"fraction\" yunits=\"fraction\"/>\n");
		fprintf(fd,"          <size x=\"0\" y=\"0\" xunits=\"fraction\" yunits=\"fraction\"/>\n");
		fprintf(fd,"       </ScreenOverlay>\n");

		for (x=0; x<txsites; x++)
		{
			fprintf(fd,"     <Placemark>\n");
			fprintf(fd,"       <name>%s</name>\n",xmtr[x].name);
			fprintf(fd,"       <visibility>1</visibility>\n");
			fprintf(fd,"       <Style>\n");
			fprintf(fd,"       <IconStyle>\n");
			fprintf(fd,"        <Icon>\n");
			fprintf(fd,"          <href>root://icons/palette-5.png</href>\n");
			fprintf(fd,"          <x>224</x>\n");
			fprintf(fd,"          <y>224</y>\n");
			fprintf(fd,"          <w>32</w>\n");
			fprintf(fd,"          <h>32</h>\n");
			fprintf(fd,"        </Icon>\n");
			fprintf(fd,"       </IconStyle>\n");
			fprintf(fd,"       </Style>\n");
			fprintf(fd,"      <Point>\n");
			fprintf(fd,"        <extrude>1</extrude>\n");
			fprintf(fd,"        <altitudeMode>relativeToGround</altitudeMode>\n");
			fprintf(fd,"        <coordinates>%f,%f,%f</coordinates>\n",(xmtr[x].lon<180.0?-xmtr[x].lon:360.0-xmtr[x].lon), xmtr[x].lat, xmtr[x].alt);
			fprintf(fd,"      </Point>\n");
			fprintf(fd,"     </Placemark>\n");
		}

		fprintf(fd,"  </Folder>\n");
		fprintf(fd,"</kml>\n");

		fclose(fd);
	}

	fd=fopenFullPath(mapfile,"wb");

	if (kml || geo)
	{
		// No bottom legend

		fprintf(fd,"P6\n%u %u\n255\n",width,height);
		fprintf(stdout, "\n");
		pmsg( stdout, (writing_pixmap_image), mapfile,width,height);
	}

	else
	{
		// Allow space for bottom legend

		fprintf(fd,"P6\n%u %u\n255\n",width,height+30);
		fprintf(stdout, "\n");
		pmsg( stdout, (writing_pixmap_image), mapfile,width,height+30);
	}

	fflush(stdout);

	for (y=0, lat=north; y<(int)height; y++, lat=north-(DegreesPerPoint*(double)y))
	{
		for (x=0, lon=max_lon; x<(int)width; x++, lon=SET_LONGITUDE_TODOWN(x) )
		{
			NormalLongitude(lon);

			// SEARCH_POINTMAP( lat, lon );
			found = SearchPointMap( lat, lon , indx, x0, y0);
			if (found)
			{
				mask=dem[indx].mask[x0][y0];
				loss=(dem[indx].signal[x0][y0]);
				cityorcounty=0;

				match=255;

				red=0;
				green=0;
				blue=0;

				if (loss<=region.level[0])
				match=0;
				else
				{
					for (z=1; (z<region.levels && match==255); z++)
					{
						if (loss>=region.level[z-1] && loss<region.level[z])
						match=z;
					}
				}

				if (match<region.levels)
				{
					if (smooth_contours && match>0)
					{
						red=(unsigned)interpolate(region.color[match-1][0],region.color[match][0],region.level[match-1],region.level[match],loss);
						green=(unsigned)interpolate(region.color[match-1][1],region.color[match][1],region.level[match-1],region.level[match],loss);
						blue=(unsigned)interpolate(region.color[match-1][2],region.color[match][2],region.level[match-1],region.level[match],loss);
					}

					else
					{
						red=region.color[match][0];
						green=region.color[match][1];
						blue=region.color[match][2];
					}
				}

				if (mask&2)
				{
					// Text Labels: Red or otherwise

					if (red>=180 && green<=75 && blue<=75 && loss!=0)
					fprintf(fd,"%c%c%c",255^red,255^green,255^blue);
					else
					fprintf(fd,"%c%c%c",255,0,0);

					cityorcounty=1;
				}

				else if (mask&4)
				{
					// County Boundaries: Black

					fprintf(fd,"%c%c%c",0,0,0);

					cityorcounty=1;
				}

				if (cityorcounty==0)
				{
					if (loss==0 || (contour_threshold!=0 && loss>abs(contour_threshold)))
					{
						if (ngs)  // No terrainGreyLevel
						fprintf(fd,"%c%c%c",255,255,255);
						else
						{
							// Display land or sea elevation

							if (isPointSeaLevel(indx, x0, y0))
							fprintf(fd,"%c%c%c",0,0,170);
							else
							{
								SetTerrainGreyLevel(indx, x0, y0);
								fprintf(fd,"%c%c%c",terrainGreyLevel,terrainGreyLevel,terrainGreyLevel);
							}
						}
					}

					else
					{
						// Plot path loss in color

						if (red!=0 || green!=0 || blue!=0)
						fprintf(fd,"%c%c%c",red,green,blue);

						else  // terrainGreyLevel / sea-level
						{
							if (isPointSeaLevel(indx, x0, y0))
							fprintf(fd,"%c%c%c",0,0,170);
							else
							{
								// Elevation: Greyscale
								SetTerrainGreyLevel(indx, x0, y0);
								fprintf(fd,"%c%c%c",terrainGreyLevel,terrainGreyLevel,terrainGreyLevel);
							}
						}
					}
				}
			}

			else
			{
				// We should never get here, but if we do, display the region as black

				fprintf(fd,"%c%c%c",0,0,0);
			}
		}
	}

	if (kml==0 && geo==0)
	{
		// Display legend along bottom of image if not generating .kml or .geo output.

		colorwidth=(int)rint((float)width/(float)region.levels);

		for (y0=0; y0<30; y0++)
		{
			for (x0=0; x0<(int)width; x0++)
			{
				indx=x0/colorwidth;
				x=x0%colorwidth;
				level=region.level[indx];

				hundreds=level/100;

				if (hundreds>0)
				level-=(hundreds*100);

				tens=level/10;

				if (tens>0)
				level-=(tens*10);

				units=level;

				if (y0>=8 && y0<=23)
				{
					if (hundreds>0)
					{
						if (x>=11 && x<=18)
						if (fontdata[16*(hundreds+'0')+(y0-8)]&(128>>(x-11)))
						indx=255;
					}

					if (tens>0 || hundreds>0)
					{
						if (x>=19 && x<=26)
						if (fontdata[16*(tens+'0')+(y0-8)]&(128>>(x-19)))
						indx=255;
					}

					if (x>=27 && x<=34)
					if (fontdata[16*(units+'0')+(y0-8)]&(128>>(x-27)))
					indx=255;

					if (x>='*' && x<=49)
					if (fontdata[16*('d')+(y0-8)]&(128>>(x-42)))
					indx=255;

					if (x>=50 && x<=57)
					if (fontdata[16*('B')+(y0-8)]&(128>>(x-50)))
					indx=255;
				}

				if (indx>region.levels)
				fprintf(fd,"%c%c%c",0,0,0);
				else
				{
					red=region.color[indx][0];
					green=region.color[indx][1];
					blue=region.color[indx][2];

					fprintf(fd,"%c%c%c",red,green,blue);
				}
			}
		}
	}

	fclose(fd);


	if (kml)
	{
		// Write colorkey image file

		fd=fopenFullPath(ckfile,"wb");

		height=30*region.levels;
		width=100;

		fprintf(fd,"P6\n%u %u\n255\n",width,height);

		for (y0=0; y0<(int)height; y0++)
		{
			for (x0=0; x0<(int)width; x0++)
			{
				indx=y0/30;
				x=x0;
				level=region.level[indx];

				hundreds=level/100;

				if (hundreds>0)
				level-=(hundreds*100);

				tens=level/10;

				if (tens>0)
				level-=(tens*10);

				units=level;

				if ((y0%30)>=8 && (y0%30)<=23)
				{
					if (hundreds>0)
					{
						if (x>=11 && x<=18)
						if (fontdata[16*(hundreds+'0')+((y0%30)-8)]&(128>>(x-11)))
						indx=255;
					}

					if (tens>0 || hundreds>0)
					{
						if (x>=19 && x<=26)
						if (fontdata[16*(tens+'0')+((y0%30)-8)]&(128>>(x-19)))
						indx=255;
					}

					if (x>=27 && x<=34)
					if (fontdata[16*(units+'0')+((y0%30)-8)]&(128>>(x-27)))
					indx=255;

					if (x>='*' && x<=49)
					if (fontdata[16*('d')+((y0%30)-8)]&(128>>(x-42)))
					indx=255;

					if (x>=50 && x<=57)
					if (fontdata[16*('B')+((y0%30)-8)]&(128>>(x-50)))
					indx=255;
				}

				if (indx>region.levels)
				fprintf(fd,"%c%c%c",0,0,0);
				else
				{
					red=region.color[indx][0];
					green=region.color[indx][1];
					blue=region.color[indx][2];

					fprintf(fd,"%c%c%c",red,green,blue);
				}
			}
		}

		fclose(fd);
	}

	pmsg( stdout, (sdf_done) );
	fflush(stdout);
}

void WritePPMSS(char *filename, unsigned char geo, unsigned char kml, unsigned char ngs, site_data *xmtr, unsigned char txsites)
{
	// This function generates a topographic map in Portable Pix Map
	// (PPM) format based on the signal strength values held in the
	// signal[][] array.  The image created is rotated counter-clockwise
	// 90 degrees from its representation in dem[][] so that north
	// points up and east points right in the image generated.

	char mapfile[255];
	char geofile[255];
	char kmlfile[255];
	char ckfile[255];
	unsigned width;
	unsigned height;
	unsigned red;
	unsigned green;
	unsigned blue;
	unsigned char found;
	unsigned char mask;
	unsigned char cityorcounty;
	int indx;
	int x;
	int y;
	int z=1;
	int x0;
	int y0;
	int signal;
	int level;
	int hundreds;
	int tens;
	int units;
	int match;
	int colorwidth;
	double lat;
	double lon;
	double north;
	double south;
	double east;
	double west;
	double minLon;
	FILE *fd;

	IniGreyLevelPoints();	// initialize the grey level points

	width=NPointsLongitude();
	height=NPointsLatitude();

	LoadSignalColors(xmtr[0]);

	if (filename[0]==0)
	{
		strncpy(filename, xmtr[0].filename,254);
		filename[strlen(filename)-4]=0;  // Remove .qth
	}

	y=strlen(filename);

	if (y>240)
	y=240;

	if (y>4)
	{
		if (filename[y-1]=='m' && filename[y-2]=='p' && filename[y-3]=='p' && filename[y-4]=='.')
		y-=4;
	}

	for (x=0; x<y; x++)
	{
		mapfile[x]=filename[x];
		geofile[x]=filename[x];
		kmlfile[x]=filename[x];
		ckfile[x]=filename[x];
	}

	mapfile[x]='.';
	geofile[x]='.';
	kmlfile[x]='.';
	mapfile[x+1]='p';
	geofile[x+1]='g';
	kmlfile[x+1]='k';
	mapfile[x+2]='p';
	geofile[x+2]='e';
	kmlfile[x+2]='m';
	mapfile[x+3]='m';
	geofile[x+3]='o';
	kmlfile[x+3]='l';
	mapfile[x+4]=0;
	geofile[x+4]=0;
	kmlfile[x+4]=0;

	ckfile[x]='-';
	ckfile[x+1]='c';
	ckfile[x+2]='k';
	ckfile[x+3]='.';
	ckfile[x+4]='p';
	ckfile[x+5]='p';
	ckfile[x+6]='m';
	ckfile[x+7]=0;

	minLon=((double)min_lon)+DegreesPerPoint;

	if (minLon>360.0)
		minLon-=360.0;

	north=(double)max_lat-DegreesPerPoint;

	if (kml || geo)
	south=(double)min_lat;	// No bottom legend
	else
	south=(double)min_lat-(30.0/ppd);	// 30 n. points for bottom legend

	east=(minLon<180.0?-minLon:360.0-min_lon);
	west=(double)(max_lon<180?-max_lon:360-max_lon);

	if (geo && kml==0)
	{
		fd=fopenFullPath(geofile,"wb");

		fprintf(fd,"FILENAME\t%s\n",mapfile);
		fprintf(fd,"#\t\tX\tY\tLong\t\tLat\n");
		fprintf(fd,"TIEPOINT\t0\t0\t%.3f\t\t%.3f\n",west,north);

		fprintf(fd,"TIEPOINT\t%u\t%u\t%.3f\t\t%.3f\n",width-1,height-1,east,south);
		fprintf(fd,"IMAGESIZE\t%u\t%u\n",width,height);

		fprintf(fd,"#\n# Auto Generated by %s v%s\n#\n",program_name,program_version);

		fclose(fd);
	}

	if (kml && geo==0)
	{
		fd=fopenFullPath(kmlfile,"wb");

		fprintf(fd,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		fprintf(fd,"<kml xmlns=\"http://earth.google.com/kml/2.1\">\n");
		fprintf(fd,"<!-- Generated by %s Version %s -->\n",program_name,program_version);
		fprintf(fd,"  <Folder>\n");
		fprintf(fd,"   <name>%s</name>\n",program_name);
		fprintf(fd,"     <description>%s Transmitter Contours</description>\n",xmtr[0].name);
		fprintf(fd,"       <GroundOverlay>\n");
		fprintf(fd,"         <name>rfprobe Signal Strength Contours</name>\n");
		fprintf(fd,"           <description>rfprobe Coverage</description>\n");
		fprintf(fd,"		<Icon>\n");
		fprintf(fd,"              <href>%s</href>\n",mapfile);
		fprintf(fd,"		</Icon>\n");
		// fprintf(fd,"            <opacity>128</opacity>\n");
		fprintf(fd,"            <LatLonBox>\n");
		fprintf(fd,"               <north>%.5f</north>\n",north);
		fprintf(fd,"               <south>%.5f</south>\n",south);
		fprintf(fd,"               <east>%.5f</east>\n",east);
		fprintf(fd,"               <west>%.5f</west>\n",west);
		fprintf(fd,"               <rotation>0.0</rotation>\n");
		fprintf(fd,"            </LatLonBox>\n");
		fprintf(fd,"       </GroundOverlay>\n");
		fprintf(fd,"       <ScreenOverlay>\n");
		fprintf(fd,"          <name>Color Key</name>\n");
		fprintf(fd,"            <description>Contour Color Key</description>\n");
		fprintf(fd,"          <Icon>\n");
		fprintf(fd,"            <href>%s</href>\n",ckfile);
		fprintf(fd,"          </Icon>\n");
		fprintf(fd,"          <overlayXY x=\"0\" y=\"1\" xunits=\"fraction\" yunits=\"fraction\"/>\n");
		fprintf(fd,"          <screenXY x=\"0\" y=\"1\" xunits=\"fraction\" yunits=\"fraction\"/>\n");
		fprintf(fd,"          <rotationXY x=\"0\" y=\"0\" xunits=\"fraction\" yunits=\"fraction\"/>\n");
		fprintf(fd,"          <size x=\"0\" y=\"0\" xunits=\"fraction\" yunits=\"fraction\"/>\n");
		fprintf(fd,"       </ScreenOverlay>\n");

		for (x=0; x<txsites; x++)
		{
			fprintf(fd,"     <Placemark>\n");
			fprintf(fd,"       <name>%s</name>\n",xmtr[x].name);
			fprintf(fd,"       <visibility>1</visibility>\n");
			fprintf(fd,"       <Style>\n");
			fprintf(fd,"       <IconStyle>\n");
			fprintf(fd,"        <Icon>\n");
			fprintf(fd,"          <href>root://icons/palette-5.png</href>\n");
			fprintf(fd,"          <x>224</x>\n");
			fprintf(fd,"          <y>224</y>\n");
			fprintf(fd,"          <w>32</w>\n");
			fprintf(fd,"          <h>32</h>\n");
			fprintf(fd,"        </Icon>\n");
			fprintf(fd,"       </IconStyle>\n");
			fprintf(fd,"       </Style>\n");
			fprintf(fd,"      <Point>\n");
			fprintf(fd,"        <extrude>1</extrude>\n");
			fprintf(fd,"        <altitudeMode>relativeToGround</altitudeMode>\n");
			fprintf(fd,"        <coordinates>%f,%f,%f</coordinates>\n",(xmtr[x].lon<180.0?-xmtr[x].lon:360.0-xmtr[x].lon), xmtr[x].lat, xmtr[x].alt);
			fprintf(fd,"      </Point>\n");
			fprintf(fd,"     </Placemark>\n");
		}

		fprintf(fd,"  </Folder>\n");
		fprintf(fd,"</kml>\n");

		fclose(fd);
	}

	fd=fopenFullPath(mapfile,"wb");

	if (kml || geo)
	{
		// No bottom legend

		fprintf(fd,"P6\n%u %u\n255\n",width,height);
		fprintf(stdout, "\n");
		pmsg( stdout, (writing_pixmap_image), mapfile,width,height);
	}

	else
	{
		// Allow space for bottom legend

		fprintf(fd,"P6\n%u %u\n255\n",width,height+30);
		fprintf(stdout, "\n");
		pmsg( stdout, (writing_pixmap_image), mapfile,width,height+30);
	}

	fflush(stdout);

	for (y=0, lat=north; y<(int)height; y++, lat=north-(DegreesPerPoint*(double)y))
	{
		for (x=0, lon=max_lon; x<(int)width; x++, lon=SET_LONGITUDE_TODOWN(x) )
		{
			NormalLongitude(lon);

			// SEARCH_POINTMAP( lat, lon );
			found = SearchPointMap( lat, lon , indx, x0, y0);
			if (found)
			{
				mask=dem[indx].mask[x0][y0];
				signal=(dem[indx].signal[x0][y0])-100;
				cityorcounty=0;

				match=255;

				red=0;
				green=0;
				blue=0;

				if (signal>=region.level[0])
				match=0;
				else
				{
					for (z=1; (z<region.levels && match==255); z++)
					{
						if (signal<region.level[z-1] && signal>=region.level[z])
						match=z;
					}
				}

				if (match<region.levels)
				{
					if (smooth_contours && match>0)
					{
						red=(unsigned)interpolate(region.color[match][0],region.color[match-1][0],region.level[match],region.level[match-1],signal);
						green=(unsigned)interpolate(region.color[match][1],region.color[match-1][1],region.level[match],region.level[match-1],signal);
						blue=(unsigned)interpolate(region.color[match][2],region.color[match-1][2],region.level[match],region.level[match-1],signal);
					}

					else
					{
						red=region.color[match][0];
						green=region.color[match][1];
						blue=region.color[match][2];
					}
				}

				if (mask&2)
				{
					// Text Labels: Red or otherwise

					if (red>=180 && green<=75 && blue<=75)
					fprintf(fd,"%c%c%c",255^red,255^green,255^blue);
					else
					fprintf(fd,"%c%c%c",255,0,0);

					cityorcounty=1;
				}

				else if (mask&4)
				{
					// County Boundaries: Black

					fprintf(fd,"%c%c%c",0,0,0);

					cityorcounty=1;
				}

				if (cityorcounty==0)
				{
					if (contour_threshold!=0 && signal<contour_threshold)
					{
						if (ngs)
						fprintf(fd,"%c%c%c",255,255,255);
						else
						{
							// Display land or sea elevation

							if (isPointSeaLevel(indx, x0, y0))
							fprintf(fd,"%c%c%c",0,0,170);
							else
							{
								SetTerrainGreyLevel(indx, x0, y0);
								fprintf(fd,"%c%c%c",terrainGreyLevel,terrainGreyLevel,terrainGreyLevel);
							}
						}
					}

					else
					{
						// Plot field strength regions in color

						if (red!=0 || green!=0 || blue!=0)
						fprintf(fd,"%c%c%c",red,green,blue);

						else  // terrainGreyLevel / sea-level
						{
							if (ngs)
							fprintf(fd,"%c%c%c",255,255,255);
							else
							{
								if (isPointSeaLevel(indx, x0, y0))
								fprintf(fd,"%c%c%c",0,0,170);
								else
								{
									// Elevation: Greyscale
									SetTerrainGreyLevel(indx, x0, y0);
									fprintf(fd,"%c%c%c",terrainGreyLevel,terrainGreyLevel,terrainGreyLevel);
								}
							}
						}
					}
				}
			}

			else
			{
				// We should never get here, but if we do, display the region as black

				fprintf(fd,"%c%c%c",0,0,0);
			}
		}
	}

	if (kml==0 && geo==0)
	{
		// Display legend along bottom of image if not generating .kml or .geo output.
		//

		colorwidth=(int)rint((float)width/(float)region.levels);

		for (y0=0; y0<30; y0++)
		{
			for (x0=0; x0<(int)width; x0++)
			{
				indx=x0/colorwidth;
				x=x0%colorwidth;
				level=region.level[indx];

				hundreds=level/100;

				if (hundreds>0)
				level-=(hundreds*100);

				tens=level/10;

				if (tens>0)
				level-=(tens*10);

				units=level;

				if (y0>=8 && y0<=23)
				{
					if (hundreds>0)
					{
						if (x>=5 && x<=12)
						if (fontdata[16*(hundreds+'0')+(y0-8)]&(128>>(x-5)))
						indx=255;
					}

					if (tens>0 || hundreds>0)
					{
						if (x>=13 && x<=20)
						if (fontdata[16*(tens+'0')+(y0-8)]&(128>>(x-13)))
						indx=255;
					}

					if (x>=21 && x<=28)
					if (fontdata[16*(units+'0')+(y0-8)]&(128>>(x-21)))
					indx=255;

					if (x>=36 && x<=43)
					if (fontdata[16*('d')+(y0-8)]&(128>>(x-36)))
					indx=255;

					if (x>=44 && x<=51)
					if (fontdata[16*('B')+(y0-8)]&(128>>(x-44)))
					indx=255;

					if (x>=52 && x<=59)
					if (fontdata[16*(230)+(y0-8)]&(128>>(x-52)))
					indx=255;

					if (x>=60 && x<=67)
					if (fontdata[16*('V')+(y0-8)]&(128>>(x-60)))
					indx=255;

					if (x>=68 && x<=75)
					if (fontdata[16*('/')+(y0-8)]&(128>>(x-68)))
					indx=255;

					if (x>=76 && x<=83)
					if (fontdata[16*('m')+(y0-8)]&(128>>(x-76)))
					indx=255;
				}

				if (indx>region.levels)
				fprintf(fd,"%c%c%c",0,0,0);
				else
				{
					red=region.color[indx][0];
					green=region.color[indx][1];
					blue=region.color[indx][2];

					fprintf(fd,"%c%c%c",red,green,blue);
				}
			}
		}
	}

	fclose(fd);

	if (kml)
	{
		// Write colorkey image file

		fd=fopenFullPath(ckfile,"wb");

		height=30*region.levels;
		width=100;

		fprintf(fd,"P6\n%u %u\n255\n",width,height);

		for (y0=0; y0<(int)height; y0++)
		{
			for (x0=0; x0<(int)width; x0++)
			{
				indx=y0/30;
				x=x0;
				level=region.level[indx];

				hundreds=level/100;

				if (hundreds>0)
				level-=(hundreds*100);

				tens=level/10;

				if (tens>0)
				level-=(tens*10);

				units=level;

				if ((y0%30)>=8 && (y0%30)<=23)
				{
					if (hundreds>0)
					{
						if (x>=5 && x<=12)
						if (fontdata[16*(hundreds+'0')+((y0%30)-8)]&(128>>(x-5)))
						indx=255;
					}

					if (tens>0 || hundreds>0)
					{
						if (x>=13 && x<=20)
						if (fontdata[16*(tens+'0')+((y0%30)-8)]&(128>>(x-13)))
						indx=255;
					}

					if (x>=21 && x<=28)
					if (fontdata[16*(units+'0')+((y0%30)-8)]&(128>>(x-21)))
					indx=255;

					if (x>=36 && x<=43)
					if (fontdata[16*('d')+((y0%30)-8)]&(128>>(x-36)))
					indx=255;

					if (x>=44 && x<=51)
					if (fontdata[16*('B')+((y0%30)-8)]&(128>>(x-44)))
					indx=255;

					if (x>=52 && x<=59)
					if (fontdata[16*(230)+((y0%30)-8)]&(128>>(x-52)))
					indx=255;

					if (x>=60 && x<=67)
					if (fontdata[16*('V')+((y0%30)-8)]&(128>>(x-60)))
					indx=255;

					if (x>=68 && x<=75)
					if (fontdata[16*('/')+((y0%30)-8)]&(128>>(x-68)))
					indx=255;

					if (x>=76 && x<=83)
					if (fontdata[16*('m')+((y0%30)-8)]&(128>>(x-76)))
					indx=255;
				}

				if (indx>region.levels)
				fprintf(fd,"%c%c%c",0,0,0);
				else
				{
					red=region.color[indx][0];
					green=region.color[indx][1];
					blue=region.color[indx][2];

					fprintf(fd,"%c%c%c",red,green,blue);
				}
			}
		}

		fclose(fd);
	}

	pmsg( stdout, (sdf_done) );
	fflush(stdout);
}

void WritePPMDBM(char *filename, unsigned char geo, unsigned char kml, unsigned char ngs, site_data *xmtr, unsigned char txsites)
{
	// This function generates a topographic map in Portable Pix Map
	// (PPM) format based on the signal power level values held in the
	// signal[][] array.  The image created is rotated counter-clockwise
	// 90 degrees from its representation in dem[][] so that north
	// points up and east points right in the image generated.

	char mapfile[255];
	char geofile[255];
	char kmlfile[255];
	char ckfile[255];
	unsigned width;
	unsigned height;
	unsigned red;
	unsigned green;
	unsigned blue;
	unsigned char found;
	unsigned char mask;
	unsigned char cityorcounty;
	int indx;
	int x;
	int y;
	int z=1;
	int x0;
	int y0;
	int dBm;
	int level;
	int hundreds;
	int tens;
	int units;
	int match;
	int colorwidth;
	double lat;
	double lon;
	double north;
	double south;
	double east;
	double west;
	double minLon;
	FILE *fd;

	IniGreyLevelPoints();	// initialize the grey level points

	width=NPointsLongitude();
	height=NPointsLatitude();

	LoadDBMColors(xmtr[0]);

	if (filename[0]==0)
	{
		strncpy(filename, xmtr[0].filename,254);
		filename[strlen(filename)-4]=0;  // Remove .qth
	}

	y=strlen(filename);

	if (y>240)
	y=240;

	if (y>4)
	{
		if (filename[y-1]=='m' && filename[y-2]=='p' && filename[y-3]=='p' && filename[y-4]=='.')
		y-=4;
	}

	for (x=0; x<y; x++)
	{
		mapfile[x]=filename[x];
		geofile[x]=filename[x];
		kmlfile[x]=filename[x];
		ckfile[x]=filename[x];
	}

	mapfile[x]='.';
	geofile[x]='.';
	kmlfile[x]='.';
	mapfile[x+1]='p';
	geofile[x+1]='g';
	kmlfile[x+1]='k';
	mapfile[x+2]='p';
	geofile[x+2]='e';
	kmlfile[x+2]='m';
	mapfile[x+3]='m';
	geofile[x+3]='o';
	kmlfile[x+3]='l';
	mapfile[x+4]=0;
	geofile[x+4]=0;
	kmlfile[x+4]=0;

	ckfile[x]='-';
	ckfile[x+1]='c';
	ckfile[x+2]='k';
	ckfile[x+3]='.';
	ckfile[x+4]='p';
	ckfile[x+5]='p';
	ckfile[x+6]='m';
	ckfile[x+7]=0;

	minLon=((double)min_lon)+DegreesPerPoint;

	if (minLon>360.0)
		minLon-=360.0;

	north=(double)max_lat-DegreesPerPoint;

	if (kml || geo)
	south=(double)min_lat;	// No bottom legend
	else
	south=(double)min_lat-(30.0/ppd);	// 30 n. points for bottom legend

	east=(minLon<180.0?-minLon:360.0-min_lon);
	west=(double)(max_lon<180?-max_lon:360-max_lon);

	if (geo && kml==0)
	{
		fd=fopenFullPath(geofile,"wb");

		fprintf(fd,"FILENAME\t%s\n",mapfile);
		fprintf(fd,"#\t\tX\tY\tLong\t\tLat\n");
		fprintf(fd,"TIEPOINT\t0\t0\t%.3f\t\t%.3f\n",west,north);

		fprintf(fd,"TIEPOINT\t%u\t%u\t%.3f\t\t%.3f\n",width-1,height-1,east,south);
		fprintf(fd,"IMAGESIZE\t%u\t%u\n",width,height);

		fprintf(fd,"#\n# Auto Generated by %s v%s\n#\n",program_name,program_version);

		fclose(fd);
	}

	if (kml && geo==0)
	{
		fd=fopenFullPath(kmlfile,"wb");

		fprintf(fd,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		fprintf(fd,"<kml xmlns=\"http://earth.google.com/kml/2.1\">\n");
		fprintf(fd,"<!-- Generated by %s Version %s -->\n",program_name,program_version);
		fprintf(fd,"  <Folder>\n");
		fprintf(fd,"   <name>%s</name>\n",program_name);
		fprintf(fd,"     <description>%s Transmitter Contours</description>\n",xmtr[0].name);
		fprintf(fd,"       <GroundOverlay>\n");
		fprintf(fd,"         <name>rfprobe Signal Power Level Contours</name>\n");
		fprintf(fd,"           <description>rfprobe Coverage</description>\n");
		fprintf(fd,"		<Icon>\n");
		fprintf(fd,"              <href>%s</href>\n",mapfile);
		fprintf(fd,"		</Icon>\n");
		// fprintf(fd,"            <opacity>128</opacity>\n");
		fprintf(fd,"            <LatLonBox>\n");
		fprintf(fd,"               <north>%.5f</north>\n",north);
		fprintf(fd,"               <south>%.5f</south>\n",south);
		fprintf(fd,"               <east>%.5f</east>\n",east);
		fprintf(fd,"               <west>%.5f</west>\n",west);
		fprintf(fd,"               <rotation>0.0</rotation>\n");
		fprintf(fd,"            </LatLonBox>\n");
		fprintf(fd,"       </GroundOverlay>\n");
		fprintf(fd,"       <ScreenOverlay>\n");
		fprintf(fd,"          <name>Color Key</name>\n");
		fprintf(fd,"            <description>Contour Color Key</description>\n");
		fprintf(fd,"          <Icon>\n");
		fprintf(fd,"            <href>%s</href>\n",ckfile);
		fprintf(fd,"          </Icon>\n");
		fprintf(fd,"          <overlayXY x=\"0\" y=\"1\" xunits=\"fraction\" yunits=\"fraction\"/>\n");
		fprintf(fd,"          <screenXY x=\"0\" y=\"1\" xunits=\"fraction\" yunits=\"fraction\"/>\n");
		fprintf(fd,"          <rotationXY x=\"0\" y=\"0\" xunits=\"fraction\" yunits=\"fraction\"/>\n");
		fprintf(fd,"          <size x=\"0\" y=\"0\" xunits=\"fraction\" yunits=\"fraction\"/>\n");
		fprintf(fd,"       </ScreenOverlay>\n");

		for (x=0; x<txsites; x++)
		{
			fprintf(fd,"     <Placemark>\n");
			fprintf(fd,"       <name>%s</name>\n",xmtr[x].name);
			fprintf(fd,"       <visibility>1</visibility>\n");
			fprintf(fd,"       <Style>\n");
			fprintf(fd,"       <IconStyle>\n");
			fprintf(fd,"        <Icon>\n");
			fprintf(fd,"          <href>root://icons/palette-5.png</href>\n");
			fprintf(fd,"          <x>224</x>\n");
			fprintf(fd,"          <y>224</y>\n");
			fprintf(fd,"          <w>32</w>\n");
			fprintf(fd,"          <h>32</h>\n");
			fprintf(fd,"        </Icon>\n");
			fprintf(fd,"       </IconStyle>\n");
			fprintf(fd,"       </Style>\n");
			fprintf(fd,"      <Point>\n");
			fprintf(fd,"        <extrude>1</extrude>\n");
			fprintf(fd,"        <altitudeMode>relativeToGround</altitudeMode>\n");
			fprintf(fd,"        <coordinates>%f,%f,%f</coordinates>\n",FMT_LONGITUDE(xmtr[x].lon), xmtr[x].lat, xmtr[x].alt);
			fprintf(fd,"      </Point>\n");
			fprintf(fd,"     </Placemark>\n");
		}

		fprintf(fd,"  </Folder>\n");
		fprintf(fd,"</kml>\n");

		fclose(fd);
	}

	fd=fopenFullPath(mapfile,"wb");

	if (kml || geo)
	{
		// No bottom legend

		fprintf(fd,"P6\n%u %u\n255\n",width,height);
		fprintf(stdout, "\n");
		pmsg( stdout, (writing_pixmap_image), mapfile,width,height);
	}

	else
	{
		// Allow for bottom legend

		fprintf(fd,"P6\n%u %u\n255\n",width,height+30);
		fprintf(stdout, "\n");
		pmsg( stdout, (writing_pixmap_image), mapfile,width,height+30);
	}

	fflush(stdout);

	for (y=0, lat=north; y<(int)height; y++, lat=north-(DegreesPerPoint*(double)y))
	{
		for (x=0, lon=max_lon; x<(int)width; x++, lon=SET_LONGITUDE_TODOWN(x) )
		{
			NormalLongitude(lon);

			// SEARCH_POINTMAP( lat, lon );
			found = SearchPointMap( lat, lon , indx, x0, y0);
			if (found)
			{
				mask=dem[indx].mask[x0][y0];
				dBm=(dem[indx].signal[x0][y0])-200;
				cityorcounty=0;

				match=255;

				red=0;
				green=0;
				blue=0;

				if (dBm>=region.level[0])
				match=0;
				else
				{
					for (z=1; (z<region.levels && match==255); z++)
					{
						if (dBm<region.level[z-1] && dBm>=region.level[z])
						match=z;
					}
				}

				if (match<region.levels)
				{
					if (smooth_contours && match>0)
					{
						red=(unsigned)interpolate(region.color[match][0],region.color[match-1][0],region.level[match],region.level[match-1],dBm);
						green=(unsigned)interpolate(region.color[match][1],region.color[match-1][1],region.level[match],region.level[match-1],dBm);
						blue=(unsigned)interpolate(region.color[match][2],region.color[match-1][2],region.level[match],region.level[match-1],dBm);
					}

					else
					{
						red=region.color[match][0];
						green=region.color[match][1];
						blue=region.color[match][2];
					}
				}

				if (mask&2)
				{
					// Text Labels: Red or otherwise

					if (red>=180 && green<=75 && blue<=75 && dBm!=0)
					fprintf(fd,"%c%c%c",255^red,255^green,255^blue);
					else
					fprintf(fd,"%c%c%c",255,0,0);

					cityorcounty=1;
				}

				else if (mask&4)
				{
					// County Boundaries: Black

					fprintf(fd,"%c%c%c",0,0,0);

					cityorcounty=1;
				}

				if (cityorcounty==0)
				{
					if (contour_threshold!=0 && dBm<contour_threshold)
					{
						if (ngs) // No terrainGreyLevel
						fprintf(fd,"%c%c%c",255,255,255);
						else
						{
							// Display land or sea elevation

							if (isPointSeaLevel(indx, x0, y0))
							fprintf(fd,"%c%c%c",0,0,170);
							else
							{
								SetTerrainGreyLevel(indx, x0, y0);
								fprintf(fd,"%c%c%c",terrainGreyLevel,terrainGreyLevel,terrainGreyLevel);
							}
						}
					}

					else
					{
						// Plot signal power level regions in color

						if (red!=0 || green!=0 || blue!=0)
						fprintf(fd,"%c%c%c",red,green,blue);

						else  // terrainGreyLevel / sea-level
						{
							if (ngs)
							fprintf(fd,"%c%c%c",255,255,255);
							else
							{
								if (isPointSeaLevel(indx, x0, y0))
								fprintf(fd,"%c%c%c",0,0,170);
								else
								{
									// Elevation: Greyscale
									SetTerrainGreyLevel(indx, x0, y0);
									fprintf(fd,"%c%c%c",terrainGreyLevel,terrainGreyLevel,terrainGreyLevel);
								}
							}
						}
					}
				}
			}

			else
			{
				// We should never get here, but if
				// we do, display the region as black

				fprintf(fd,"%c%c%c",0,0,0);
			}
		}
	}

	if (kml==0 && geo==0)
	{
		// Display legend along bottom of image
		// if not generating .kml or .geo output.

		colorwidth=(int)rint((float)width/(float)region.levels);

		for (y0=0; y0<30; y0++)
		{
			for (x0=0; x0<(int)width; x0++)
			{
				indx=x0/colorwidth;
				x=x0%colorwidth;

				level=abs(region.level[indx]);

				hundreds=level/100;

				if (hundreds>0)
				level-=(hundreds*100);

				tens=level/10;

				if (tens>0)
				level-=(tens*10);

				units=level;

				if (y0>=8 && y0<=23)
				{
					if (hundreds>0)
					{
						if (region.level[indx]<0)
						{
							if (x>=5 && x<=12)
							if (fontdata[16*('-')+(y0-8)]&(128>>(x-5)))
							indx=255;
						}

						else
						{
							if (x>=5 && x<=12)
							if (fontdata[16*('+')+(y0-8)]&(128>>(x-5)))
							indx=255;
						}

						if (x>=13 && x<=20)
						if (fontdata[16*(hundreds+'0')+(y0-8)]&(128>>(x-13)))
						indx=255;
					}

					if (tens>0 || hundreds>0)
					{
						if (hundreds==0)
						{
							if (region.level[indx]<0)
							{
								if (x>=13 && x<=20)
								if (fontdata[16*('-')+(y0-8)]&(128>>(x-13)))
								indx=255;
							}

							else
							{
								if (x>=13 && x<=20)
								if (fontdata[16*('+')+(y0-8)]&(128>>(x-13)))
								indx=255;
							}
						}

						if (x>=21 && x<=28)
						if (fontdata[16*(tens+'0')+(y0-8)]&(128>>(x-21)))
						indx=255;
					}

					if (hundreds==0 && tens==0)
					{
						if (region.level[indx]<0)
						{
							if (x>=21 && x<=28)
							if (fontdata[16*('-')+(y0-8)]&(128>>(x-21)))
							indx=255;
						}

						else
						{
							if (x>=21 && x<=28)
							if (fontdata[16*('+')+(y0-8)]&(128>>(x-21)))
							indx=255;
						}
					}

					if (x>=29 && x<=36)
					if (fontdata[16*(units+'0')+(y0-8)]&(128>>(x-29)))
					indx=255;

					if (x>=37 && x<=44)
					if (fontdata[16*('d')+(y0-8)]&(128>>(x-37)))
					indx=255;

					if (x>=45 && x<=52)
					if (fontdata[16*('B')+(y0-8)]&(128>>(x-45)))
					indx=255;

					if (x>=53 && x<=60)
					if (fontdata[16*('m')+(y0-8)]&(128>>(x-53)))
					indx=255;
				}

				if (indx>region.levels)
				fprintf(fd,"%c%c%c",0,0,0);
				else
				{
					red=region.color[indx][0];
					green=region.color[indx][1];
					blue=region.color[indx][2];

					fprintf(fd,"%c%c%c",red,green,blue);
				}
			}
		}
	}

	fclose(fd);


	if (kml)
	{
		// Write colorkey image file

		fd=fopenFullPath(ckfile,"wb");

		height=30*region.levels;
		width=100;

		fprintf(fd,"P6\n%u %u\n255\n",width,height);

		for (y0=0; y0<(int)height; y0++)
		{
			for (x0=0; x0<(int)width; x0++)
			{
				indx=y0/30;
				x=x0;

				level=abs(region.level[indx]);

				hundreds=level/100;

				if (hundreds>0)
				level-=(hundreds*100);

				tens=level/10;

				if (tens>0)
				level-=(tens*10);

				units=level;


				if ((y0%30)>=8 && (y0%30)<=23)
				{
					if (hundreds>0)
					{
						if (region.level[indx]<0)
						{
							if (x>=5 && x<=12)
							if (fontdata[16*('-')+((y0%30)-8)]&(128>>(x-5)))
							indx=255;
						}

						else
						{
							if (x>=5 && x<=12)
							if (fontdata[16*('+')+((y0%30)-8)]&(128>>(x-5)))
							indx=255;
						}

						if (x>=13 && x<=20)
						if (fontdata[16*(hundreds+'0')+((y0%30)-8)]&(128>>(x-13)))
						indx=255;
					}

					if (tens>0 || hundreds>0)
					{
						if (hundreds==0)
						{
							if (region.level[indx]<0)
							{
								if (x>=13 && x<=20)
								if (fontdata[16*('-')+((y0%30)-8)]&(128>>(x-13)))
								indx=255;
							}

							else
							{
								if (x>=13 && x<=20)
								if (fontdata[16*('+')+((y0%30)-8)]&(128>>(x-13)))
								indx=255;
							}
						}

						if (x>=21 && x<=28)
						if (fontdata[16*(tens+'0')+((y0%30)-8)]&(128>>(x-21)))
						indx=255;
					}

					if (hundreds==0 && tens==0)
					{
						if (region.level[indx]<0)
						{
							if (x>=21 && x<=28)
							if (fontdata[16*('-')+((y0%30)-8)]&(128>>(x-21)))
							indx=255;
						}

						else
						{
							if (x>=21 && x<=28)
							if (fontdata[16*('+')+((y0%30)-8)]&(128>>(x-21)))
							indx=255;
						}
					}

					if (x>=29 && x<=36)
					if (fontdata[16*(units+'0')+((y0%30)-8)]&(128>>(x-29)))
					indx=255;

					if (x>=37 && x<=44)
					if (fontdata[16*('d')+((y0%30)-8)]&(128>>(x-37)))
					indx=255;

					if (x>=45 && x<=52)
					if (fontdata[16*('B')+((y0%30)-8)]&(128>>(x-45)))
					indx=255;

					if (x>=53 && x<=60)
					if (fontdata[16*('m')+((y0%30)-8)]&(128>>(x-53)))
					indx=255;
				}

				if (indx>region.levels)
				fprintf(fd,"%c%c%c",0,0,0);

				else
				{
					red=region.color[indx][0];
					green=region.color[indx][1];
					blue=region.color[indx][2];

					fprintf(fd,"%c%c%c",red,green,blue);
				}
			}
		}

		fclose(fd);
	}

	pmsg( stdout, (sdf_done) );
	fflush(stdout);
}

void GraphTerrain(site_data source, site_data destination, char *name)
{
	// This function invokes gnuplot to generate an appropriate
	// output file indicating the terrain profile between the source
	// and destination locations when the -p command line option
	// is used.  "basename" is the name assigned to the output
	// file generated by gnuplot.  The filename extension is used
	// to set gnuplot's terminal setting and output file type.
	// If no extension is found, .png is assumed.

	int	x;
	int y;
	int z;
	char basename[255];
	char term[30];
	char ext[15];
	double minheight=100000.0;
	double maxheight=-100000.0;
	FILE *fd=NULL;
	FILE *fClutter=NULL;

	ReadPath(destination,source);

	fd=fopen(fn_profile_gp,"wb");

	if (clutter>0.0)
	fClutter=fopen(fn_clutter_gp,"wb");

	for (x=0; x<path.length; x++)
	{
		if ((path.elevation[x]+clutter)>maxheight)
		maxheight=path.elevation[x]+clutter;

		if (path.elevation[x]<minheight)
		minheight=path.elevation[x];

		if (metric)
		{
			fprintf(fd,"%f\t%f\n",mi2Km(path.distance[x]),fo2Meters(path.elevation[x]));

			if (fClutter!=NULL && x>0 && x<path.length-2)
			fprintf(fClutter,"%f\t%f\n",mi2Km(path.distance[x]),fo2Meters(path.elevation[x]==0.0?path.elevation[x]:(path.elevation[x]+clutter)));
		}

		else
		{
			fprintf(fd,"%f\t%f\n",path.distance[x],path.elevation[x]);

			if (fClutter!=NULL && x>0 && x<path.length-2)
			fprintf(fClutter,"%f\t%f\n",path.distance[x],(path.elevation[x]==0.0?path.elevation[x]:(path.elevation[x]+clutter)));
		}
	}

	fclose(fd);

	if (fClutter!=NULL)
	fclose(fClutter);

	if (name[0]=='.')
	{
		// Default filename and output file type

		strncpy(basename,"profile\0",8);
		strncpy(term,"png\0",4);
		strncpy(ext,"png\0",4);
	}

	else
	{
		// Extract extension and terminal type from "name"

		ext[0]=0;
		y=strlen(name);
		strncpy(basename,name,254);

		for (x=y-1; x>0 && name[x]!='.'; x--);

		if (x>0)  // Extension found
		{
			for (z=x+1; z<=y && (z-(x+1))<10; z++)
			{
				ext[z-(x+1)]=tolower(name[z]);
				term[z-(x+1)]=name[z];
			}

			ext[z-(x+1)]=0;  // Ensure an ending 0
			term[z-(x+1)]=0;
			basename[x]=0;
		}

		if (ext[0]==0)	// No extension -- Default is png
		{
			strncpy(term,"png\0",4);
			strncpy(ext,"png\0",4);
		}
	}

	// Either .ps or .postscript may be used as an extension for postscript output.

	if (strncmp(term,"postscript",10)==0)
	strncpy(ext,"ps\0",3);

	else if (strncmp(ext,"ps",2)==0)
	strncpy(term,"postscript enhanced color\0",26);

	if (maxheight<1.0)
	{
		maxheight=1.0;	// Avoid a gnuplot y-range error
		minheight=-1.0;	// over a completely sea-level path
	}

	else
	minheight-=(0.01*maxheight);

	fd=fopen(fn_splat_gp,"w");
	fprintf(fd,"set grid\n");
	fprintf(fd,"set yrange [%2.3f to %2.3f]\n", metric?fo2Meters(minheight):minheight, metric?fo2Meters(maxheight):maxheight);
	fprintf(fd,"set encoding iso_8859_1\n");
	fprintf(fd,"set term %s truecolor size %i,%i\n",term, 1280, 768);
	fprintf(fd,"set title \"%s Terrain Profile Between %s and %s (%.2f%c magnetic azimuth)\"\n",program_name,destination.name, source.name, AzimuthSites(destination,source),176);

	if (metric)
	{
		fprintf(fd,"set xlabel \"From %s to %s : %.2f km\"\n",destination.name,source.name,mi2Km(Distance(source,destination)));
		fprintf(fd,"set ylabel \"Elevation: above sea level (meters)\"\n");
	}

	else
	{
		fprintf(fd,"set xlabel \"From %s to %s : %.2f miles\"\n",destination.name,source.name,Distance(source,destination));
		fprintf(fd,"set ylabel \"Elevation: above sea level (feet)\"\n");
	}

	fprintf(fd,"set output \"%s/%s.%s\"\n",PathCurrDir,basename,ext);

	if (clutter>0.0)
	{
		if (metric)
		{
			fprintf(fd,"plot ");
			fprintf(fd,"\"%s\" title \"Terrain Profile\"with lines lw 2, ", fn_profile_gp);
			fprintf(fd,"\"%s\" title \"Clutter Profile (%.2f meters)\" with lines\n" ,fn_clutter_gp,fo2Meters(clutter));
		}
		else
		{
			fprintf(fd,"plot ");
			fprintf(fd,"\"%s\" title \"Terrain Profile\"with lines lw 2, ", fn_profile_gp);
			fprintf(fd,"\"%s\" title \"Clutter Profile (%.2f feet)\" with lines\n" ,fn_clutter_gp,clutter);
		}
	}

	else
	{
		fprintf(fd,"plot \"%s/profile.gp\" title \"\" with lines\n",PathCurrDir);
	}

	fclose(fd);

	x=GenerateGraphGnuplot();

	if (x!=-1)
	{
		if (gpsav==0)
		{
			unlink(fn_splat_gp);
			unlink(fn_profile_gp);
		}

		fprintf(stdout, "Terrain plot written to: \"%s.%s\"\n",basename,ext);
		fflush(stdout);
	}

	else
	fprintf(stderr,"\n*** ERROR: Error occurred invoking gnuplot!\n");
}

void GraphElevation(site_data source, site_data destination, char *name)
{
	// This function invokes gnuplot to generate an appropriate
	// output file indicating the terrain elevation profile between
	// the source and destination locations when the -e command line
	// option is used.  "basename" is the name assigned to the output
	// file generated by gnuplot.  The filename extension is used
	// to set gnuplot's terminal setting and output file type.
	// If no extension is found, .png is assumed.

	int	x;
	int y;
	int z;
	char basename[255];
	char term[30];
	char ext[15];
	double angle;
	double clutter_angle=0.0;
	double refangle;
	double maxangle=-90.0;
	double minangle=90.0;
	double distance;
	site_data remote=ZeroSiteData;
	site_data remote2;
	FILE *fProfile=NULL;
	FILE *fClutter=NULL;
	FILE *fReference=NULL;

	ReadPath(destination,source);  // destination=RX, source=TX
	refangle=ElevationAngle(destination,source);
	distance=Distance(source,destination);

	fProfile=fopen(fn_profile_gp,"wb");

	if (clutter>0.0)
	fClutter=fopen(fn_clutter_gp,"wb");

	fReference=fopen(fn_reference_gp,"wb");

	for (x=1; x<path.length-1; x++)
	{
		remote.lat=path.lat[x];
		remote.lon=path.lon[x];
		remote.alt=0.0;
		angle=ElevationAngle(destination,remote);

		if (clutter>0.0)
		{
			remote2.lat=path.lat[x];
			remote2.lon=path.lon[x];

			if (path.elevation[x]!=0.0)
			remote2.alt=clutter;
			else
			remote2.alt=0.0;

			clutter_angle=ElevationAngle(destination,remote2);
		}

		if (metric)
		{
			fprintf(fProfile,"%f\t%f\n",mi2Km(path.distance[x]),angle);

			if (fClutter!=NULL)
			fprintf(fClutter,"%f\t%f\n",mi2Km(path.distance[x]),clutter_angle);

			fprintf(fReference,"%f\t%f\n",mi2Km(path.distance[x]),refangle);
		}

		else
		{
			fprintf(fProfile,"%f\t%f\n",path.distance[x],angle);

			if (fClutter!=NULL)
			fprintf(fClutter,"%f\t%f\n",path.distance[x],clutter_angle);

			fprintf(fReference,"%f\t%f\n",path.distance[x],refangle);
		}

		if (angle>maxangle)
		maxangle=angle;

		if (clutter_angle>maxangle)
		maxangle=clutter_angle;

		if (angle<minangle)
		minangle=angle;
	}

	if (metric)
	{
		fprintf(fProfile,"%f\t%f\n",mi2Km(path.distance[path.length-1]),refangle);
		fprintf(fReference,"%f\t%f\n",mi2Km(path.distance[path.length-1]),refangle);
	}

	else
	{
		fprintf(fProfile,"%f\t%f\n",path.distance[path.length-1],refangle);
		fprintf(fReference,"%f\t%f\n",path.distance[path.length-1],refangle);
	}

	fclose(fProfile);

	if (fClutter!=NULL)
	fclose(fClutter);

	fclose(fReference);

	if (name[0]=='.')
	{
		// Default filename and output file type

		strncpy(basename,"profile\0",8);
		strncpy(term,"png\0",4);
		strncpy(ext,"png\0",4);
	}

	else
	{
		// Extract extension and terminal type from "name"

		ext[0]=0;
		y=strlen(name);
		strncpy(basename,name,254);

		for (x=y-1; x>0 && name[x]!='.'; x--);

		if (x>0)  // Extension found
		{
			for (z=x+1; z<=y && (z-(x+1))<10; z++)
			{
				ext[z-(x+1)]=tolower(name[z]);
				term[z-(x+1)]=name[z];
			}

			ext[z-(x+1)]=0;  // Ensure an ending 0
			term[z-(x+1)]=0;
			basename[x]=0;
		}

		if (ext[0]==0)	// No extension -- Default is png
		{
			strncpy(term,"png\0",4);
			strncpy(ext,"png\0",4);
		}
	}

	// Either .ps or .postscript may be used as an extension for postscript output.

	if (strncmp(term,"postscript",10)==0)
	strncpy(ext,"ps\0",3);

	else if (strncmp(ext,"ps",2)==0)
	strncpy(term,"postscript enhanced color\0",26);

	fProfile=fopen(fn_splat_gp,"w");

	fprintf(fProfile,"set grid\n");

	if (distance>2.0)
	fprintf(fProfile,"set yrange [%2.3f to %2.3f]\n", (-fabs(refangle)-0.25), maxangle+0.25);
	else
	fprintf(fProfile,"set yrange [%2.3f to %2.3f]\n", minangle, refangle+(-minangle/8.0));

	fprintf(fProfile,"set encoding iso_8859_1\n");
	// fprintf(fProfile,"set term %s\n",term);
	fprintf(fProfile,"set term %s truecolor size %i,%i\n",term, 1280, 768);
	fprintf(fProfile,"set title \"%s Elevation Profile Between %s and %s (%.2f%c magnetic azimuth)\"\n",program_name,destination.name,source.name,AzimuthSites(destination,source),176);

	if (metric)
	fprintf(fProfile,"set xlabel \"From %s to %s : %.2f km\"\n",destination.name,source.name,mi2Km(distance));
	else
	fprintf(fProfile,"set xlabel \"From %s to %s : %.2f miles\"\n",destination.name,source.name,distance);


	fprintf(fProfile,"set ylabel \"Elevation Angle Along LOS Path Between\\n%s and %s (degrees)\"\n",destination.name,source.name);
	fprintf(fProfile,"set output \"%s/%s.%s\"\n",PathCurrDir,basename,ext);

	if (clutter>0.0)
	{
		if (metric)
		{
			fprintf(fProfile,"plot ");
			fprintf(fProfile,"\"%s\" title \"Real Earth Profile\"with lines lw 2, ", fn_profile_gp);
			fprintf(fProfile,"\"%s\" title \"Clutter Profile (%.2f meters)\"with lines lw 2, " ,fn_clutter_gp,fo2Meters(clutter));
			fprintf(fProfile,"\"%s\" title \"Optical Line of Sight Path (%.2f%c elevation)\" with lines\n", fn_reference_gp,refangle,176);
		}
		else
		{
			fprintf(fProfile,"plot ");
			fprintf(fProfile,"\"%s\" title \"Real Earth Profile\"with lines lw 2, ", fn_profile_gp);
			fprintf(fProfile,"\"%s\" title \"Clutter Profile (%.2f feet)\"with lines lw 2, " ,fn_clutter_gp,clutter);
			fprintf(fProfile,"\"%s\" title \"Optical Line of Sight Path (%.2f%c elevation)\" with lines\n", fn_reference_gp,refangle,176);
		}
	}

	else
	{
		fprintf(fProfile,"plot ");
		fprintf(fProfile,"\"%s\" title \"Real Earth Profile\"with lines lw 2, ", fn_profile_gp);
		fprintf(fProfile,"\"%s\" title \"Optical Line of Sight Path (%.2f%c elevation)\" with lines\n", fn_reference_gp,refangle,176);
	}

	fclose(fProfile);

	x=GenerateGraphGnuplot();

	if (x!=-1)
	{
		if (gpsav==0)
		{
			unlink(fn_splat_gp);
			unlink(fn_profile_gp);
			unlink(fn_reference_gp);

			if (clutter>0.0)
			unlink(fn_clutter_gp);
		}

		fprintf(stdout, "Elevation plot written to: \"%s.%s\"\n",basename,ext);
		fflush(stdout);
	}

	else
	fprintf(stderr,"\n*** ERROR: Error occurred invoking gnuplot!\n");
}

//---------------------------------------------------------
// original elevation graph
// only lines
void Original_GenerateElevationGraph(
	site_data TxSite,
	site_data RxSite,
	char *name,
	unsigned char fresnel_plot,
	unsigned char normalized
	)
{
	// This function invokes gnuplot to generate an appropriate
	// output file indicating the terrain height profile between
	// the source and RxSite locations referenced to the
	// line-of-sight path between the receive and transmit sites
	// when the -h or -H command line option is used.  "basename"
	// is the name assigned to the output file generated by gnuplot.
	// The filename extension is used to set gnuplot's terminal
	// setting and output file type.  If no extension is found,
	// .png is assumed.

	int	x;
	int y;
	int z;

	char basename[255];
	char term[30];
	char ext[15];

	double a;
	double b;
	double c;
	double height=0.0;
	double refangle;
	double cangle;
	double maxheight=-100000.0;
	double minheight=100000.0;
	double WaveLength=0.0;
	double f_zone=0.0;
	double fpt6_zone=0.0;
	double nm=0.0;
	double nb=0.0;
	double ed=0.0;
	double es=0.0;
	double r=0.0;
	double d=0.0;
	double d1=0.0;
	double terrain;
	double azimuth;
	double distance;
	double dheight=0.0;
	double minterrain=100000.0;
	double minearth=100000.0;
	double miny;
	double maxy;
	double min2y;
	double max2y;

	site_data remote=ZeroSiteData;
	FILE *fProfile=NULL;
	FILE *fClutter=NULL;
	FILE *fReference=NULL;
	FILE *fFresnel=NULL;
	FILE *fFresnel_pt_6=NULL;
	FILE *fCurvature=NULL;

	ReadPath(RxSite,TxSite);  // RxSite=RX, source=TX
	azimuth=AzimuthSites(RxSite,TxSite);
	distance=Distance(RxSite,TxSite);
	refangle=ElevationAngle(RxSite,TxSite);
	b=GetElevation(RxSite)+RxSite.alt+earthRadiusFoot;

	// Wavelength and path distance (great circle) in feet.

	if (fresnel_plot)
	{
		WaveLength=SpeedLightAir_foot_sec/(LR.frq_mhz*1e6);
		d=mi2Foot(path.distance[path.length-1]);
	}

	if (normalized)
	{
		ed=GetElevation(RxSite);
		es=GetElevation(TxSite);
		nb=-RxSite.alt-ed;
		nm=(-TxSite.alt-es-nb)/(path.distance[path.length-1]);
	}

	fProfile=fopen(fn_profile_gp,"wb");

	if (clutter>0.0)
	fClutter=fopen(fn_clutter_gp,"wb");

	fReference=fopen(fn_reference_gp,"wb");
	fCurvature=fopen(fn_curvature_gp, "wb");

	if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
	{
		fFresnel=fopen(fn_fresnel_gp, "wb");
		fFresnel_pt_6=fopen(fn_fresnel_pt_6_gp, "wb");
	}

	for (x=0; x<path.length-1; x++)
	{
		remote.lat=path.lat[x];
		remote.lon=path.lon[x];
		remote.alt=0.0;

		terrain=GetElevation(remote);

		if (x==0)
		{
			terrain+=RxSite.alt;  				// RX antenna spike
		}

		a=terrain+earthRadiusFoot;
		cangle=mi2Foot(Distance(RxSite,remote))/earthRadiusFoot;
		c=b*sin(RADIANS(refangle)+HALFPI)/sin(HALFPI-RADIANS(refangle)-cangle);

		height=a-c;

		// Per Fink and Christiansen, Electronics
		// Engineers' Handbook, 1989:
		//   H = sqrt(lamba * d1 * (d - d1)/d)
		// where H is the distance from the LOS
		// path to the first Fresnel zone boundary.
		//

		if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
		{
			d1=mi2Foot( path.distance[x] );
			f_zone=-1.0*sqrt(WaveLength*d1*(d-d1)/d);
			fpt6_zone=f_zone*fzone_clearance;
		}

		if (normalized)
		{
			r=-(nm*path.distance[x])-nb;
			height+=r;

			if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
			{
				f_zone+=r;
				fpt6_zone+=r;
			}
		}

		else
		r=0.0;

		if (metric)
		{
			fprintf(fProfile,"%f\t%f\n",mi2Km(path.distance[x]),fo2Meters(height));

			if (fClutter!=NULL && x>0 && x<path.length-2)
			fprintf(fClutter,"%f\t%f\n",mi2Km(path.distance[x]),fo2Meters(terrain==0.0?height:(height+clutter)));

			fprintf(fReference,"%f\t%f\n",mi2Km(path.distance[x]),fo2Meters(r));
			fprintf(fCurvature,"%f\t%f\n",mi2Km(path.distance[x]),fo2Meters(height-terrain));
		}

		else
		{
			fprintf(fProfile,"%f\t%f\n",path.distance[x],height);

			if (fClutter!=NULL && x>0 && x<path.length-2)
			fprintf(fClutter,"%f\t%f\n",path.distance[x],(terrain==0.0?height:(height+clutter)));

			fprintf(fReference,"%f\t%f\n",path.distance[x],r);
			fprintf(fCurvature,"%f\t%f\n",path.distance[x],height-terrain);
		}

		if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
		{
			if (metric)
			{
				fprintf(fFresnel,"%f\t%f\n",mi2Km(path.distance[x]),fo2Meters(f_zone));
				fprintf(fFresnel_pt_6,"%f\t%f\n",mi2Km(path.distance[x]),fo2Meters(fpt6_zone));
			}

			else
			{
				fprintf(fFresnel,"%f\t%f\n",path.distance[x],f_zone);
				fprintf(fFresnel_pt_6,"%f\t%f\n",path.distance[x],fpt6_zone);
			}

			if (f_zone<minheight)
			minheight=f_zone;
		}

		if ((height+clutter)>maxheight)
		maxheight=height+clutter;

		if (height<minheight)
		minheight=height;

		if (r>maxheight)
		maxheight=r;

		if (terrain<minterrain)
		minterrain=terrain;

		if ((height-terrain)<minearth)
		minearth=height-terrain;
	}

	if (normalized)
	r=-(nm*path.distance[path.length-1])-nb;
	else
	r=0.0;

	if (metric)
	{
		fprintf(fProfile,"%f\t%f\n",mi2Km(path.distance[path.length-1]),fo2Meters(r));
		fprintf(fReference,"%f\t%f\n",mi2Km(path.distance[path.length-1]),fo2Meters(r));
	}

	else
	{
		fprintf(fProfile,"%f\t%f\n",path.distance[path.length-1],r);
		fprintf(fReference,"%f\t%f\n",path.distance[path.length-1],r);
	}

	if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
	{
		if (metric)
		{
			fprintf(fFresnel,"%f\t%f\n",mi2Km(path.distance[path.length-1]),fo2Meters(r));
			fprintf(fFresnel_pt_6,"%f\t%f\n",mi2Km(path.distance[path.length-1]),fo2Meters(r));
		}

		else
		{
			fprintf(fFresnel,"%f\t%f\n",path.distance[path.length-1],r);
			fprintf(fFresnel_pt_6,"%f\t%f\n",path.distance[path.length-1],r);
		}
	}

	if (r>maxheight)
	{
		maxheight=r;
	}

	if (r<minheight)
	{
		minheight=r;
	}

	fclose(fProfile);

	if (fClutter!=NULL)
	{
		fclose(fClutter);
	}

	fclose(fReference);
	fclose(fCurvature);

	if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
	{
		fclose(fFresnel);
		fclose(fFresnel_pt_6);
	}

	if (name[0]=='.')
	{
		// Default filename and output file type

		strncpy(basename,"profile\0",8);
		strncpy(term,"png\0",4);
		strncpy(ext,"png\0",4);
	}

	else
	{
		// Extract extension and terminal type from "name"

		ext[0]=0;
		y=strlen(name);
		strncpy(basename,name,254);

		for (x=y-1; x>0 && name[x]!='.'; x--);

		if (x>0)  // Extension found
		{
			for (z=x+1; z<=y && (z-(x+1))<10; z++)
			{
				ext[z-(x+1)]=tolower(name[z]);
				term[z-(x+1)]=name[z];
			}

			ext[z-(x+1)]=0;  // Ensure an ending 0
			term[z-(x+1)]=0;
			basename[x]=0;
		}

		if (ext[0]==0)	// No extension -- Default is png
		{
			strncpy(term,"png\0",4);
			strncpy(ext,"png\0",4);
		}
	}

	// Either .ps or .postscript may be used as an extension for postscript output.

	if (strncmp(term,"postscript",10)==0)
	{
		strncpy(ext,"ps\0",3);
	}
	else if (strncmp(ext,"ps",2)==0)
	{
		strncpy(term,"postscript enhanced color\0",26);
	}

	fProfile=fopen(fn_splat_gp,"w");

	dheight=maxheight-minheight;
	miny=minheight-0.15*dheight;
	maxy=maxheight+0.05*dheight;

	if (maxy<20.0)
	{
		maxy=20.0;
	}

	dheight=maxheight-minheight;
	min2y=miny-minterrain+0.05*dheight;

	if (minearth<min2y)
	{
		miny-=min2y-minearth+0.05*dheight;
		min2y=minearth-0.05*dheight;
	}

	max2y=min2y+maxy-miny;

	fprintf(fProfile,"set grid\n");
	fprintf(fProfile,"set yrange [%2.3f to %2.3f]\n", metric?fo2Meters(miny):miny, metric?fo2Meters(maxy):maxy);
	fprintf(fProfile,"set y2range [%2.3f to %2.3f]\n", metric?fo2Meters(min2y):min2y, metric?fo2Meters(max2y):max2y);
	fprintf(fProfile,"set xrange [-0.5 to %2.3f]\n",metric?mi2Km(rint(distance+0.5)):rint(distance+0.5));
	fprintf(fProfile,"set encoding iso_8859_1\n");
	// fprintf(fProfile,"set term %s\n",term);
	fprintf(fProfile,"set term %s truecolor size %i,%i\n",term, 1280, 768);

	if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
	{
		fprintf(fProfile,"set title \"Profiles Between %s and %s (%.2f%c magnetic azimuth)\\nat %.0f MHz for K=%.3f\"\n",
					RxSite.name, TxSite.name, azimuth,176,
					forced_freq, (earthRadiusFoot/EARTHRADIUS_FOOT) );
	}

	else
	{
		fprintf(fProfile,"set title \"%s Height Profile Between %s and %s (%.2f%c magnetic azimuth)\"\n",program_name, RxSite.name, TxSite.name, azimuth,176);
	}

	if (metric)
	{
		fprintf(fProfile,"set xlabel \"From %s to %s : %.2f km\"\n",RxSite.name,TxSite.name,mi2Km(Distance(TxSite,RxSite)));
	}
	else
	{
		fprintf(fProfile,"set xlabel \"From %s to %s : %.2f miles\"\n",RxSite.name,TxSite.name,Distance(TxSite,RxSite));
	}

	if (normalized)
	{
		if (metric)
		{
			fprintf(fProfile,"set ylabel \"Height in meters referenced to Radio Line of Sight between\\n%s and %s\"\n",RxSite.name,TxSite.name);
		}
		else
		{
			fprintf(fProfile,"set ylabel \"Height in feet referenced to Radio Line of Sight between\\n%s and %s\"\n",RxSite.name,TxSite.name);
		}
	}

	else
	{
		if (metric)
		{
			fprintf(fProfile,"set ylabel \"Height Referenced To LOS Path Between\\n%s and %s (meters)\"\n",RxSite.name,TxSite.name);
		}
		else
		{
			fprintf(fProfile,"set ylabel \"Height Referenced To LOS Path Between\\n%s and %s (feet)\"\n",RxSite.name,TxSite.name);
		}
	}

	fprintf(fProfile,"set output \"%s/%s.%s\"\n",PathCurrDir,basename,ext);

	if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
	{
		if (clutter>0.0)
		{
			if (metric)
			{
				fprintf(fProfile,"plot ");
				fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\"with lines lw 2, ", fn_profile_gp);
				fprintf(fProfile,"\"%s\" title \"Ground Clutter (%.2f meters)\"with lines lw 2, " ,fn_clutter_gp,fo2Meters(clutter));
				fprintf(fProfile,"\"%s\" title \"Optical Line of Sight Path\"with lines lw 2, ", fn_reference_gp);
				fprintf(fProfile,"\"%s\" axes x1y2 title \a\"with lines lw 2, ", fn_curvature_gp);
				fprintf(fProfile,"\"%s\" axes x1y1 title \"First Fresnel Zone (%.0f MHz)\"with lines lw 2, ",fn_fresnel_gp,LR.frq_mhz);
				fprintf(fProfile,"\"%s\" title \"%.0f%% of First Fresnel Zone\" with lines\n", fn_fresnel_pt_6_gp,fzone_clearance*100.0);
			}
			else
			{
				fprintf(fProfile,"plot ");
				fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\"with lines lw 2, ", fn_profile_gp);
				fprintf(fProfile,"\"%s\" title \"Ground Clutter (%.2f feet)\"with lines lw 2, " ,fn_clutter_gp, clutter);
				fprintf(fProfile,"\"%s\" title \"Optical Line of Sight Path\"with lines lw 2, ", fn_reference_gp);
				fprintf(fProfile,"\"%s\" axes x1y2 title \"Apparent Earth's Curvature Contour\"with lines lw 2, ", fn_curvature_gp);
				fprintf(fProfile,"\"%s\" axes x1y1 title \"First Fresnel Zone (%.0f MHz)\"with lines lw 2, ",fn_fresnel_gp,LR.frq_mhz);
				fprintf(fProfile,"\"%s\" title \"%.0f%% of First Fresnel Zone\" with lines\n", fn_fresnel_pt_6_gp,fzone_clearance*100.0);
			}
		}

		else
		{
			fprintf(fProfile,"plot ");
			fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\"with lines lw 2, ", fn_profile_gp);
			fprintf(fProfile,"\"%s\" title \"Optical Line of Sight Path\"with lines lw 2, ", fn_reference_gp);
			fprintf(fProfile,"\"%s\" axes x1y2 title \"Apparent Earth Curvature Contour\"with lines lw 2, ", fn_curvature_gp);
			fprintf(fProfile,"\"%s\" axes x1y1 title \"First Fresnel Zone\" with lines lw 2, ",fn_fresnel_gp);		// ,LR.frq_mhz);
			fprintf(fProfile,"\"%s\" title \"%.0f%% of First Fresnel Zone\" with lines\n", fn_fresnel_pt_6_gp,fzone_clearance*100.0);
		}
	}

	else
	{
		if (clutter>0.0)
		{
			if (metric)
			{
				fprintf(fProfile,"plot ");
				fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\"with lines lw 2, ", fn_profile_gp);
				fprintf(fProfile,"\"%s\" title \"Ground Clutter (%.2f meters)\"with lines lw 2, " ,fn_clutter_gp,fo2Meters(clutter) );
				fprintf(fProfile,"\"%s\" title \"Optical Line of Sight Path\"with lines lw 2, ", fn_reference_gp);
				fprintf(fProfile,"\"%s\" axes x1y2 title \"Apparent Earth's Curvature Contour\" with lines\n", fn_curvature_gp);
			}
			else
			{
				fprintf(fProfile,"plot ");
				fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\"with lines lw 2, ", fn_profile_gp);
				fprintf(fProfile,"\"%s\" title \"Ground Clutter (%.2f feet)\"with lines lw 2, " ,fn_clutter_gp,clutter);
				fprintf(fProfile,"\"%s\" title \"Optical Line of Sight Path\"with lines lw 2, ", fn_reference_gp);
				fprintf(fProfile,"\"%s\" axes x1y2 title \"Apparent Earth's Curvature Contour\" with lines\n", fn_curvature_gp);
			}
		}

		else
		{
			fprintf(fProfile,"plot ");
			fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\"with lines lw 2, ", fn_profile_gp);
			fprintf(fProfile,"\"%s\" title \"Optical Line of Sight Path\"with lines lw 2, ", fn_reference_gp);
			fprintf(fProfile,"\"%s\" axes x1y2 title \"Apparent Earth's Curvature Contour\" with lines\n", fn_curvature_gp);
		}

	}

	fclose(fProfile);

	x=GenerateGraphGnuplot();

	if (x!=-1)
	{
		if (gpsav==0)
		{
			unlink(fn_splat_gp);
			unlink(fn_profile_gp);
			unlink(fn_reference_gp);
			unlink(fn_curvature_gp);

			if (fClutter!=NULL)
			{
				unlink(fn_clutter_gp);
			}

			if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
			{
				unlink(fn_fresnel_gp);
				unlink(fn_fresnel_pt_6_gp);
			}
		}

		fprintf(stdout, "\n");
		fprintf(stdout, "Height plot written to: \"%s.%s\"",basename,ext);
		fflush(stdout);
	}

	else
	fprintf(stderr,"\n*** ERROR: Error occurred invoking gnuplot!\n");
}		// end of original elevation graph


//---------------------------------------------------------
// Elevation graph (-h or -H option)
//
// new version:
// -----------------------------------

void GenerateElevationGraph(
	site_data TxSite,
	site_data RxSite,
	char *name,
	unsigned char fresnel_plot,
	unsigned char normalized
	)
{
	// This function invokes gnuplot to generate an appropriate
	// output file indicating the terrain height profile between
	// the source and GrpStartSite locations referenced to the
	// line-of-sight path between the receive and transmit sites
	// when the -h or -H command line option is used.  "basename"
	// is the name assigned to the output file generated by gnuplot.
	// The filename extension is used to set gnuplot's terminal
	// setting and output file type.  If no extension is found,
	// .png is assumed.

	int	x;
	int y;
	int z;

	char basename[255];
	char term[30];
	char ext[15];

	double a;
	double b;
	double c;
	double height=0.0;
	double refangle;
	double cangle;
	double maxheight=-100000.0;
	double minheight=100000.0;
	double WaveLength=0.0;
	double f_zone=0.0;
	double fpt6_zone=0.0;
	double nm=0.0;
	double nb=0.0;
	double ed=0.0;
	double es=0.0;
	double r=0.0;
	double d=0.0;
	double d1=0.0;
	double terrain;
	double azimuth;
	double distance;
	double dheight=0.0;
	double minterrain=100000.0;
	double minearth=100000.0;
	double miny;
	double maxy;
	double min2y;
	double max2y;
	double altPro;

	site_data remote=ZeroSiteData;
	FILE *fProfile=NULL;
	FILE *fClutter=NULL;
	FILE *fReference=NULL;

#ifdef EN_DRAW_FRESNEL
	FILE *fFresnel=NULL;
#endif

	FILE *fFresnel_pt_6=NULL;
	FILE *fCurvature=NULL;

	ReadPath(GrpStartSite,GrpEndSite);  // GrpStartSite=RX, source=TX
	azimuth=AzimuthSites(GrpStartSite,GrpEndSite);
	distance=Distance(GrpStartSite,GrpEndSite);
	refangle=ElevationAngle(GrpStartSite,GrpEndSite);
	b=GetElevation(GrpStartSite)+GrpStartSite.alt+earthRadiusFoot;

	// Wavelength and path distance (great circle) in feet.
	if (fresnel_plot)
	{
		WaveLength=SpeedLightAir_foot_sec/(LR.frq_mhz*1e6);
		d=mi2Foot(path.distance[path.length-1]);
	}
	if (normalized)
	{
		ed=GetElevation(GrpStartSite);
		es=GetElevation(GrpEndSite);
		nb=-GrpStartSite.alt-ed;
		nm=(-GrpEndSite.alt-es-nb)/(path.distance[path.length-1]);
	}

	fProfile=fopen(fn_profile_gp,"wb");
	if (clutter>0.0)
	{
		fClutter=fopen(fn_clutter_gp,"wb");
	}
	fReference=fopen(fn_reference_gp,"wb");
	fCurvature=fopen(fn_curvature_gp, "wb");

	if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
	{
#ifdef EN_DRAW_FRESNEL
		fFresnel=fopen(fn_fresnel_gp, "wb");
#endif
		fFresnel_pt_6=fopen(fn_fresnel_pt_6_gp, "wb");
	}

	for (x=0; x<path.length-1; x++)
	{
		remote.lat=path.lat[x];
		remote.lon=path.lon[x];
		remote.alt=0.0;
		terrain=GetElevation(remote);
		if (x==0)
		{
			terrain += GrpStartSite.alt;  				// RX antenna spike
		}
		a=terrain+earthRadiusFoot;
		cangle=mi2Foot(Distance(GrpStartSite,remote))/earthRadiusFoot;
		c=b*sin(RADIANS(refangle)+HALFPI)/sin(HALFPI-RADIANS(refangle)-cangle);

		height=a-c;

		// Per Fink and Christiansen, Electronics
		// Engineers' Handbook, 1989:
		//   H = sqrt(lamba * d1 * (d - d1)/d)
		// where H is the distance from the LOS
		// path to the first Fresnel zone boundary.
		//
		if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
		{
			d1=mi2Foot( path.distance[x] );
			f_zone=-1.0*sqrt(WaveLength*d1*(d-d1)/d);
			fpt6_zone=f_zone*fzone_clearance;
		}
		if (normalized)
		{
			r=-(nm*path.distance[x])-nb;
			height+=r;

			if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
			{
				f_zone+=r;
				fpt6_zone+=r;
			}
		}
		else
		{
			r=0.0;
		}
		if (metric)
		{
			altPro = fo2Meters(height);
			fprintf(fProfile,"%f\t%f\n",mi2Km(path.distance[x]),altPro);

			if (fClutter!=NULL && x>0 && x<path.length-2)
			{
				fprintf(fClutter,"%f\t%f\n", mi2Km(path.distance[x]),fo2Meters(terrain==0.0?height:(height+clutter)));
			}
			fprintf(fReference,"%f\t%f\n",mi2Km(path.distance[x]),fo2Meters(r));
			fprintf(fCurvature,"%f\t%f\n",mi2Km(path.distance[x]),fo2Meters(height-terrain));
		}

		else
		{
			altPro = height;
			fprintf(fProfile,"%f\t%f\n",path.distance[x],altPro);
			if(altPro<0.0)
			{
				altPro=0.0;
			}
			fprintf(fProfile,"%f\t%f\n",path.distance[x],altPro);

			if (fClutter!=NULL && x>0 && x<path.length-2)
			{
				fprintf(fClutter,"%f\t%f\n",path.distance[x],(terrain==0.0?height:(height+clutter)));
			}
			fprintf(fReference,"%f\t%f\n",path.distance[x],r);
			fprintf(fCurvature,"%f\t%f\n",path.distance[x],height-terrain);
		}

		if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
		{
			if (metric)
			{
#ifdef EN_DRAW_FRESNEL
				fprintf(fFresnel,"%f\t%f\n", mi2Km(path.distance[x]), fo2Meters(f_zone));
#endif
				fprintf(fFresnel_pt_6,"%f\t%f\n", mi2Km(path.distance[x]) ,fo2Meters(fpt6_zone));
			}
			else
			{
#ifdef EN_DRAW_FRESNEL
				fprintf(fFresnel,"%f\t%f\n",path.distance[x],f_zone);
#endif
				fprintf(fFresnel_pt_6,"%f\t%f\n",path.distance[x],fpt6_zone);
			}
			if (f_zone<minheight)
			{
				minheight=f_zone;
			}
		}

		if ((height+clutter)>maxheight)
		{
			maxheight=height+clutter;
		}
		if (height<minheight)
		{
			minheight=height;
		}
		if (r>maxheight)
		{
			maxheight=r;
		}
		if (terrain<minterrain)
		{
			minterrain=terrain;
		}
		if ((height-terrain)<minearth)
		{
			minearth=height-terrain;
		}
	}			// end for (x=0; x<path.length-1; x++)

	if (normalized)
	{
		r=-(nm*path.distance[path.length-1])-nb;
	}
	else
	{
		r=0.0;
	}

	if (metric)
	{
		altPro = fo2Meters(r);
		fprintf(fProfile,"%f\t%f\n", mi2Km(path.distance[path.length-1]),altPro);
		fprintf(fReference,"%f\t%f\n", mi2Km(path.distance[path.length-1]) ,fo2Meters(r));
	}

	else
	{
		altPro = fo2Meters(r);
		fprintf(fProfile,"%f\t%f\n",path.distance[path.length-1],altPro);
		if(altPro<0.0)
		{
			altPro = 0.0;
		}
		fprintf(fProfile,"%f\t%f\n",path.distance[path.length-1],altPro);
		fprintf(fReference,"%f\t%f\n",path.distance[path.length-1],r);
	}

	if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
	{
		if (metric)
		{
#ifdef EN_DRAW_FRESNEL
			fprintf(fFresnel,"%f\t%f\n", mi2Km(path.distance[path.length-1]),fo2Meters(r));
#endif
			fprintf(fFresnel_pt_6,"%f\t%f\n", mi2Km(path.distance[path.length-1]) ,fo2Meters(r));
		}
		else
		{
#ifdef EN_DRAW_FRESNEL
			fprintf(fFresnel,"%f\t%f\n",path.distance[path.length-1],r);
#endif
			fprintf(fFresnel_pt_6,"%f\t%f\n",path.distance[path.length-1],r);
		}
	}
	if (r>maxheight)
	{
		maxheight=r;
	}
	if (r<minheight)
	{
		minheight=r;
	}

	fclose(fProfile);
	if (fClutter!=NULL)
	{
		fclose(fClutter);
	}
	fclose(fReference);
	fclose(fCurvature);

	if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
	{
#ifdef EN_DRAW_FRESNEL
		fclose(fFresnel);
#endif
		fclose(fFresnel_pt_6);
	}

	if (name[0]=='.')
	{
		// Default filename and output file type

		strncpy(basename,"profile\0",8);
		strncpy(term,"png\0",4);
		strncpy(ext,"png\0",4);
	}

	else
	{
		// Extract extension and terminal type from "name"

		ext[0]=0;
		y=strlen(name);
		strncpy(basename,name,254);

		for (x=y-1; x>0 && name[x]!='.'; x--);

		if (x>0)  // Extension found
		{
			for (z=x+1; z<=y && (z-(x+1))<10; z++)
			{
				ext[z-(x+1)]=tolower(name[z]);
				term[z-(x+1)]=name[z];
			}

			ext[z-(x+1)]=0;  // Ensure an ending 0
			term[z-(x+1)]=0;
			basename[x]=0;
		}

		if (ext[0]==0)	// No extension -- Default is png
		{
			strncpy(term,"png\0",4);
			strncpy(ext,"png\0",4);
		}
	}

	// Either .ps or .postscript may be used as an extension for postscript output.

	if (strncmp(term,"postscript",10)==0)
	{
		strncpy(ext,"ps\0",3);
	}
	else if (strncmp(ext,"ps",2)==0)
	{
		strncpy(term,"postscript enhanced color\0",26);
	}

	fProfile=fopen(fn_splat_gp,"w");

	dheight=maxheight-minheight;
	miny=minheight-0.15*dheight;
	maxy=maxheight+0.05*dheight;
	if (maxy<20.0)
	{
		maxy=20.0;
	}

	dheight=maxheight-minheight;
	min2y=miny-minterrain+0.05*dheight;
	if (minearth<min2y)
	{
		miny-=min2y-minearth+0.05*dheight;
		min2y=minearth-0.05*dheight;
	}
	max2y=min2y+maxy-miny;

	// mr ---------------------------------

	fprintf(fProfile,"set style line 100 lt 2 lc rgb \"gray\" lw 2\n");
	fprintf(fProfile,"set style line 101 lt 0.5 lc rgb \"blue\" lw 1\n");

	fprintf(fProfile,"set mytics 10\n");
	fprintf(fProfile,"set mxtics 10\n");
	fprintf(fProfile,"set grid mytics ytics ls 100 lt 0, ls 101\n");
	fprintf(fProfile,"set grid mxtics xtics ls 100 lt 0, ls 101\n");

	fprintf(fProfile,"set yrange [%2.3f to %2.3f]\n", metric?fo2Meters(miny):miny, metric?fo2Meters(maxy):maxy);
	fprintf(fProfile,"set y2range [%2.3f to %2.3f]\n", metric?fo2Meters(min2y):min2y, metric?fo2Meters(max2y):max2y);

	fprintf(fProfile,"set xrange [-0.5 to %2.3f]\n",metric?mi2Km(rint(distance+0.5)):rint(distance+0.5));

	fprintf(fProfile,"set encoding iso_8859_1\n");

	fprintf(fProfile,"set term %s truecolor size %i,%i\n",term, 1280, 768);

	fprintf(fProfile,"set linetype 1 lc rgb '#6B8E23'\n");			// profile.gp montagne "Apparent terrain profile" verde
	fprintf(fProfile,"set linetype 2 lc rgb '#CD853F'\n");			// curvature.gp terra marrone
	fprintf(fProfile,"set linetype 3 lc rgb '#87CEEB'\n");          // reference.gp Optical Line of Sight Path" azzurro
	fprintf(fProfile,"set linetype 4 lc rgb '#008080'\n");          // fresnel.gp "First Fresnel Zone"	teal
	fprintf(fProfile,"set linetype 5 lc rgb '#FF00FF'\n");          // fresnel_pt_6.gp" "75% of First Fresnel Zone" viola
	// set fill
	fprintf(fProfile,"set style fill transparent solid 0.6\n");
	// mr ---------------------------------

	if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
	{
		fprintf(fProfile,"set title \"Profiles Between %s and %s (%.2f%c magnetic azimuth)\\nat %.0f MHz for K=%.3f\"\n",
					GrpStartSite.name, GrpEndSite.name, azimuth,176,
					forced_freq, (earthRadiusFoot/EARTHRADIUS_FOOT) );
	}

	else
	{
		fprintf(fProfile,"set title \"%s Height Profile Between %s and %s (%.2f%c magnetic azimuth)\"\n",program_name, GrpStartSite.name, GrpEndSite.name, azimuth,176);
	}

	if (metric)
	{
		fprintf(fProfile,"set xlabel \"From %s to %s : %.2f km\"\n",GrpStartSite.name,GrpEndSite.name, mi2Km(Distance(GrpEndSite,GrpStartSite)) );
	}
	else
	{
		fprintf(fProfile,"set xlabel \"From %s to %s : %.2f miles\"\n",GrpStartSite.name,GrpEndSite.name,Distance(GrpEndSite,GrpStartSite));
	}

	if (normalized)
	{
		if (metric)
		{
			fprintf(fProfile,"set ylabel \"Height in meters referenced to Radio Line of Sight between\\n%s and %s\"\n",GrpStartSite.name,GrpEndSite.name);
		}
		else
		{
			fprintf(fProfile,"set ylabel \"Height in feet referenced to Radio Line of Sight between\\n%s and %s\"\n",GrpStartSite.name,GrpEndSite.name);
		}
	}

	else
	{
		if (metric)
		{
			fprintf(fProfile,"set ylabel \"Height Referenced To LOS Path Between\\n%s and %s (meters)\"\n",GrpStartSite.name,GrpEndSite.name);
		}
		else
		{
			fprintf(fProfile,"set ylabel \"Height Referenced To LOS Path Between\\n%s and %s (feet)\"\n",GrpStartSite.name,GrpEndSite.name);
		}
	}

	fprintf(fProfile,"set output \"%s/%s.%s\"\n",PathCurrDir,basename,ext);

	if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
	{
		if (clutter>0.0)
		{
			if (metric)
			{
				fprintf(fProfile,"plot ");
				fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\" with filledcurves x1 lt 1, ", fn_profile_gp);
				fprintf(fProfile,"\"%s\" title \"Ground Clutter (%.2f meters)\" with lines lw 2, " ,fn_clutter_gp,fo2Meters(clutter));
				fprintf(fProfile,"\"%s\" title \"optical Line of Sight\" with lines lw 3 lt 3, ", fn_reference_gp);
				fprintf(fProfile,"\"%s\" axes x1y2 title \a\" with filledcurves x1 lt 2, ", fn_curvature_gp);
#ifdef EN_DRAW_FRESNEL
				fprintf(fProfile,"\"%s\" axes x1y1 title \"First Fresnel Zone (%.0f MHz)\" with lines lw 3 lt 4, ",fn_fresnel_gp,LR.frq_mhz);
#endif
				fprintf(fProfile,"\"%s\" title \"%.0f%% of First Fresnel Zone\" with lines lw 3 lt 5\n", fn_fresnel_pt_6_gp,fzone_clearance*100.0);
			}
			else
			{
				fprintf(fProfile,"plot ");
				fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\" with filledcurves x1 lt 1, ", fn_profile_gp);
				fprintf(fProfile,"\"%s\" title \"Ground Clutter (%.2f feet)\" with lines lw 2, " ,fn_clutter_gp, clutter);
				fprintf(fProfile,"\"%s\" title \"optical Line of Sight\" with lines lw 3 lt 3, ", fn_reference_gp);
				fprintf(fProfile,"\"%s\" axes x1y2 title \"Apparent earth's bulge\" with filledcurves x1 lt 2, ", fn_curvature_gp);
				fprintf(fProfile,"\"%s\" axes x1y1 title \"First Fresnel Zone (%.0f MHz)\"with lines lw 2, ",fn_fresnel_gp,LR.frq_mhz);
				fprintf(fProfile,"\"%s\" title \"%.0f%% of First Fresnel Zone\" with lines\n", fn_fresnel_pt_6_gp,fzone_clearance*100.0);
			}
		}

		else
		{
			// @@@ qui plot
			fprintf(fProfile,"plot ");
			fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\" with filledcurves x1 lt 1, ", fn_profile_gp);
			// fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\" with filledcurves x1 lt 1, ", fn_profile_clean_gp);
			fprintf(fProfile,"\"%s\" title \"optical Line of Sight\" with lines lw 3 lt 3, ", fn_reference_gp);
			fprintf(fProfile,"\"%s\" axes x1y2 title \"Apparent earth's bulge\" with filledcurves x1 lt 2, ", fn_curvature_gp);
#ifdef EN_DRAW_FRESNEL
			fprintf(fProfile,"\"%s\" axes x1y1 title \"First Fresnel Zone\" with lines lw 3 lt 4, ",fn_fresnel_gp);		// ,LR.frq_mhz);
#endif
			fprintf(fProfile,"\"%s\" title \"%.0f%% of First Fresnel Zone\" with lines lw 3 lt 5\n", fn_fresnel_pt_6_gp,fzone_clearance*100.0);
		}
	}

	else
	{
		if (clutter>0.0)
		{
			if (metric)
			{
				fprintf(fProfile,"plot ");
				fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\" with filledcurves x1 lt 1, ", fn_profile_gp);
				fprintf(fProfile,"\"%s\" title \"Ground Clutter (%.2f meters)\" with lines lw 2, " ,fn_clutter_gp,fo2Meters(clutter));
				fprintf(fProfile,"\"%s\" title \"optical Line of Sight\" with lines lw 3 lt 3, ", fn_reference_gp);
				fprintf(fProfile,"\"%s\" axes x1y2 title \"Apparent earth's bulge\" with filledcurves x1 lt 2\n", fn_curvature_gp);
			}
			else
			{
				fprintf(fProfile,"plot ");
				fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\" with filledcurves x1 lt 1, ", fn_profile_gp);
				fprintf(fProfile,"\"%s\" title \"Ground Clutter (%.2f feet)\" with lines lw 2, " ,fn_clutter_gp,clutter);
				fprintf(fProfile,"\"%s\" title \"optical Line of Sight\" with lines lw 3 lt 3, ", fn_reference_gp);
				fprintf(fProfile,"\"%s\" axes x1y2 title \"Apparent earth's bulge\" with filledcurves x1 lt 2\n", fn_curvature_gp);
			}
		}
		else
		{
			fprintf(fProfile,"plot ");
			fprintf(fProfile,"\"%s\" title \"Apparent terrain profile\" with filledcurves x1 lt 1, ", fn_profile_gp);
			fprintf(fProfile,"\"%s\" title \"optical Line of Sight\" with lines lw 3 lt 3, ", fn_reference_gp);
			fprintf(fProfile,"\"%s\" axes x1y2 title \"Apparent earth's bulge\" with filledcurves x1 lt 2\n", fn_curvature_gp);
		}
	}

	fclose(fProfile);

	x=GenerateGraphGnuplot();

	if (x!=-1)
	{
		if (gpsav==0)
		{
			unlink(fn_splat_gp);
			unlink(fn_profile_gp);
			unlink(fn_reference_gp);
			unlink(fn_curvature_gp);

			if (fClutter!=NULL)
			{
				unlink(fn_clutter_gp);
			}

			if ((LR.frq_mhz>=20.0) && (LR.frq_mhz<=20000.0) && fresnel_plot)
			{
				unlink(fn_fresnel_gp);
				unlink(fn_fresnel_pt_6_gp);
			}
		}

		fprintf(stdout, "\n");
		fprintf(stdout, "Height plot written to: \"%s.%s\"",basename,ext);
		fflush(stdout);
	}

	else
	fprintf(stderr,"\n*** ERROR: Error occurred invoking gnuplot!\n");
}

///////////////////////////////////////////////////////////////////
// start generazione Path analysis
//

// Perform an obstruction analysis along the path between receiver and transmitter.
void PathObstructions(
	site_data TxSite,
	site_data RxSite,
	double frq_mhz
	)
{
	int	x;
	site_data site_x=ZeroSiteData;
	double h_r;
	double h_t;
	double h_x;
	double h_r_orig;
	double cos_tx_angle;
	double cos_test_angle;
	double cos_tx_angle_f1;
	double cos_tx_angle_fpt6;
	double d_tx;
	double d_x;						// distance used to analysis
	double distToStartSite;			// distance to print from start site
	double h_r_f1;
	double h_r_fpt6;
	double h_f;
	double h_los;
	double WaveLength=0.0;
	int n_obstructions;			// n. obstructions
	
	int fCalcAntennaRaise = 1;	// 0 if not calc antenna raise

	n_obstructions = 0;			// no obstructions

	//------------------------------
	ReadPath(TxSite,RxSite);
	h_r=GetElevation(RxSite)+RxSite.alt+earthRadiusFoot;
	h_r_f1=h_r;
	h_r_fpt6=h_r;
	h_r_orig=h_r;
	h_t=GetElevation(TxSite)+TxSite.alt+earthRadiusFoot;
	d_tx=mi2Foot(Distance(RxSite,TxSite));
	cos_tx_angle=((h_r*h_r)+(d_tx*d_tx)-(h_t*h_t))/(2.0*h_r*d_tx);
	cos_tx_angle_f1=cos_tx_angle;
	cos_tx_angle_fpt6=cos_tx_angle;

	//------------------------------
	anPath.WaveLength = 0.0;
	if (frq_mhz)
	{
		WaveLength=SpeedLightAir_foot_sec/(frq_mhz*1e6);
		anPath.WaveLength = WaveLength;
	}

	// At each point along the path calculate the cosine of a sort of "inverse
	// elevation angle" at the receiver.
	// From the antenna, 0 deg. looks at the ground, and 90 deg. is parallel to the ground.

	// Start at the receiver.
	// If this is the lowest antenna, then terrain obstructions will be nearest to it.
	// (Plus, that's the way rfprobe's original los() did it.)

	// Calculate cosines only.
	// That's sufficient to compare angles and it saves the extra computational burden of acos().
	// However, note the inverted comparison: if acos(A) > acos(B), then B > A.

	// ----------------------------------------
	// obstruction analysis
	// ----------------------------------------
	for(x=path.length-1; x>0; x--)
	{
		site_x.lat=path.lat[x];
		site_x.lon=path.lon[x];
		site_x.alt=0.0;

		h_x=GetElevation(site_x)+earthRadiusFoot+clutter;

		// get the distance from the start site of analysis
		d_x=mi2Foot( Distance(RxSite,site_x) );

		// if the analysis start from RxSite, the distance to print is the same
#ifdef ORI_STARTSITE_ANALYSIS
		// the analysis start from RxSite
		distToStartSite=mi2Foot( d_x );
#else
		// the analysis start from TxSite
		distToStartSite=mi2Foot( Distance(TxSite,site_x) );
#endif

		// Deal with the LOS path first.
		cos_test_angle=((h_r*h_r)+(d_x*d_x)-(h_x*h_x))/(2.0*h_r*d_x);

		if (cos_tx_angle>cos_test_angle)
		{
			// obstruction detected
			if(site_x.lat>=0.0)
			{
				if(n_obstructions <= MAX_OBSTRUCTIONS)
				{
					if (metric)
					{
						anPath.obstruction[n_obstructions].lat =		site_x.lat;
						anPath.obstruction[n_obstructions].lon =		site_x.lon;
						anPath.obstruction[n_obstructions].distance =	fo2Km(distToStartSite);
						anPath.obstruction[n_obstructions].height =		fo2Meters((h_x-earthRadiusFoot));
					}
					else
					{
						anPath.obstruction[n_obstructions].lat =		site_x.lat;
						anPath.obstruction[n_obstructions].lon =		site_x.lon;
						anPath.obstruction[n_obstructions].distance =	fo2Miles(distToStartSite);
						anPath.obstruction[n_obstructions].height =		(h_x-earthRadiusFoot);
					}
				}
			}
			else
			{
				if(n_obstructions <= MAX_OBSTRUCTIONS)
				{
					if (metric)
					{
						anPath.obstruction[n_obstructions].lat =		site_x.lat;
						anPath.obstruction[n_obstructions].lon =		site_x.lon;
						anPath.obstruction[n_obstructions].distance =	fo2Km(distToStartSite);
						anPath.obstruction[n_obstructions].height =		fo2Meters((h_x-earthRadiusFoot));
					}
					else
					{
						anPath.obstruction[n_obstructions].lat =		site_x.lat;
						anPath.obstruction[n_obstructions].lon =		site_x.lon;
						anPath.obstruction[n_obstructions].distance =	fo2Miles(distToStartSite);
						anPath.obstruction[n_obstructions].height =		(h_x-earthRadiusFoot);
					}
				}
			}
			n_obstructions++;		// increment n. obstructions
		}

		while (cos_tx_angle>cos_test_angle)
		{
			h_r+=1;
			cos_test_angle=((h_r*h_r)+(d_x*d_x)-(h_x*h_x))/(2.0*h_r*d_x);
			cos_tx_angle=((h_r*h_r)+(d_tx*d_tx)-(h_t*h_t))/(2.0*h_r*d_tx);
		}

		if (frq_mhz)
		{
			// Now clear the first Fresnel zone...

			cos_tx_angle_f1=((h_r_f1*h_r_f1)+(d_tx*d_tx)-(h_t*h_t))/(2.0*h_r_f1*d_tx);
			h_los=sqrt(h_r_f1*h_r_f1+d_x*d_x-2*h_r_f1*d_x*cos_tx_angle_f1);
			h_f=h_los-sqrt(WaveLength*d_x*(d_tx-d_x)/d_tx);

			while (h_f<h_x)
			{
				h_r_f1+=1;
				cos_tx_angle_f1=((h_r_f1*h_r_f1)+(d_tx*d_tx)-(h_t*h_t))/(2.0*h_r_f1*d_tx);
				h_los=sqrt(h_r_f1*h_r_f1+d_x*d_x-2*h_r_f1*d_x*cos_tx_angle_f1);
				h_f=h_los-sqrt(WaveLength*d_x*(d_tx-d_x)/d_tx);
			}

			// and clear the 60% F1 zone.

			cos_tx_angle_fpt6=((h_r_fpt6*h_r_fpt6)+(d_tx*d_tx)-(h_t*h_t))/(2.0*h_r_fpt6*d_tx);
			h_los=sqrt(h_r_fpt6*h_r_fpt6+d_x*d_x-2*h_r_fpt6*d_x*cos_tx_angle_fpt6);
			h_f=h_los-fzone_clearance*sqrt(WaveLength*d_x*(d_tx-d_x)/d_tx);

			while (h_f<h_x)
			{
				h_r_fpt6+=1;
				cos_tx_angle_fpt6=((h_r_fpt6*h_r_fpt6)+(d_tx*d_tx)-(h_t*h_t))/(2.0*h_r_fpt6*d_tx);
				h_los=sqrt(h_r_fpt6*h_r_fpt6+d_x*d_x-2*h_r_fpt6*d_x*cos_tx_angle_fpt6);
				h_f=h_los-fzone_clearance*sqrt(WaveLength*d_x*(d_tx-d_x)/d_tx);
			}
		}
	}	// end of obstruction analysis

	anPath.NumObstructions = (n_obstructions);

	anPath.RxAntennaRaiseAboveGround=0.0;
	anPath.RxAntennaRaiseAboveGroundPercFresnel=0.0;

	if(fCalcAntennaRaise)
	{
		// calc antenna raise
		//--------------------------
		// modify 04/07
		if (h_r>h_r_orig)
		{
			if (metric)
			{
				anPath.RxAntennaRaiseAboveGround = (fo2Meters(h_r-GetElevation(RxSite)-earthRadiusFoot));
			}
			else
			{
				anPath.RxAntennaRaiseAboveGround = (h_r-GetElevation(RxSite)-earthRadiusFoot);
			}
		}
	}
	if(frq_mhz)
	{
		if(fCalcAntennaRaise)
		{
			//--------------------------
			// modify 04/07
			if (h_r_fpt6>h_r_orig)
			{
				if (metric)
				{
					anPath.RxAntennaRaiseAboveGroundPercFresnel = (fo2Meters(h_r_fpt6-GetElevation(RxSite)-earthRadiusFoot));
				}
				else
				{
					anPath.RxAntennaRaiseAboveGroundPercFresnel = (h_r_fpt6-GetElevation(RxSite)-earthRadiusFoot);
				}
			}
		}
		else
		{
			if (h_r_fpt6<=h_r_orig)
			{
				anPath.RxAntennaRaiseAboveGroundPercFresnel=0.0;
			}
			//--------------------------
		}
		if(fCalcAntennaRaise)
		{
			//--------------------------
			// modify 04/07
			if (h_r_f1>h_r_orig)
			{
				if (metric)
				{
					anPath.RxAntennaRaiseAboveGroundPercFresnel = (fo2Meters(h_r_f1-GetElevation(RxSite)-earthRadiusFoot));
				}
				else
				{
					anPath.RxAntennaRaiseAboveGroundPercFresnel = (h_r_f1-GetElevation(RxSite)-earthRadiusFoot);
				}
			}
		}
	}			// if(frq_mhz)
}

// ---------------------------------------------------------------------
// EXECUTE PATH ANALYSIS

void AnalysisPropagationModel(
	site_data TxSite, 
	site_data RxSite
)
{
	FILE *fdProfile = NULL;
	
	int	x;
	int y;
	int errnum;
	char strmode[100];
	double distance;
	char FoundObstruction=0;
	double elevation;
	double maxloss=-100000.0;
	double minloss=100000.0;
	double azimuth;

	double source_alt;
	double dest_alt;
	double source_alt2;
	double dest_alt2;
	double test_alt;
	
	double cos_xmtr_angle;
	double cos_test_angle=0.0;

	double path_loss;
	double total_loss=0.0;

	double pattern=1.0;
	double patterndB=0.0;
	// -----------------------------------------------
	// Analysis of propagation model
	// -----------------------------------------------
	//
	fdProfile=fopen(fn_profile_gp,"w");

	for (y=2; y<(path.length-1); y++)  // path.length-1 avoids LR error
	{
		distance=mi2Foot(path.distance[y]);
		source_alt=four_thirds_earthRadiusFoot+TxSite.alt+path.elevation[0];
		dest_alt=four_thirds_earthRadiusFoot+RxSite.alt+path.elevation[y];
		dest_alt2=dest_alt*dest_alt;
		source_alt2=source_alt*source_alt;

		// Calculate the cosine of the elevation of the receiver as seen by the transmitter.
		cos_xmtr_angle=((source_alt2)+(distance*distance)-(dest_alt2))/(2.0*source_alt*distance);

		if (got_elevation_pattern)
		{
			// If an antenna elevation pattern is available, the
			// following code determines the elevation angle to
			// the first obstruction along the path.

			for (x=2, FoundObstruction=0; x<y && FoundObstruction==0; x++)
			{
				distance=mi2Foot((path.distance[y]-path.distance[x]));
				test_alt=four_thirds_earthRadiusFoot+path.elevation[x];

				// Calculate the cosine of the elevation
				// angle of the terrain (test point)
				// as seen by the transmitter.

				cos_test_angle=((source_alt2)+(distance*distance)-(test_alt*test_alt))/(2.0*source_alt*distance);

				// Compare these two angles to determine if
				// an obstruction exists.  Since we're comparing
				// the cosines of these angles rather than
				// the angles themselves, the sense of the
				// following "if" statement is reversed from
				// what it would be if the angles themselves
				// were compared.

				if (cos_xmtr_angle>=cos_test_angle)
				{
					FoundObstruction=1;
				}
			}
			// At this point, we have the elevation angle
			// to the first obstruction (if it exists).
		}			// end if (got_elevation_pattern)

		// Determine path loss for each point along the path using ITWOM's point_to_point mode
		// starting at x=2 (number_of_points = 1), the shortest distance terrain can play a role in
		// path loss.

		elev[0]=y-1;	// (number of points - 1)
		// Distance between elevation samples
		elev[1]=METERS_PER_MILE*(path.distance[y]-path.distance[y-1]);

#ifdef DBG_ITWOM
		// for dbg:
		// rep moroni6 maweni2 generate err data of path loss
		// rep moroni9 maweni2 generate correct data
		if(((int)elev[0])==18)
		{
			char pro_model[32]="";
			//----------------------------
			// 30/09
			fprintf(fMr, "---------------[start before point_to_point]\n");
			if(PropagationModel==PROP_MODEL_LRICE)
			sprintf(pro_model,"PROP_MODEL_LRICE");
			else
			sprintf(pro_model,"ITWOM_Ver3.0");
			fprintf(fMr, "point_to_point modello : %s\n", pro_model);
			fprintf(fMr,  "elev[0]= [%lf]\n",							elev[0]); 
			fprintf(fMr,  "elev[1]= [%lf]\n",							elev[1]); 
			fprintf(fMr,  "(anPath.Tx.alt)= [%lf]\n",            		(anPath.Tx.alt));
			fprintf(fMr,  "(anPath.Rx.alt)= [%lf]\n",            		(anPath.Rx.alt));
			fprintf(fMr,  "fo2Meters(anPath.Tx.alt)= [%lf]\n",           fo2Meters(anPath.Tx.alt));
			fprintf(fMr,  "fo2Meters(anPath.Rx.alt)= [%lf]\n",           fo2Meters(anPath.Rx.alt));
			fprintf(fMr,  "anPath.lrPar.EarthDielectricConst= [%lf]\n",  anPath.lrPar.EarthDielectricConst);
			fprintf(fMr,  "anPath.lrPar.EarthConductivity= [%lf]\n",     anPath.lrPar.EarthConductivity);
			fprintf(fMr,  "anPath.lrPar.AtmBendingCons= [%lf]\n",        anPath.lrPar.AtmBendingCons);
			fprintf(fMr,  "anPath.lrPar.frq_mhz= [%lf]\n",               anPath.lrPar.frq_mhz);
			fprintf(fMr,  "anPath.lrPar.radio_climate= [%lf]\n",         anPath.lrPar.radio_climate);
			fprintf(fMr,  "anPath.lrPar.polarization= [%lf]\n",          anPath.lrPar.polarization);
			fprintf(fMr,  "anPath.lrPar.FractionSituations= [%lf]\n",    anPath.lrPar.FractionSituations);
			fprintf(fMr,  "anPath.lrPar.FractionTime= [%lf]\n",          anPath.lrPar.FractionTime);
			fprintf(fMr, "---------------[end before point_to_point]\n");
			fflush(fMr);
			//----------------------------
		}
#endif
		
		// model analysis
		Point2PointModel(
			elev,
			(anPath.Tx.alt),
			(anPath.Rx.alt),
			&anPath.lrPar,
			path_loss,
			strmode,
			errnum
		);
		SaveModelResults(
			path_loss,
			strmode,
			errnum
		);
		
#ifdef DBG_ITWOM
		if(((int)elev[0])==18)
		{
		//---------------------
		fprintf(fMr,  "path_loss= [%lf]\n",						path_loss); 
		fprintf(fMr,  "errnum= [%i]\n",							errnum); 
		fprintf(fMr, "---------------[end before point_to_point]\n");
		//---------------------
		}
#endif
		
		if (FoundObstruction)
		{
			elevation=(DEGREES(acos(cos_test_angle)))-90.0;
		}
		else
		{
			elevation=(DEGREES(acos(cos_xmtr_angle)))-90.0;
		}

		// Integrate the antenna's radiation
		// pattern into the overall path loss.
		x=(int)rint(10.0*(10.0-elevation));

		pattern=1.0;
		patterndB=0.0;
		if (x>=0 && x<=1000)
		{
			pattern=(double)anPath.lrPar.antenna_pattern[(int)azimuth][x];

			if (pattern!=0.0)
			{
				patterndB=20.0*log10(pattern);
			}
		}
		anPath.patterndB = patterndB;
		total_loss=path_loss-patterndB;

		// ----------------------
		// store total_loss result
		anPath.total_loss = total_loss;
		// ----------------------
		if (total_loss>maxloss)
		{
			maxloss=total_loss;
		}
		if (total_loss<minloss)
		{
			minloss=total_loss;
		}
		// ----------------------
		// store max min loss result
		anPath.maxloss = maxloss;
		anPath.minloss = minloss;
		
		// ----------------------
		if (metric)
		{
			fprintf(fdProfile,"%f\t%f\n",mi2Km(path.distance[y]),total_loss);
		}
		else
		{
			fprintf(fdProfile,"%f\t%f\n",path.distance[y],total_loss);
		}
	}		// end of analysis of propagation model

	fclose(fdProfile);
}

// This function compile path data analysis.
//
// void GeneratePathAnalysis(site_data TxSite, site_data RxSite, char fDrawGraph)
// ex GeneratePathAnalysis: viene chiamata da un'altra funzione per poter 
// correggere il problema rilevato nel modulo itwom3.0.cpp
//
void ExecutePathAnalysis(site_data TxSite, site_data RxSite)
{
	int	x;
	int y;
	char propstring[20];
	double haatHeight;
	double angle1;
	double angle2;
	
	double distance;
	double azimuth;
	
	// double four_thirds_earthRadiusFoot;
	double field_strength;
	double free_space_loss=0.0;
	double eirp=0.0;				// EIRP: equivalent isotropically radiated power (effective isotropically radiated power)
	double voltage;
	double rxp;
	double dBm;
	double power_density;

	int errnum;
	char strmode[100];
	/**
	double pattern=1.0;
	double patterndB=0.0;
	**/
	//--------------------
	anPath.units = metric;			// has unit types

	// copy parameters data_LR used in analysis (all)
	anPath.lrPar = LR;				// data_LR used in analysis

	//--------------------
	// @@@ debug
	// anPath.PropagationModel = PROP_MODEL_ITWOM;		// ITWOM_Ver3.0

	anPath.PropagationModel = PROP_MODEL_LRICE;			// longley rice
	//--------------------
	PropagationModel = anPath.PropagationModel;

	// ---------------- site names:
	snprintf(anPath.Tx.SiteName, 50, "%s",TxSite.name);		// transmitter
	snprintf(anPath.Rx.SiteName, 50, "%s",RxSite.name);		// receiver

	// ---------------- gps position
	// transmitter
	anPath.Tx.lat = TxSite.lat;
	anPath.Tx.lon = TxSite.lon;
	anPath.Tx.alt = TxSite.alt;		// antenna height
	// receiver
	anPath.Rx.lat = RxSite.lat;
	anPath.Rx.lon = RxSite.lon;
	anPath.Rx.alt = RxSite.alt;		// antenna height

	// ---------------- elevation:
	if (metric)
	{
		anPath.Tx.Elevation = fo2Meters(GetElevation(TxSite));
		anPath.Rx.Elevation = fo2Meters(GetElevation(RxSite));
	}
	else
	{
		anPath.Tx.Elevation = (GetElevation(TxSite));
		anPath.Rx.Elevation = (GetElevation(RxSite));
	}
	// ---------------- antenna height ground:
	if (metric)
	{
		anPath.Tx.AntennaHground = fo2Meters(TxSite.alt);
		anPath.Rx.AntennaHground = fo2Meters(RxSite.alt);
	}
	else
	{
		anPath.Tx.AntennaHground = (TxSite.alt);
		anPath.Rx.AntennaHground = (RxSite.alt);
	}
	// ---------------- antenna height sea:
	if (metric)
	{
		anPath.Tx.AntennaHSea = (fo2Meters(TxSite.alt+GetElevation(TxSite)));
		anPath.Rx.AntennaHSea = (fo2Meters(RxSite.alt+GetElevation(RxSite)));
	}
	else
	{
		anPath.Tx.AntennaHSea = (TxSite.alt+GetElevation(TxSite));
		anPath.Rx.AntennaHSea = (RxSite.alt+GetElevation(RxSite));
	}

	// ---------------- distance from tx and rx
	if (metric)
	{
		anPath.TxRxDistance = ( mi2Km(Distance(TxSite,RxSite)) );
	}
	else
	{
		anPath.TxRxDistance = (Distance(TxSite,RxSite));
	}

	// ---------------- calculate haat: Height Antenna above Average Terrain
	// transmitter
	haatHeight=haat(TxSite);
	if(haatHeight>NO_LAND_HEIGHT)
	{
		if (metric)
		{
			anPath.Tx.HAAT_AntHeightAboveAverageTerrain = (fo2Meters(haatHeight));
		}
		else
		{
			anPath.Tx.HAAT_AntHeightAboveAverageTerrain = (haatHeight);
		}
	}
	else
	{
		//  can't calc haat
		anPath.Tx.HAAT_AntHeightAboveAverageTerrain = (-5000);
	}
	// receiver
	haatHeight=haat(RxSite);
	if(haatHeight>NO_LAND_HEIGHT)
	{
		if (metric)
		{
			anPath.Rx.HAAT_AntHeightAboveAverageTerrain = (fo2Meters(haatHeight));
		}
		else
		{
			anPath.Rx.HAAT_AntHeightAboveAverageTerrain = (haatHeight);
		}
	}
	else
	{
		//  can't calc haat
		anPath.Rx.HAAT_AntHeightAboveAverageTerrain = (-5000);
	}

	// ---------------- calculate azimuth
	// transmitter
	azimuth=AzimuthSites(TxSite,RxSite);		// azimuth from tx to rx
	anPath.Tx.azimuth = (azimuth);

	angle1 = ElevationAngle(TxSite,RxSite);
	anPath.Tx.ElevationAngle = angle1;

	angle2 = ElevationAngleUntilObstruction(TxSite,RxSite,earthRadiusFoot);
	anPath.Tx.AngleObstruction1 = angle2;

	anPath.Tx.pattern = 1.0;
	anPath.Tx.patterndB = 0.0;
	if (got_azimuth_pattern || got_elevation_pattern)
	{
		x=(int)rint(10.0*(10.0-angle2));

		if (x>=0 && x<=1000)
		{
			anPath.Tx.pattern = (double)anPath.lrPar.antenna_pattern[(int)rint(azimuth)][x];
		}
		anPath.Tx.patterndB = 20.0*log10(anPath.Tx.pattern);
	}

	// ---------------- calculate azimuth
	// Receiver
	azimuth=AzimuthSites(RxSite,TxSite);		// azimuth from rx to tx
	anPath.Rx.azimuth = (azimuth);

	angle1=ElevationAngle(RxSite,TxSite);
	anPath.Rx.ElevationAngle = angle1;

	angle2=ElevationAngleUntilObstruction(RxSite,TxSite,earthRadiusFoot);
	anPath.Rx.AngleObstruction1 = angle2;

	anPath.Rx.pattern = 1.0;
	anPath.Rx.patterndB = 0.0;

	// decode radio climate
	switch (anPath.lrPar.radio_climate)
	{
	case CLIMA_EQUATORIAL				: snprintf(anPath.RadioClimateType, 50, "Equatorial");					break;
	case CLIMA_CONTINENTAL_SUBTROPICAL	: snprintf(anPath.RadioClimateType, 50, "Continental Subtropical");		break;
	case CLIMA_MARITIME_SUBTROPICAL		: snprintf(anPath.RadioClimateType, 50, "Maritime Subtropical");		break;
	case CLIMA_DESERT					: snprintf(anPath.RadioClimateType, 50, "Desert");						break;
	case CLIMA_CONTINENTAL_TEMPERATE	: snprintf(anPath.RadioClimateType, 50, "Continental Temperate");		break;
	case CLIMA_MARITIME_TEMPERATE_LAND	: snprintf(anPath.RadioClimateType, 50, "Martitime Temperate, Over Land");break;
	case CLIMA_MARITIME_TEMPERATE_SEA	: snprintf(anPath.RadioClimateType, 50, "Maritime Temperate, Over Sea");break;
	default:
		snprintf(anPath.RadioClimateType, 50, "Unknown");
	}
	// decode polarization
	if (anPath.lrPar.polarization==POL_HORIZONTAL)
	{
		snprintf(anPath.PolarizationType, 50, "Horizontal");
	}
	if (anPath.lrPar.polarization==POL_VERTICAL)
	{
		snprintf(anPath.PolarizationType, 50, "Vertical");
	}

	// initialize values
	snprintf(anPath.txERP_W, 30, "0.0 mW");
	anPath.txERP_dBm = 0.0;
	snprintf(anPath.txEIRP_W, 30, "0.0 mW");
	anPath.txEIRP_dBm = 0.0;
	anPath.TxRxAntennaPattern = 1e-20;
	anPath.TxRxAntennaPattern_dB = 0;

	if (anPath.lrPar.EffRadiatedPower!=0.0)
	{
		// calculate Transmitter Effective Radiated Power in Watts or dBm, https://en.wikipedia.org/wiki/Effective_radiated_power
		if (anPath.lrPar.EffRadiatedPower<1.0)
		{
			snprintf(anPath.txERP_W, 30, "%.1lf mW",1000.0*anPath.lrPar.EffRadiatedPower);
		}
		if (anPath.lrPar.EffRadiatedPower>=1.0 && anPath.lrPar.EffRadiatedPower<10.0)
		{
			snprintf(anPath.txERP_W, 30, "%.1lf W",anPath.lrPar.EffRadiatedPower);
		}
		if (anPath.lrPar.EffRadiatedPower>=10.0 && anPath.lrPar.EffRadiatedPower<10.0e3)
		{
			snprintf(anPath.txERP_W, 30, "%.0lf W",anPath.lrPar.EffRadiatedPower);
		}
		if (anPath.lrPar.EffRadiatedPower>=10.0e3)
		{
			snprintf(anPath.txERP_W, 30, "%.3lf kW",anPath.lrPar.EffRadiatedPower/1.0e3);
		}
		dBm=10.0*(log10(anPath.lrPar.EffRadiatedPower*1000.0));
		anPath.txERP_dBm = dBm;

		// calc EIRP: equivalent isotropically radiated power (effective isotropically radiated power)
		eirp = anPath.lrPar.EffRadiatedPower*1.636816521;
		if (eirp<1.0)
		{
			snprintf(anPath.txEIRP_W, 30, "%.1lf mW",1000.0*eirp);
		}
		if (eirp>=1.0 && eirp<10.0)
		{
			snprintf(anPath.txEIRP_W, 30, "%.1lf W",eirp);
		}
		if (eirp>=10.0 && eirp<10.0e3)
		{
			snprintf(anPath.txEIRP_W, 30, "%.0lf W",eirp);
		}
		if (eirp>=10.0e3)
		{
			snprintf(anPath.txEIRP_W, 30, "%.3lf kW",eirp/1.0e3);
		}
		dBm=10.0*(log10(eirp*1000.0));
		anPath.txERP_dBm = dBm;
	}

	// analysis in base of model type
	snprintf(anPath.lrice.model, 50, "Longley-Rice");
	snprintf(anPath.itwom.model, 50, "ITWOM_Ver%.1f",ITWOMVersion());

	if (anPath.lrPar.frq_mhz>0.0)
	{
		// generates sequence of (lat, lon) positions between source and destination
		// and stores elevation and distance information for points along that path in the "path" structure
		ReadPath(TxSite, RxSite);  // source=TX, RxSite=RX

		// Copy elevations plus clutter along path into the elev[] array.
		COPY_ELEVATIONS_PLUS_CLUTTER();

		azimuth=rint(AzimuthSites(TxSite,RxSite));

		AnalysisPropagationModel(TxSite, RxSite);
		
		// calc free space path loss
		distance=Distance(TxSite,RxSite);
		logUsr("PathReport Distance\n");
		
		//----------------------------
		// 30/09
		fprintf(stdout, "free_space_loss before: %lf\n", free_space_loss);
		fprintf(stdout, "frequency: %lf\n", anPath.lrPar.frq_mhz);
		fprintf(stdout, "distance: %lf\n", distance);
		fflush(stdout);
		//----------------------------
		if (distance!=0.0)
		{
			free_space_loss=36.6+(20.0*log10(anPath.lrPar.frq_mhz))+(20.0*log10(distance));
			anPath.free_space_loss = free_space_loss;
		}
		
		//----------------------------
		// 30/09
		fprintf(stdout, "free_space_loss after calc: %lf\n", free_space_loss);
		fflush(stdout);
		//----------------------------

		anPath.TerrainShieldingAttenuation = 0.0;

		//----------------------------
		// 30/09
		fprintf(stdout, "path_loss before calc TerrainShieldingAttenuation: %lf\n", anPath.path_loss);
		fprintf(stdout, "free_space_loss before calc TerrainShieldingAttenuation: %lf\n", free_space_loss);
		fflush(stdout);
		//----------------------------
		
		if (free_space_loss!=0.0)
		{
			anPath.TerrainShieldingAttenuation = (anPath.path_loss-free_space_loss);
		}

		//----------------------------
		// 30/09
		fprintf(stdout, "TerrainShieldingAttenuation: %lf\n", anPath.TerrainShieldingAttenuation);
		fflush(stdout);
		//----------------------------
		
		anPath.PathLossWithTxAntennaPattern = 0.0;
		if (anPath.patterndB!=0.0)
		{
			anPath.PathLossWithTxAntennaPattern = (anPath.total_loss);
		}

		anPath.field_strength = 0.0;

		if (anPath.lrPar.EffRadiatedPower!=0.0)
		{
			field_strength=(139.4+
						(20.0*log10(anPath.lrPar.frq_mhz))-
						 anPath.total_loss)+
						 (10.0*log10(anPath.lrPar.EffRadiatedPower/1000.0));
			anPath.field_strength = field_strength;

			// dBm is referenced to EIRP

			rxp=eirp/(pow(10.0,(anPath.total_loss/10.0)));
			dBm=10.0*(log10(rxp*1000.0));

			power_density=(eirp/(pow(10.0,(anPath.total_loss-free_space_loss)/10.0)));
			// divide by 4*PI*distance_in_meters squared
			power_density/=(4.0*PI*distance*distance*2589988.11);
			anPath.power_density = power_density;

			// values
			anPath.FieldStrength = (field_strength);
			anPath.SignalPowerLevel = (dBm);
			anPath.SignalPowerDensity = (10.0*log10(power_density));
			voltage=1.0e6*sqrt(50.0*(eirp/(pow(10.0,(anPath.total_loss-2.14)/10.0))));
			anPath.VoltageAcross50OhmDipole_uV = (voltage);
			anPath.VoltageAcross50OhmDipole_dBuV = (20.0*log10(voltage));

			voltage=1.0e6*sqrt(75.0*(eirp/(pow(10.0,(anPath.total_loss-2.14)/10.0))));
			anPath.VoltageAcross75OhmDipole_uV = (voltage);
			anPath.VoltageAcross75OhmDipole_dBuV = (20.0*log10(voltage));

			// units
			snprintf(anPath.umFieldStrength, 20, "dBuV/meter");
			snprintf(anPath.umSignalPowerLevel, 20, "dBm");
			snprintf(anPath.umSignalPowerDensity, 20, "dBW per square meter");
			snprintf(anPath.umVoltageAcross50OhmDipole_uV, 20, "uV");
			snprintf(anPath.umVoltageAcross50OhmDipole_dBuV, 20, "dBuV");
			snprintf(anPath.umVoltageAcross75OhmDipole_uV, 20, "uV");
			snprintf(anPath.umVoltageAcross75OhmDipole_dBuV, 20, "dBuV");
		}

		// modif. 04/10
		errnum = anPath.ErrorCode;
		snprintf(strmode, 50, "%s", anPath.strmode);

		if(PropagationModel==PROP_MODEL_ITWOM)
		{
			if (strcmp(strmode,"L-o-S")==0)
			{
				snprintf(anPath.extmode, 50, "Line of Sight ");
			}
			if (strncmp(strmode,"1_Hrzn",6)==0)
			{
				snprintf(anPath.extmode, 50, "Single Horizon ");
			}
			if (strncmp(strmode,"2_Hrzn",6)==0)
			{
				snprintf(anPath.extmode, 50, "Double Horizon ");
			}
			y=strlen(strmode);
			if (y>19)
			{
				y=19;
			}
			for (x=6; x<y; x++)
			{
				propstring[x-6]=strmode[x];
			}
			propstring[x]=0;

			if (strncmp(propstring,"_Diff",5)==0)
			{
				strncat(anPath.extmode,"Diffraction Dominant", 50);
			}
			if (strncmp(propstring,"_Tropo",6)==0)
			{
				strncat(anPath.extmode,"Troposcatter Dominant", 50);
			}
			if (strncmp(propstring,"_Peak",5)==0)
			{
				strncat(anPath.extmode,"RX at Peak Terrain Along Path", 50);
			}
			strncpy(anPath.itwom.extmode, anPath.extmode, 50);
		}

		// decode analysis error message
		switch (errnum)
		{
		case 0:
			snprintf(anPath.ErrorMsg, 200, "No error");
			break;

		case 1:
			snprintf(anPath.ErrorMsg, 200, "Warning: Some parameters are nearly out of range.");
			strncat(anPath.ErrorMsg, " Results should be used with caution", 200);
			break;

		case 2:
			snprintf(anPath.ErrorMsg, 200, " Note: Default parameters have been substituted for impossible ones.");
			break;

		case 3:
			snprintf(anPath.ErrorMsg, 200, "Warning: A combination of parameters is out of range.");
			strncat(anPath.ErrorMsg, " Results are probably invalid.", 200);
			break;

		default:
			snprintf(anPath.ErrorMsg, 200, "Warning: Some parameters are out of range.");
			strncat(anPath.ErrorMsg, " Results are probably invalid.", 200);
		}
	}

	if (metric)
	{
		anPath.RaiseTerrainForGroundClutter = (fo2Meters(clutter));
	}
	else
	{
		anPath.RaiseTerrainForGroundClutter = (clutter);
	}

	// analisi delle ostruzioni
	PathObstructions(TxSite, RxSite, anPath.lrPar.frq_mhz);

}	// end of void GeneratePathAnalysis

// to save the old results of path analysis
analysis_data_path saveAnPath;			// data path report

void GeneratePathAnalysis(site_data TxSite, site_data RxSite)
{
	// call ExecutePathAnalysis for analysis
	ExecutePathAnalysis(TxSite, RxSite);		// PathAnalysis for analysis report PathReport
	// check the error condition
	if((anPath.ErrorCode != 0) && (anPath.NumObstructions == 0))
	{
		// error condition
		saveAnPath = anPath;			// save the old analysis
		// repeat analysis inverting tx with rx
		ExecutePathAnalysis(RxSite, TxSite);		// PathAnalysis for analysis report PathReport
		if((anPath.ErrorCode == 0) && (anPath.NumObstructions == 0))
		{
			// the new analysis has the correct velues
			saveAnPath.path_loss = anPath.path_loss;
			saveAnPath.total_loss = anPath.total_loss;
			saveAnPath.ErrorCode = anPath.ErrorCode;
			saveAnPath.TerrainShieldingAttenuation = anPath.TerrainShieldingAttenuation;
			snprintf(saveAnPath.strmode, 50, "%s", anPath.strmode);
			snprintf(saveAnPath.extmode, 50, "%s", anPath.extmode);
			snprintf(saveAnPath.ErrorMsg, 50, "%s", anPath.ErrorMsg);
			
			saveAnPath.lrice.path_loss = anPath.lrice.path_loss;
			saveAnPath.lrice.total_loss = anPath.lrice.total_loss;
			saveAnPath.itwom.path_loss = anPath.itwom.path_loss;
			saveAnPath.itwom.total_loss = anPath.itwom.total_loss;
		}
		anPath = saveAnPath;			// restore the analysis
	}
}


///////////////////////////////////////////////////////////////////


char stringCommandLine[2048];				// sCmd string

// generate error report
void GenerateErrorReport(site_data TxSite, site_data RxSite, char *str_error)
{
	FILE *fPathReport=NULL;			// full report
	FILE *fPathReportRed=NULL;		// reduced report
	double dist;
	char fn_outimg[PATH_MAX];

	// modify 25/06:
	// name of report in the output directory
	// modify 07/07: rep from tx to rx
	snprintf(fn_pathreport, PATH_MAX, "%s/%s_%s.txt",
					PathOutputDir,
					TxSite.name, RxSite.name);
	printf("Err Report Name: [%s]\n", fn_pathreport);

	// name of reduced report in the output directory
	// modify 07/07: rep from tx to rx
	snprintf(fn_pathreport_red, PATH_MAX, "%s/%s_%s_red.txt",
					PathOutputDir,
					TxSite.name,RxSite.name);

	fPathReport=fopenFullPath(fn_pathreport,"w");
	fPathReportRed=fopenFullPath(fn_pathreport_red,"w");

	dist = mi2Km( Distance(TxSite, RxSite) );
	fprintf(fPathReport,"\nError report\n");
	fprintf(fPathReport,"-----------------------------------\n\n");

	fprintf(fPathReportRed,"\nError report\n");
	fprintf(fPathReportRed,"-----------------------------------\n\n");

	fprintf(fPathReport,"Site %s (%8.4f N,%9.4f W)\n",
					RxSite.name, RxSite.lat, RxSite.lon);
	fprintf(fPathReport,"Site %s (%8.4f N,%9.4f W)\n",
					TxSite.name, TxSite.lat, TxSite.lon);
	fprintf(fPathReport,"Distance: %.2f km\n", dist);

	fprintf(fPathReportRed,"Site %s (%8.4f N,%9.4f W)\n",
					RxSite.name, RxSite.lat, RxSite.lon);
	fprintf(fPathReportRed,"Site %s (%8.4f N,%9.4f W)\n",
					TxSite.name, TxSite.lat, TxSite.lon);
	fprintf(fPathReportRed,"Distance: %.2f km\n", dist);

	fprintf(fPathReport,"Error:\n%s\n\n",
					str_error);
	fprintf(fPathReportRed,"Error:\n%s\n\n",
					str_error);

	// name of report in the output directory
	snprintf(fn_outimg, PATH_MAX, "%s/%s_%s.png",
					PathOutputDir,
					TxSite.name, RxSite.name);

	// copy error image
	sprintf(stringCommandLine, "cp %s/botrf_error.png %s",
				PathCurrDir,
				fn_outimg);

	logUsr("--- GenerateErrorReport cmd copy [%s]\n", stringCommandLine);
	system(stringCommandLine);
}

// ---------------------------------------------------------------------


// ---------------------------------------------------------------------
// GeneratePowerPlot
// ---------------------------------------------------------------------
void GeneratePowerPlot(site_data TxSite, site_data RxSite, char *name)
{
	// This function writes a Power PLot (name.txt) to the filesystem.
	// output file indicating the ITM model loss between the source and
	// RxSite locations.
	// "filename" is the name assigned to the output file generated by gnuplot.
	// The filename extension is used to set gnuplot's terminal setting and output file type.
	// If no extension is found, .png is assumed.

	char cmd[PATH_MAX];

	int	x;
	char report_name[80];
	FILE *fd=NULL;

	double ris;

	double TxPower;					// Tx transmission power
	double TxCableLoss;				// Tx cable loss
	double TxAntGain;				// Tx antenna gain

	double RxAntGain;				// Rx antenna gain
	double RxCableLoss;				// Rx cable loss
	double RxSensitivity;			// Rx sensitivity

	double Eirp;					// EIRP: equivalent isotropically radiated power (effective isotropically radiated power)
	double Margin;

	// call GeneratePathAnalysis for analysis
	GeneratePathAnalysis(TxSite, RxSite);		// PathAnalysis for PowerPlot

		fprintf(stdout,">>>>>>>>>>>>> END PATH ANALYSIS\n");
		fflush(stdout);

	// =============================================================
	// plot the graph
	//

	TxPower = dtPowerLink.TxPower;							// Tx transmission power
	TxCableLoss = dtPowerLink.TxCableLoss;					// Tx cable loss
	TxAntGain = dtPowerLink.TxAntGain;						// Tx antenna gain

	RxAntGain = dtPowerLink.RxAntGain;						// Rx antenna gain
	RxCableLoss = dtPowerLink.RxCableLoss;					// Rx cable loss
	RxSensitivity = dtPowerLink.RxSensitivity;				// Rx sensitivity

	// calc eirp and calc margin
	ris = TxPower;
	ris = ris + TxAntGain;
	ris = ris + TxCableLoss;
	Eirp = ris;
	ris = ris - anPath.free_space_loss;
	ris = ris + RxAntGain;
	ris = ris + RxCableLoss;
	ris = ris - RxSensitivity;
	Margin = ris;

	fd=fopen(fn_power_gp,"w");

	fprintf(fd,"reset\n");
	fprintf(fd,"set term png truecolor size 1280,768\n");
	fprintf(fd,"set output \"%s\"\n", graph_pow_name);
	fprintf(fd,"set multiplot\n");
	fprintf(fd,"set title \"From %s to %s\" font \"Verdiana,22\" tc rgb \"#000000\"\n", anPath.Tx.SiteName, anPath.Rx.SiteName);
	fprintf(fd,"set title \"From %s to %s\" font \"Verdiana,22\" tc rgb \"#000000\"\n", anPath.Tx.SiteName, anPath.Rx.SiteName);
	fprintf(fd,"set xrange [0:1279]\n");
	fprintf(fd,"set yrange [0:767]\n");
	fprintf(fd,"set xlabel \"distance\" font \"Verdiana,16\"\n");
	fprintf(fd,"set ylabel \"dBm\" font \"Verdiana,16\"\n");
	fprintf(fd,"unset tics\n");
	fprintf(fd,"unset border\n");
	fprintf(fd,"set lmargin at screen 0.175\n");
	fprintf(fd,"set rmargin at screen 0.9\n");
	fprintf(fd,"set bmargin at screen 0.175\n");
	fprintf(fd,"set tmargin at screen 0.9\n");
	fprintf(fd,"set arrow from graph 0.0,0 to graph 0.0,0.6 filled\n");
	fprintf(fd,"set arrow from graph 0,0.0 to graph 1,0.0 filled\n");
	// set graphic background
	if(Margin >= 0.0)
	{
		fprintf(fd,"plot \"%s\" binary filetype=png origin=(4,2) w rgbimage\n", fn_power_bck_pos);
	}
	else
	{
		fprintf(fd,"plot \"%s\" binary filetype=png origin=(4,2) w rgbimage\n", fn_power_bck_neg);
	}

	fprintf(fd,"set label \"%s\\nTX\" at 125,700 center front font \"Verdiana,20\" tc rgb \"#000000\"\n", anPath.Tx.SiteName);
	fprintf(fd,"set label \"%s\\nRX\" at 1000,700 center front font \"Verdiana,20\" tc rgb \"#000000\"\n", anPath.Rx.SiteName);
	fprintf(fd,"set label \"%.0f dB at %.0f km\" at 570,495 center front font \"Verdiana,16\" tc rgb \"#000000\"\n", -anPath.free_space_loss, anPath.TxRxDistance);

	// --------------------
	// print frequency value
	fprintf(fd,"set label \"frequency\\n%.0f MHz\" at 570,700 center front font \"Verdiana,16\" tc rgb \"#000000\"\n", anPath.lrPar.frq_mhz);
	// --------------------
	
	fprintf(fd,"set label \"%.0f dBm\" at 356,438 left front font \"Verdiana,16\" tc rgb \"#000000\"\n", Eirp);
	fprintf(fd,"set label \"%.0f dBi\" at 390,620 left front font \"Verdiana,16\" tc rgb \"#000000\"\n", TxAntGain);
	fprintf(fd,"set label \"%.0f dBi\" at 770,620 right front font \"Verdiana,16\" tc rgb \"#000000\"\n", RxAntGain);
	fprintf(fd,"set label \"%.0f dB\" at 244,492 center front font \"Verdiana,16\" tc rgb \"#000000\"\n", TxCableLoss);
	fprintf(fd,"set label \"%.0f dB\" at 884,492 center front font \"Verdiana,16\" tc rgb \"#000000\"\n", RxCableLoss);
	fprintf(fd,"set label \"%.0f dBm\" at 90,620 center front font \"Verdiana,16\" tc rgb \"#000000\"\n", TxPower);
	// print margin and sensitivity
	if(Margin >= 0.0)
	{
		fprintf(fd,"set label \"%.0f dB\" at 1060,198 left front font \"Verdiana,20\" tc rgb \"#000000\"\n", Margin);
		fprintf(fd,"set label \"%.0f dBm\" at 1075,135 left front font \"Verdiana,16\" tc rgb \"#000000\"\n", RxSensitivity);
	}
	else
	{
		fprintf(fd,"set label \"%.0f dB\" at 1060,320 left front font \"Verdiana,20\" tc rgb \"#000000\"\n", Margin);
		fprintf(fd,"set label \"%.0f dBm\" at 1075,400 left front font \"Verdiana,16\" tc rgb \"#000000\"\n", RxSensitivity);
	}

	fprintf(fd,"replot\n");
	fprintf(fd,"unset multiplot\n");

	fclose(fd);

	sprintf(cmd, "gnuplot %s", fn_power_gp);
	x=(system(cmd));

	if (x!=-1)
	{
		if (gpsav==0)
		{
			unlink(fn_splat_gp);
			unlink(fn_profile_gp);
			unlink(fn_reference_gp);
		}

		fprintf(stdout, "Power plot written to: \"%s\"\n",graph_pow_name);
		fflush(stdout);
	}

	else
	fprintf(stderr,"\n*** ERROR: Error occurred invoking gnuplot!\n");
	// end of graph design
	// ===========================================================

	if (x!=-1 && gpsav==0)
	unlink(fn_profile_gp);

	logUsr("END PathReport[%S]\n", report_name);
}		// end of GeneratePathReport()

// ---------------------------------------------------------------------

void ObstructionAnalysis(
		site_data TxSite,
		site_data RxSite,
		double frq_mhz,
		FILE *fPathReport,
		FILE *fPathReportRed
		)
{
	// Perform an obstruction analysis along the path between receiver and transmitter.

	int i;

	double WaveLength=0.0;

	char string_f1[255]="";

	if(frq_mhz)
	{
		WaveLength=SpeedLightAir_foot_sec/(frq_mhz*1e6);
	}

	if (clutter>0.0)
	{
		fprintf(fPathReport,"Terrain has been raised by");
		fprintf(fPathReportRed,"Terrain has been raised by");

		if (metric)
		{
			fprintf(fPathReport," %.0f m",fo2Meters(clutter) );
			fprintf(fPathReportRed," %.0f m",fo2Meters(clutter) );
		}
		else
		{
			fprintf(fPathReport," %.2f ft",clutter);
			fprintf(fPathReportRed," %.2f ft",clutter);
		}

		fprintf(fPathReport," to account for ground clutter.\n\n");
		fprintf(fPathReportRed," to account for ground clutter.\n\n");
	}

	// At each point along the path calculate the cosine of a sort of "inverse
	// elevation angle" at the receiver.
	// From the antenna, 0 deg. looks at the ground, and 90 deg. is parallel to the ground.

	// Start at the receiver.
	// If this is the lowest antenna, then terrain obstructions will be nearest to it.
	// (Plus, that's the way rfprobe's original los() did it.)

	// Calculate cosines only.
	// That's sufficient to compare angles and it saves the extra computational burden of acos().
	// However, note the inverted comparison: if acos(A) > acos(B), then B > A.

	if(anPath.NumObstructions)
	{
		fprintf(fPathReport,"N. of obstructions detected: %i\n", anPath.NumObstructions);
		fprintf(fPathReportRed,"N. of obstructions detected: %i\n", anPath.NumObstructions);
	}
	else
	{
		fprintf(fPathReport,"No obstructions to LOS path due to terrain were detected.\n");
		fprintf(fPathReportRed,"No obstructions to LOS path due to terrain were detected.\n");
	}

	if(anPath.NumObstructions)
	{
		fprintf(fPathReport,"Coordinates              distance (km), height (m) above sea level:\n");
		fprintf(fPathReportRed,"Coordinates              distance (km), height (m) above sea level:\n");
		for(i=0;i<anPath.NumObstructions;i++)
		{
			fprint_deg(fPathReport, anPath.obstruction[i].lat, anPath.obstruction[i].lon);
			fprintf(fPathReport, ", ");
			fprintf(fPathReport,"% 14.1f, % 14.1f\n",
				anPath.obstruction[i].distance,				// distance (km)
				anPath.obstruction[i].height);				// height (m) above sea level

			fprint_deg(fPathReportRed, anPath.obstruction[i].lat, anPath.obstruction[i].lon);
			fprintf(fPathReportRed, ", ");
			fprintf(fPathReportRed,"% 14.1f, % 14.1f\n",
				anPath.obstruction[i].distance,				// distance (km)
				anPath.obstruction[i].height);				// height (m) above sea level
		}
	}

#ifdef	EN_PRINTREP_ANTENNARAISE
	//--------------------------
	if(anPath.RxAntennaRaiseAboveGround>0.0)
	{
		if (metric)
		{
			snprintf(string,150,"\nAntenna at %s must be raised to at least %.0f m above groud level to clear all obstructions detected.\n",
						anPath.Rx.SiteName,
						anPath.RxAntennaRaiseAboveGround));
		}
		else
		{
			snprintf(string,150,"\nAntenna at %s must be raised to at least %.2f ft above groud level to clear all obstructions detected.\n",
						anPath.Rx.SiteName,
						anPath.RxAntennaRaiseAboveGround);
		}
		fprintf(fPathReport,"%s",string);
		fprintf(fPathReportRed,"%s",string);
	}
	//--------------------------
#endif

	if(frq_mhz)
	{
#ifdef	EN_PRINTREP_ANTENNARAISE
		//--------------------------
		// modify 04/07
		if(anPath.RxAntennaRaiseAboveGroundPercFresnel>0.0)
		{
			if (metric)
			{
				snprintf(string_fpt6,150,"\nAntenna at %s must be raised to at least %.0f m above groud level to clear %.0f%c of the first Fresnel zone.\n",
							RxSite.name,
							anPath.RxAntennaRaiseAboveGroundPercFresnel,37);
			}
			else
			{
				snprintf(string_fpt6,150,"\nAntenna at %s must be raised to at least %.2f ft above groud level to clear %.0f%c of the first Fresnel zone.\n",
							RxSite.name,
							anPath.RxAntennaRaiseAboveGroundPercFresnel,37);
			}
		}
		else
		{
			snprintf(string_fpt6,150,"\n%.0f%c of the first Fresnel zone is clear.\n",
						fzone_clearance*100.0,37);
		}

#elif defined (CHK_FRESNEL_CLEAR)
		snprintf(string_fpt6,150,"\n");
		// original:
		// if (h_r_fpt6<=h_r_orig)
		// modify mr 10/08
		if(anPath.RxAntennaRaiseAboveGroundPercFresnel<=0.0)
		{
			snprintf(string_fpt6,150,"\n%.0f%c of the first Fresnel zone is clear.\n",
						fzone_clearance*100.0,37);
		}
		//--------------------------
#endif

#ifdef	EN_PRINTREP_ANTENNARAISE
		//--------------------------
		// modify 04/07
		if (h_r_f1>h_r_orig)
		{
			if (metric)
			{
				snprintf(string_f1,150,"\nAntenna at %s must be raised to at least %.0f m above groud level to clear the first Fresnel zone.\n",
							RxSite.name, fo2Meters(h_r_f1-GetElevation(RxSite)-earthRadiusFoot));
			}
			else
			{
				snprintf(string_f1,150,"\nAntenna at %s must be raised to at least %.2f ft above groud level to clear the first Fresnel zone.\n",
							RxSite.name, h_r_f1-GetElevation(RxSite)-earthRadiusFoot);
			}
		}
		else
		{
			snprintf(string_f1,150,"\nThe first Fresnel zone is clear.\n");
		}

#elif defined (CHK_FRESNEL_CLEAR)
		snprintf(string_f1,150,"\n");

		if(anPath.NumObstructions==0)
		{
			snprintf(string_f1,150,"\nThe first Fresnel zone is clear.\n");
		}

		//--------------------------
#endif

	}			// if(frq_mhz)


	if(frq_mhz)
	{

		// Antenna at ... must be raised to at least ... to clear 60% of the first Fresnel zone
		fprintf(fPathReport,"%s",string_f1);
		fprintf(fPathReportRed,"%s",string_f1);
	}
}			// end of ObstructionAnalysis()

void GeneratePathReport(site_data TxSite, site_data RxSite, char *name, char fDrawGraph)
{
	// This function writes a rfprobe Path Report (name.txt) to the filesystem.
	// If (fDrawGraph == 1), then gnuplot is invoked to generate an appropriate
	// output file indicating the ITM model loss between the source and
	// RxSite locations.
	// "filename" is the name assigned to the output file generated by gnuplot.
	// The filename extension is used to set gnuplot's terminal setting and output file type.
	// If no extension is found, .png is assumed.

	int	x;
	int y;
	int z;
	char basename[255];
	char term[30];
	char ext[15];
	char report_name[80];
	char propstring[20];
	double maxloss=-100000.0;
	double minloss=100000.0;
	double path_loss;
	double angle1;
	double angle2;

	double eirp=0.0;			// EIRP: equivalent isotropically radiated power (effective isotropically radiated power)
	double dBm;
	FILE *fd=NULL;
	FILE *fPathReport=NULL;			// full report
	FILE *fPathReportRed=NULL;		// reduced report

	// call GeneratePathAnalysis for analysis
	GeneratePathAnalysis(TxSite, RxSite);		// PathAnalysis for analysis report PathReport

	// name of report in the output directory

	snprintf(fn_pathreport, PATH_MAX, "%s/%s_%s.txt",
					PathOutputDir,
					anPath.Tx.SiteName,
					anPath.Rx.SiteName);
	strcpy(report_name, fn_pathreport);
	printf("Path Report Name: [%s]\n", fn_pathreport);
	printf("Path Report Name: [%s]\n", report_name);

	// name of reduced report in the output directory

	snprintf(fn_pathreport_red, PATH_MAX, "%s/%s_%s_red.txt",
					PathOutputDir,
					anPath.Tx.SiteName,
					anPath.Rx.SiteName);

	CleanFileName(report_name);
	fPathReport=fopenFullPath(fn_pathreport,"w");
	fPathReportRed=fopenFullPath(fn_pathreport_red,"w");

	// start report
	//
	fprintf(fPathReport,"\nPath Analysis from %s to %s\n", anPath.Tx.SiteName, anPath.Rx.SiteName);
	fprintf(fPathReport,"%s\n",dashes);
	
	// distance from tx to rx:
	if (metric)
	{
		fprintf(fPathReport,"Distance between %s and %s: %.2f km",
			anPath.Tx.SiteName, anPath.Rx.SiteName,
			anPath.TxRxDistance
			);
	}
	else
	{
		fprintf(fPathReport,"Distance from %s to %s: %.2f miles",
			anPath.Tx.SiteName, anPath.Rx.SiteName,
			anPath.TxRxDistance
			);
	}
	fprintf(fPathReport,"\n\n");

	fprintf(fPathReport,"Transmitter site: %s\n",anPath.Tx.SiteName);

	fprintf(fPathReport,"Site location: ");
	fprint_deg(fPathReport, anPath.Tx.lat, anPath.Tx.lon);
	fprintf(fPathReport, " ");
	fprint_dms(fPathReport, anPath.Tx.lat, anPath.Tx.lon);
	fprintf(fPathReport,"\n");

	if (metric)
	{
		fprintf(fPathReport,"Elevation: %.0f m above sea level\n", anPath.Tx.Elevation);
		fprintf(fPathReport,"Antenna height: %.0f m above ground / %.0f m above sea level\n",
				anPath.Tx.AntennaHground,
				anPath.Tx.AntennaHSea
				);
	}
	else
	{
		fprintf(fPathReport,"Elevation: %.2f feet above sea level\n", anPath.Rx.Elevation);
		fprintf(fPathReport,"Antenna height: %.2f feet above ground / %.2f feet above sea level\n",
				anPath.Tx.AntennaHground,
				anPath.Tx.AntennaHSea
				);
	}

#ifdef EN_PRINTREP_HAAT
	if(anPath.Tx.HAAT_AntHeightAboveAverageTerrain>NO_LAND_HEIGHT)
	{
		if (metric)
		fprintf(fPathReport,"Antenna height above average terrain: %.0f m\n",
				anPath.Tx.HAAT_AntHeightAboveAverageTerrain);
		else
		fprintf(fPathReport,"Antenna height above average terrain: %.2f feet\n",
				anPath.Tx.HAAT_AntHeightAboveAverageTerrain);
	}
#endif

	angle1 = anPath.Tx.ElevationAngle;
	angle2 = anPath.Tx.AngleObstruction1;

	fprintf(fPathReport,"Azimuth to %s: %.2f degrees\n",
			anPath.Rx.SiteName,
			anPath.Tx.azimuth);
	if (angle1>=0.0)
	{
		fprintf(fPathReport,"Elevation angle to %s: %+.4f degrees\n",
			anPath.Rx.SiteName,
			anPath.Tx.ElevationAngle);
	}
	else
	{
		fprintf(fPathReport,"Depression angle to %s: %+.4f degrees\n",
			anPath.Rx.SiteName,
			anPath.Tx.ElevationAngle);
	}
	if ((angle2-angle1)>0.0001)
	{
		if (angle2<0.0)
		{
			fprintf(fPathReport,"Depression");
		}
		else
		{
			fprintf(fPathReport,"Elevation");
		}
		fprintf(fPathReport," angle to the first obstruction: %+.4f degrees\n",
			anPath.Tx.AngleObstruction1);
	}
	fprintf(fPathReport,"\n\n");		// fprintf(fPathReport,"\n%s\n\n",dashes);

	// Receiver
	fprintf(fPathReport,"Receiver site: %s\n", anPath.Rx.SiteName);
	fprintf(fPathReport,"Site location: ");
	fprint_deg(fPathReport, anPath.Rx.lat, anPath.Rx.lon);
	fprintf(fPathReport, " ");
	fprint_dms(fPathReport, anPath.Rx.lat, anPath.Rx.lon);
	fprintf(fPathReport,"\n");

	if (metric)
	{
		fprintf(fPathReport,"Elevation: %.0f m above sea level\n", anPath.Rx.Elevation);
		fprintf(fPathReport,"Antenna height: %.0f m above ground / %.0f m above sea level\n",
				anPath.Rx.AntennaHground,
				anPath.Rx.AntennaHSea);
	}
	else
	{
		fprintf(fPathReport,"Elevation: %.2f feet AMSL\n", anPath.Rx.Elevation);
		fprintf(fPathReport,"Antenna height: %.2f feet AGL / %.2f feet AMSL\n",
				anPath.Rx.AntennaHground,
				anPath.Rx.AntennaHSea);
	}

#ifdef EN_PRINTREP_HAAT
	if(anPath.Rx.HAAT_AntHeightAboveAverageTerrain>NO_LAND_HEIGHT)
	{
		if (metric)
		fprintf(fPathReport,"Antenna height above average terrain: %.0f m\n", anPath.Rx.HAAT_AntHeightAboveAverageTerrain);
		else
		fprintf(fPathReport,"Antenna height above average terrain: %.2f feet\n", anPath.Rx.HAAT_AntHeightAboveAverageTerrain);
	}
#endif

	angle1 = anPath.Rx.ElevationAngle;
	angle2 = anPath.Rx.AngleObstruction1;
	fprintf(fPathReport,"Azimuth to %s: %.2f degrees\n", anPath.Tx.SiteName, anPath.Rx.azimuth);
	if (angle1>=0.0)
	{
		fprintf(fPathReport,"Elevation angle to %s: %+.4f degrees\n", anPath.Tx.SiteName, angle1);
	}
	else
	{
		fprintf(fPathReport,"Depression angle to %s: %+.4f degrees\n", anPath.Tx.SiteName, angle1);
	}
	if ((angle2-angle1)>0.0001)
	{
		if (angle2<0.0)
		{
			fprintf(fPathReport,"Depression");
		}
		else
		{
			fprintf(fPathReport,"Elevation");
		}
		fprintf(fPathReport," angle to the first obstruction: %+.4f degrees\n",angle2);
	}
	fprintf(fPathReport,"\n\n");		// fprintf(fPathReport,"\n%s\n\n",dashes);

	// analysis in base of model type
	if (anPath.lrPar.frq_mhz>0.0)
	{
		if(anPath.PropagationModel==PROP_MODEL_LRICE)
		{
			fprintf(fPathReport,"Analysis model: Longley-Rice\n");
		}
		else
		{
			fprintf(fPathReport,"Analysis model: ITWOM Version %.1f\n",ITWOMVersion());
		}
		fprintf(fPathReport,"Parameters used in this analysis:\n");
		fprintf(fPathReport,"Earth's Dielectric Constant: %.3lf\n", anPath.lrPar.EarthDielectricConst);
		fprintf(fPathReport,"Earth's Conductivity: %.3lf Siemens/meter\n", anPath.lrPar.EarthConductivity);
		fprintf(fPathReport,"Atmospheric Bending Constant (N-units): %.3lf ppm\n", anPath.lrPar.AtmBendingCons);
		fprintf(fPathReport,"Frequency: %.0lf MHz\n", anPath.lrPar.frq_mhz);
		fprintf(fPathReport,"Radio Climate: %d (", anPath.lrPar.radio_climate);

		switch (anPath.lrPar.radio_climate)
		{
		case CLIMA_EQUATORIAL				: fprintf(fPathReport,"Equatorial");					break;
		case CLIMA_CONTINENTAL_SUBTROPICAL	: fprintf(fPathReport,"Continental Subtropical");		break;
		case CLIMA_MARITIME_SUBTROPICAL		: fprintf(fPathReport,"Maritime Subtropical");			break;
		case CLIMA_DESERT					: fprintf(fPathReport,"Desert");						break;
		case CLIMA_CONTINENTAL_TEMPERATE	: fprintf(fPathReport,"Continental Temperate");			break;
		case CLIMA_MARITIME_TEMPERATE_LAND	: fprintf(fPathReport,"Martitime Temperate, Over Land");break;
		case CLIMA_MARITIME_TEMPERATE_SEA	: fprintf(fPathReport,"Maritime Temperate, Over Sea");	break;
		default:
			fprintf(fPathReport,"Unknown");
		}

		fprintf(fPathReport,")\nPolarization: %d (", anPath.lrPar.polarization);
		if (anPath.lrPar.polarization==POL_HORIZONTAL)
		{
			fprintf(fPathReport,"Horizontal");
		}
		if (anPath.lrPar.polarization==POL_VERTICAL)
		{
			fprintf(fPathReport,"Vertical");
		}
		fprintf(fPathReport,")\nFraction of Situations: %.1lf%c\n", anPath.lrPar.FractionSituations*100.0,37);
		fprintf(fPathReport,"Fraction of Time: %.1lf%c\n", anPath.lrPar.FractionTime*100.0,37);

		if (anPath.lrPar.EffRadiatedPower!=0.0)
		{
			// calculate Transmitter Effective Radiated Power in Watts or dBm, https://en.wikipedia.org/wiki/Effective_radiated_power
			fprintf(fPathReport,"Transmitter ERP: ");
			if (anPath.lrPar.EffRadiatedPower<1.0)
			{
				fprintf(fPathReport,"%.1lf milliwatts",1000.0* anPath.lrPar.EffRadiatedPower);
			}
			if (anPath.lrPar.EffRadiatedPower>=1.0 && anPath.lrPar.EffRadiatedPower<10.0)
			{
				fprintf(fPathReport,"%.1lf Watts", anPath.lrPar.EffRadiatedPower);
			}
			if (anPath.lrPar.EffRadiatedPower>=10.0 && anPath.lrPar.EffRadiatedPower<10.0e3)
			{
				fprintf(fPathReport,"%.0lf Watts", anPath.lrPar.EffRadiatedPower);
			}
			if (anPath.lrPar.EffRadiatedPower>=10.0e3)
			{
				fprintf(fPathReport,"%.3lf kilowatts", anPath.lrPar.EffRadiatedPower/1.0e3);
			}
			dBm=10.0*(log10(anPath.lrPar.EffRadiatedPower*1000.0));
			fprintf(fPathReport," (%+.2f dBm)\n",dBm);

			// calc EIRP: equivalent isotropically radiated power (effective isotropically radiated power)
			fprintf(fPathReport,"Transmitter EIRP: ");
			eirp= anPath.lrPar.EffRadiatedPower*1.636816521;
			if (eirp<1.0)
			{
				fprintf(fPathReport,"%.1lf milliwatts",1000.0*eirp);
			}
			if (eirp>=1.0 && eirp<10.0)
			{
				fprintf(fPathReport,"%.1lf Watts",eirp);
			}
			if (eirp>=10.0 && eirp<10.0e3)
			{
				fprintf(fPathReport,"%.0lf Watts",eirp);
			}
			if (eirp>=10.0e3)
			{
				fprintf(fPathReport,"%.3lf kilowatts",eirp/1.0e3);
			}
			dBm=10.0*(log10(eirp*1000.0));
			fprintf(fPathReport," (%+.2f dBm)\n",dBm);
		}
		fprintf(fPathReport,"\n\n");		// fprintf(fPathReport,"\n%s\n\n",dashes);

		fprintf(fPathReport,"Summary for the link between %s and %s:\n\n",TxSite.name, RxSite.name);
		if (anPath.Tx.patterndB!=0.0)
		{
			fprintf(fPathReport,"%s antenna pattern towards %s: %.3f (%.2f dB)\n",
				anPath.Tx.SiteName,
				anPath.Rx.SiteName,
				anPath.Tx.pattern,
				anPath.Tx.patterndB);
		}

		// distance=Distance(TxSite,RxSite);
		logUsr("PathReport Distance\n");
		if (anPath.TxRxDistance!=0.0)
		{
			// free_space_loss=36.6+(20.0*log10(LR.frq_mhz))+(20.0*log10(distance));
			fprintf(fPathReport,"Free space path loss: %.2f dB\n",
					anPath.free_space_loss);
			// 12/07/2016: added also in reduced report
			fprintf(fPathReportRed,"Free space path loss: %.2f dB\n",
					anPath.free_space_loss);
		}

		path_loss = 0.0;
		if(anPath.PropagationModel==PROP_MODEL_LRICE)
		{
			// Longley-Rice path loss
			path_loss = anPath.lrice.path_loss;
			fprintf(fPathReport,"Longley-Rice path loss: %.2f dB\n",path_loss);
		}
		else
		{
			// ITWOM path loss
			path_loss = anPath.itwom.path_loss;
			fprintf(fPathReport,"ITWOM Version %.1f path loss: %.2f dB\n",ITWOMVersion(),path_loss);
		}

		if (anPath.free_space_loss!=0.0)
		{
			fprintf(fPathReport,"Attenuation due to terrain shielding: %.2f dB\n",
				anPath.TerrainShieldingAttenuation);
		}
		if (anPath.Tx.patterndB!=0.0)
		{
			fprintf(fPathReport,"Total path loss including %s antenna pattern: %.2f dB\n",
				anPath.Tx.SiteName,
				anPath.PathLossWithTxAntennaPattern);
		}
		if (anPath.lrPar.EffRadiatedPower!=0.0)
		{
			fprintf(fPathReport,"Field strength at %s: %.2f dBuV/meter\n", anPath.Rx.SiteName, anPath.FieldStrength);
			fprintf(fPathReport,"Signal power level at %s: %+.2f dBm\n", RxSite.name, anPath.SignalPowerLevel);
			fprintf(fPathReport,"Signal power density at %s: %+.2f dBW per square meter\n", anPath.Rx.SiteName, anPath.SignalPowerDensity);
			fprintf(fPathReport,"Voltage across a 50 ohm dipole at %s: %.2f uV (%.2f dBuV)\n",
					anPath.Rx.SiteName,
					anPath.VoltageAcross50OhmDipole_uV,
					anPath.VoltageAcross50OhmDipole_dBuV);

			fprintf(fPathReport,"Voltage across a 75 ohm dipole at %s: %.2f uV (%.2f dBuV)\n",
					anPath.Rx.SiteName,
					anPath.VoltageAcross75OhmDipole_uV,
					anPath.VoltageAcross75OhmDipole_dBuV);
		}

		fprintf(fPathReport,"Mode of propagation: ");
		if(anPath.PropagationModel==PROP_MODEL_LRICE)
		{
			fprintf(fPathReport,"%s\n", anPath.strmode);
			fprintf(fPathReport,"Longley-Rice model error code: %d", anPath.ErrorCode);
		}
		else
		{
			if (strcmp(anPath.strmode,"L-o-S")==0)
			{
				fprintf(fPathReport,"Line of Sight\n");
			}
			if (strncmp(anPath.strmode,"1_Hrzn",6)==0)
			{
				fprintf(fPathReport,"Single Horizon ");
			}
			if (strncmp(anPath.strmode,"2_Hrzn",6)==0)
			{
				fprintf(fPathReport,"Double Horizon ");
			}

			y=strlen(anPath.strmode);
			if (y>19)
			{
				y=19;
			}
			for (x=6; x<y; x++)
			{
				propstring[x-6]=anPath.strmode[x];
			}
			propstring[x]=0;

			if (strncmp(propstring,"_Diff",5)==0)
			{
				fprintf(fPathReport,"Diffraction Dominant\n");
			}
			if (strncmp(propstring,"_Tropo",6)==0)
			{
				fprintf(fPathReport,"Troposcatter Dominant\n");
			}
			if (strncmp(propstring,"_Peak",5)==0)
			{
				fprintf(fPathReport,"RX at Peak Terrain Along Path\n");
			}
			fprintf(fPathReport,"ITWOM model error code: %d",anPath.ErrorCode);
		}
		logUsr("PathReport errnum\n");

		fprintf(fPathReport,"\nErrorMessage[%i]:\n",anPath.ErrorCode);
		fprintf(fPathReport,"  \"%s\"\n", anPath.ErrorMsg);
		fprintf(fPathReport,"\n\n");		// fprintf(fPathReport,"\n%s\n\n",dashes);
	}

	fprintf(stdout, "\n");
	fprintf(stdout, "Path Loss Report written to: \"%s\"\n",report_name);
	fflush(stdout);

	ObstructionAnalysis(TxSite, RxSite, anPath.lrPar.frq_mhz, fPathReport, fPathReportRed);

	fclose(fPathReport);			// close full report
	fclose(fPathReportRed);			// close partial report

	// Skip plotting the graph if ONLY a path-loss report is needed.

	if(fDrawGraph)
	{
		if (name[0]=='.')
		{
			// Default filename and output file type

			strncpy(basename,"profile\0",8);
			strncpy(term,"png\0",4);
			strncpy(ext,"png\0",4);
		}

		else
		{
			// Extract extension and terminal type from "name"

			ext[0]=0;
			y=strlen(name);
			strncpy(basename,name,254);

			for (x=y-1; x>0 && name[x]!='.'; x--);

			if (x>0)  	// Extension found
			{
				for (z=x+1; z<=y && (z-(x+1))<10; z++)
				{
					ext[z-(x+1)]=tolower(name[z]);
					term[z-(x+1)]=name[z];
				}

				ext[z-(x+1)]=0;  // Ensure an ending 0
				term[z-(x+1)]=0;
				basename[x]=0;
			}
		}

		if (ext[0]==0)	// No extension -- Default is png
		{
			strncpy(term,"png\0",4);
			strncpy(ext,"png\0",4);
		}

		// Either .ps or .postscript may be used as an extension for postscript output.

		if (strncmp(term,"postscript",10)==0)
		strncpy(ext,"ps\0",3);

		else if (strncmp(ext,"ps",2)==0)
		strncpy(term,"postscript enhanced color\0",26);

		fd=fopen(fn_splat_gp,"w");

		fprintf(fd,"set grid\n");
		fprintf(fd,"set yrange [%2.3f to %2.3f]\n", minloss, maxloss);
		fprintf(fd,"set encoding iso_8859_1\n");
		fprintf(fd,"set term %s truecolor size %i,%i\n",term, 1280, 768);
		fprintf(fd,"set title \"%s Loss Profile Along Path Between %s and %s (%.2f%c azimuth)\"\n",program_name, RxSite.name, TxSite.name, AzimuthSites(RxSite,TxSite),176);

		if (metric)
		fprintf(fd,"set xlabel \"From %s to %s : %.2f km\"\n",RxSite.name,TxSite.name, mi2Km(Distance(RxSite,TxSite)) );
		else
		fprintf(fd,"set xlabel \"From %s to %s : %.2f miles\"\n",RxSite.name,TxSite.name,Distance(RxSite,TxSite));

		if (got_azimuth_pattern || got_elevation_pattern)
		fprintf(fd,"set ylabel \"Total Path Loss (including TX antenna pattern) (dB)");
		else
		{
			if(PropagationModel==PROP_MODEL_LRICE)
			fprintf(fd,"set ylabel \"Longley-Rice Path Loss (dB)");
			else
			fprintf(fd,"set ylabel \"ITWOM Version %.1f Path Loss (dB)",ITWOMVersion());
		}

		fprintf(fd,"\"\nset output \"%s/%s.%s\"\n",PathCurrDir,basename,ext);
		fprintf(fd,"plot \"%s/profile.gp\" title \"Path Loss\" with lines\n",PathCurrDir);

		fclose(fd);

		x=GenerateGraphGnuplot();

		if (x!=-1)
		{
			if (gpsav==0)
			{
				unlink(fn_splat_gp);
				unlink(fn_profile_gp);
				unlink(fn_reference_gp);
			}

			fprintf(stdout, "Path loss plot written to: \"%s.%s\"\n",basename,ext);
			fflush(stdout);
		}

		else
		fprintf(stderr,"\n*** ERROR: Error occurred invoking gnuplot!\n");
	}

	if (x!=-1 && gpsav==0)
	unlink(fn_profile_gp);

	logUsr("END PathReport[%S]\n", report_name);
}		// end of GeneratePathReport()

// ---------------------------------------------------------------------

void SiteReport(site_data TxSite)
{
	char report_name[80];
	double terrain;
	int	x;
	int azi;
	FILE *fd;

	sprintf(report_name,"%s-site_report.txt",TxSite.name);

	CleanFileName(report_name);

	fd=fopenFullPath(report_name,"w");

	fprintf(fd,"\n\t--==[ %s v%s Site Analysis Report For: %s ]==--\n\n",program_name, program_version, TxSite.name);

	fprintf(fd,"%s\n\n",dashes);

	fprintf(fd,"Site location: ");
	fprint_deg(fd, (TxSite.lat), (TxSite.lon) );
	fprintf(fd, " ");
	fprint_dms(fd, (TxSite.lat), (TxSite.lon) );
	fprintf(fd,"\n");

	if (metric)
	{
		fprintf(fd,"Elevation: %.2f meters AMSL\n",fo2Meters(GetElevation(TxSite)));
		fprintf(fd,"Antenna height: %.2f meters AGL / %.2f meters AMSL\n",fo2Meters(TxSite.alt), fo2Meters(TxSite.alt+GetElevation(TxSite)));
	}

	else
	{
		fprintf(fd,"Elevation: %.2f feet AMSL\n",GetElevation(TxSite));
		fprintf(fd,"Antenna height: %.2f feet AGL / %.2f feet AMSL\n",TxSite.alt, TxSite.alt+GetElevation(TxSite));
	}

	terrain=haat(TxSite);

	if(terrain>NO_LAND_HEIGHT)
	{

#ifdef EN_PRINTREP_HAAT
		if (metric)
		fprintf(fd,"Antenna height above average terrain: %.2f meters\n\n",fo2Meters(terrain));
		else
		fprintf(fd,"Antenna height above average terrain: %.2f feet\n\n",terrain);
#endif

		// Display the average terrain between 2 and 10 miles
		// from the transmitter site at azimuths of 0, 45, 90,
		// 135, 180, 225, 270, and 315 degrees.

		for (azi=0; azi<=315; azi+=45)
		{
			fprintf(fd,"Average terrain at %3d degrees azimuth: ",azi);
			terrain=AverageTerrain(TxSite,(double)azi,2.0,10.0);

			if(terrain>NO_LAND_HEIGHT)
			{
				if (metric)
				fprintf(fd,"%.2f meters AMSL\n",fo2Meters(terrain));
				else
				fprintf(fd,"%.2f feet AMSL\n",terrain);
			}

			else
			fprintf(fd,"No terrain\n");
		}
	}

	fprintf(fd,"\n%s\n\n",dashes);
	fclose(fd);
	fprintf(stdout, "\n");
	fprintf(stdout, "Site analysis report written to: \"%s\"\n",report_name);
}

// Inserted mr 05/07
// Use the LoadTopoData algorithm.
// Used to estimate the number of maps that the program load in memory.
// MUST BE OPTIMIZED !!!
//
int chkNMapsUsed(int minlat, int maxlat, int minlon, int maxlon)
{
	int width;
	int cntMaps;			// count Maps to load in memory

	fprintf(stdout, "--- chkNMapsUsed (max_lon, min_lon)[%i][%i]\n", maxlon, minlon);
	fprintf(stdout, "--- chkNMapsUsed (max_lat, min_lat)[%i][%i]\n", maxlat, minlat);
	fflush(stdout);

	width=ReduceAngle(maxlon-minlon);

	cntMaps = (width+1) * (maxlat - minlat + 1);
	fprintf(stdout, "--- chkNMapsUsed (cntMaps)[%i]\n", cntMaps);
	fflush(stdout);

	return(cntMaps);
}


int LoadZoneMaps(int minlat, int maxlat, int minlon, int maxlon)
{
	// This function loads all the maps required to cover the limits of the region specified.

	int lat;
	int y;
	int width;
	int minLongitude;
	int maxLongitude;
	// int NMapsUsed;			// count Maps to load in memory

	// inserted 04/06, disabled 05/06
	// NMapsUsed = CountMapsUsed(maxlon, minlon, maxlat, minlat);

	fprintf(stdout, "--- LoadTopoData (max_lon, min_lon)[%i][%i]\n", maxlon, minlon);
	fprintf(stdout, "--- LoadTopoData (max_lat, min_lat)[%i][%i]\n", maxlat, minlat);
	// fprintf(stdout, "--- Count Maps to load in memory:[%i]\n", NMapsUsed);
	fflush(stdout);

	//---------------------------
	// check if the n. of maps is greater than MAXPAGES
	if((y = chkNMapsUsed(minlat, maxlat, minlon, maxlon)) > MAXPAGES)
	{
		//  n. pages requested is greater than limit
		fprintf(stdout, "--- LoadTopoData  n. pages [%i]>[%i]\n", y, MAXPAGES);
		fflush(stdout);
		return(0);
	}
	//---------------------------

	width=ReduceAngle(maxlon-minlon);

	// load group of maps using LoadSrtmMap
	for (y=0; y<=width; y++)
	{
		for (lat=minlat; lat<=maxlat; lat++)
		{
			if ((maxlon-minlon)<=180.0)
			{
				minLongitude=(int)(minlon+(double)y);
			}
			else
			{
				minLongitude=maxlon+y;
			}
			Degrees2FirstRound(minLongitude);

			maxLongitude=minLongitude+1;
			Degrees2FirstRound(maxLongitude);

			LoadSrtmMap(lat, lat+1, minLongitude, maxLongitude);
		}
	}
	// end load maps
	return(1);				// maps loaded
}

int LoadANO(char *filename)
{
	// This function reads a rfprobe alphanumeric output
	// file (-ani option) for analysis and/or map generation.

	int	error=0;
	int max_lon;
	int min_lon;
	int max_lat;
	int min_lat;
	char string[80];
	char *pointer=NULL;
	double latitude=0.0;
	double longitude=0.0;
	double azimuth=0.0;
	double elevation=0.0;
	double ano=0.0;
	FILE *fd;

	fd=fopenFullPath(filename,"r");

	if (fd!=NULL)
	{
		fgets(string,78,fd);
		pointer=strchr(string,';');

		if (pointer!=NULL)
		*pointer=0;

		sscanf(string,"%d, %d",&max_lon, &min_lon);

		fgets(string,78,fd);
		pointer=strchr(string,';');

		if (pointer!=NULL)
		*pointer=0;

		sscanf(string,"%d, %d",&max_lat, &min_lat);

		fgets(string,78,fd);
		pointer=strchr(string,';');

		if (pointer!=NULL)
		*pointer=0;

		LoadZoneMaps(min_lat, max_lat-1, min_lon, max_lon-1);

		pmsg( stdout, (reading_file), filename);
		fflush(stdout);

		fgets(string,78,fd);
		sscanf(string,"%lf, %lf, %lf, %lf, %lf",&latitude, &longitude, &azimuth, &elevation, &ano);

		while (feof(fd)==0)
		{
			if (LR.EffRadiatedPower==0.0)
			{
				// Path loss

				if (contour_threshold==0 || (fabs(ano)<=(double)contour_threshold))
				{
					ano=fabs(ano);

					if (ano>255.0)
					ano=255.0;

					PutSignal(latitude,longitude,((unsigned char)round(ano)));
				}
			}

			if (LR.EffRadiatedPower!=0.0 && dbm!=0)
			{
				// signal power level in dBm

				if (contour_threshold==0 || (ano>=(double)contour_threshold))
				{
					ano=200.0+rint(ano);

					if (ano<0.0)
					ano=0.0;

					if (ano>255.0)
					ano=255.0;

					PutSignal(latitude,longitude,((unsigned char)round(ano)));
				}
			}

			if (LR.EffRadiatedPower!=0.0 && dbm==0)
			{
				// field strength dBuV/m

				if (contour_threshold==0 || (ano>=(double)contour_threshold))
				{
					ano=100.0+rint(ano);

					if (ano<0.0)
					ano=0.0;

					if (ano>255.0)
					ano=255.0;

					PutSignal(latitude,longitude,((unsigned char)round(ano)));
				}
			}

			fgets(string,78,fd);
			sscanf(string,"%lf, %lf, %lf, %lf, %lf",&latitude, &longitude, &azimuth, &elevation, &ano);
		}

		fclose(fd);

		pmsg( stdout, (sdf_done) );
		fflush(stdout);
	}

	else
	error=1;

	return error;
}

void WriteKML(site_data source, site_data destination)
{
	int	x;
	int y;
	char FoundObstruction;
	char report_name[80];
	double distance;
	double rx_alt;
	double tx_alt;
	double cos_xmtr_angle;
	double azimuth;
	double cos_test_angle;
	double test_alt;
	FILE *fd=NULL;

	ReadPath(source,destination);

	sprintf(report_name,"%s-to-%s.kml",source.name,destination.name);

	CleanFileName(report_name);

	fd=fopenFullPath(report_name,"w");

	fprintf(fd,"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
	fprintf(fd,"<kml xmlns=\"http://earth.google.com/kml/2.0\">\n");
	fprintf(fd,"<!-- Generated by %s Version %s -->\n",program_name, program_version);
	fprintf(fd,"<Folder>\n");
	fprintf(fd,"<name>rfprobe Path</name>\n");
	fprintf(fd,"<open>1</open>\n");
	fprintf(fd,"<description>Path Between %s and %s</description>\n",source.name,destination.name);

	fprintf(fd,"<Placemark>\n");
	fprintf(fd,"    <name>%s</name>\n",source.name);
	fprintf(fd,"    <description>\n");
	fprintf(fd,"       Transmit Site\n");

	if (source.lat>=0.0)
	fprintf(fd,"       <BR>%s North</BR>\n",dec2dms(source.lat));
	else
	fprintf(fd,"       <BR>%s South</BR>\n",dec2dms(source.lat));

	fprintf(fd,"       <BR>%s West</BR>\n",dec2dms(source.lon));

	azimuth=AzimuthSites(source,destination);
	distance=Distance(source,destination);

	if (metric)
	fprintf(fd,"       <BR>%.2f km",mi2Km(distance) );
	else
	fprintf(fd,"       <BR>%.2f miles",distance);

	fprintf(fd," to %s</BR>\n       <BR>toward an azimuth of %.2f%c</BR>\n",destination.name,azimuth,176);

	fprintf(fd,"    </description>\n");
	fprintf(fd,"    <visibility>1</visibility>\n");
	fprintf(fd,"    <Style>\n");
	fprintf(fd,"      <IconStyle>\n");
	fprintf(fd,"        <Icon>\n");
	fprintf(fd,"          <href>root://icons/palette-5.png</href>\n");
	fprintf(fd,"          <x>224</x>\n");
	fprintf(fd,"          <y>224</y>\n");
	fprintf(fd,"          <w>32</w>\n");
	fprintf(fd,"          <h>32</h>\n");
	fprintf(fd,"        </Icon>\n");
	fprintf(fd,"      </IconStyle>\n");
	fprintf(fd,"    </Style>\n");
	fprintf(fd,"    <Point>\n");
	fprintf(fd,"      <extrude>1</extrude>\n");
	fprintf(fd,"      <altitudeMode>relativeToGround</altitudeMode>\n");
	fprintf(fd,"      <coordinates>%f,%f,30</coordinates>\n",FMT_LONGITUDE(source.lon),source.lat);
	fprintf(fd,"    </Point>\n");
	fprintf(fd,"</Placemark>\n");

	fprintf(fd,"<Placemark>\n");
	fprintf(fd,"    <name>%s</name>\n",destination.name);
	fprintf(fd,"    <description>\n");
	fprintf(fd,"       Receive Site\n");

	if (destination.lat>=0.0)
	fprintf(fd,"       <BR>%s North</BR>\n",dec2dms(destination.lat));
	else
	fprintf(fd,"       <BR>%s South</BR>\n",dec2dms(destination.lat));

	fprintf(fd,"       <BR>%s West</BR>\n",dec2dms(destination.lon));

	if (metric)
	fprintf(fd,"       <BR>%.2f km",mi2Km(distance) );
	else
	fprintf(fd,"       <BR>%.2f miles",distance);

	fprintf(fd," to %s</BR>\n       <BR>toward an azimuth of %.2f%c</BR>\n",source.name,AzimuthSites(destination,source),176);

	fprintf(fd,"    </description>\n");
	fprintf(fd,"    <visibility>1</visibility>\n");
	fprintf(fd,"    <Style>\n");
	fprintf(fd,"      <IconStyle>\n");
	fprintf(fd,"        <Icon>\n");
	fprintf(fd,"          <href>root://icons/palette-5.png</href>\n");
	fprintf(fd,"          <x>224</x>\n");
	fprintf(fd,"          <y>224</y>\n");
	fprintf(fd,"          <w>32</w>\n");
	fprintf(fd,"          <h>32</h>\n");
	fprintf(fd,"        </Icon>\n");
	fprintf(fd,"      </IconStyle>\n");
	fprintf(fd,"    </Style>\n");
	fprintf(fd,"    <Point>\n");
	fprintf(fd,"      <extrude>1</extrude>\n");
	fprintf(fd,"      <altitudeMode>relativeToGround</altitudeMode>\n");
	fprintf(fd,"      <coordinates>%f,%f,30</coordinates>\n",FMT_LONGITUDE(destination.lon),destination.lat);
	fprintf(fd,"    </Point>\n");
	fprintf(fd,"</Placemark>\n");

	fprintf(fd,"<Placemark>\n");
	fprintf(fd,"<name>Point-to-Point Path</name>\n");
	fprintf(fd,"  <visibility>1</visibility>\n");
	fprintf(fd,"  <open>0</open>\n");
	fprintf(fd,"  <Style>\n");
	fprintf(fd,"    <LineStyle>\n");
	fprintf(fd,"      <color>7fffffff</color>\n");
	fprintf(fd,"    </LineStyle>\n");
	fprintf(fd,"    <PolyStyle>\n");
	fprintf(fd,"       <color>7fffffff</color>\n");
	fprintf(fd,"    </PolyStyle>\n");
	fprintf(fd,"  </Style>\n");
	fprintf(fd,"  <LineString>\n");
	fprintf(fd,"    <extrude>1</extrude>\n");
	fprintf(fd,"    <tessellate>1</tessellate>\n");
	fprintf(fd,"    <altitudeMode>relativeToGround</altitudeMode>\n");
	fprintf(fd,"    <coordinates>\n");

	for (x=0; x<path.length; x++)
	fprintf(fd,"      %f,%f,5\n",FMT_LONGITUDE(path.lon[x]),path.lat[x]);

	fprintf(fd,"    </coordinates>\n");
	fprintf(fd,"   </LineString>\n");
	fprintf(fd,"</Placemark>\n");

	fprintf(fd,"<Placemark>\n");
	fprintf(fd,"<name>Line-of-Sight Path</name>\n");
	fprintf(fd,"  <visibility>1</visibility>\n");
	fprintf(fd,"  <open>0</open>\n");
	fprintf(fd,"  <Style>\n");
	fprintf(fd,"    <LineStyle>\n");
	fprintf(fd,"      <color>ff00ff00</color>\n");
	fprintf(fd,"    </LineStyle>\n");
	fprintf(fd,"    <PolyStyle>\n");
	fprintf(fd,"       <color>7f00ff00</color>\n");
	fprintf(fd,"    </PolyStyle>\n");
	fprintf(fd,"  </Style>\n");
	fprintf(fd,"  <LineString>\n");
	fprintf(fd,"    <extrude>1</extrude>\n");
	fprintf(fd,"    <tessellate>1</tessellate>\n");
	fprintf(fd,"    <altitudeMode>relativeToGround</altitudeMode>\n");
	fprintf(fd,"    <coordinates>\n");

	// Walk across the "path", indentifying obstructions along the way

	for (y=0; y<path.length; y++)
	{
		distance=mi2Foot(path.distance[y]);
		tx_alt=earthRadiusFoot+source.alt+path.elevation[0];
		rx_alt=earthRadiusFoot+destination.alt+path.elevation[y];

		// Calculate the cosine of the elevation of the
		// transmitter as seen at the temp rx point.

		cos_xmtr_angle=((rx_alt*rx_alt)+(distance*distance)-(tx_alt*tx_alt))/(2.0*rx_alt*distance);

		for (x=y, FoundObstruction=0; x>=0 && FoundObstruction==0; x--)
		{
			distance=mi2Foot( (path.distance[y]-path.distance[x]) );
			test_alt=earthRadiusFoot+path.elevation[x];

			cos_test_angle=((rx_alt*rx_alt)+(distance*distance)-(test_alt*test_alt))/(2.0*rx_alt*distance);

			// Compare these two angles to determine if an obstruction exists. Since
			// we're comparing the cosines of these angles rather than the angles
			// themselves, the following "if" statement is reversed from what it would
			// be if the actual angles were compared.

			if (cos_xmtr_angle>=cos_test_angle)
			FoundObstruction=1;
		}

		if (FoundObstruction)
		fprintf(fd,"      %f,%f,-30\n",FMT_LONGITUDE(path.lon[y]),path.lat[y]);
		else
		fprintf(fd,"      %f,%f,5\n",FMT_LONGITUDE(path.lon[y]),path.lat[y]);
	}

	fprintf(fd,"    </coordinates>\n");
	fprintf(fd,"  </LineString>\n");
	fprintf(fd,"</Placemark>\n");

	fprintf(fd,"    <LookAt>\n");
	fprintf(fd,"      <longitude>%f</longitude>\n",FMT_LONGITUDE(source.lon));
	fprintf(fd,"      <latitude>%f</latitude>\n",source.lat);
	fprintf(fd,"      <range>300.0</range>\n");
	fprintf(fd,"      <tilt>45.0</tilt>\n");
	fprintf(fd,"      <heading>%f</heading>\n",azimuth);
	fprintf(fd,"    </LookAt>\n");

	fprintf(fd,"</Folder>\n");
	fprintf(fd,"</kml>\n");

	fclose(fd);

	fprintf(stdout, "\n");
	fprintf(stdout, "KML file written to: \"%s\"",report_name);

	fflush(stdout);
}


//===============================================================
// main aux functions
//===============================================================

// initialize program name and version
void InitProgramInfo(void)
{
	strncpy(program_version,"1.0.0\0",6);
	if (HD_MODE==1)
	{
		strncpy(program_name,"BotRf HD\0",10);
	}
	else
	{
		strncpy(program_name,"BotRf \0",7);
	}
}

// ------------------------------------------------------
// print help
//
void PrintHelp(void)
{
	int y;

	pmsg( stdout, (available_options) ,program_name, program_version);

	pmsg( stdout, ( t_txsitesqth_max_4 ) );
	pmsg( stdout, ( r_rxsiteqth ) );
	pmsg( stdout, ( c_plot_los_coverage_of_txs ) );
	pmsg( stdout, ( l_plot_path_loss_map_of_tx ) );
	pmsg( stdout, ( s_filenames_of_city ) );
	pmsg( stdout, ( b_filenames_of_cartographic_boundary ) );
	pmsg( stdout, ( p_terrain_profile_graph ) );
	pmsg( stdout, ( e_terrain_elevation_graph ) );
	pmsg( stdout, ( h_terrain_height_graph ) );
	pmsg( stdout, ( h_normalized_terrain_height_graph ) );
	pmsg( stdout, ( l_path_loss_graph ) );
	pmsg( stdout, ( o_topographic_map_to_generate_ppm ) );
	pmsg( stdout, ( u_userdefined_terrain_file ) );
	pmsg( stdout, ( d_sdf_file_directory_path ) );
	pmsg( stdout, ( m_earth_radius_multiplier ) );
	pmsg( stdout, ( n_do_not_plot_los_paths_in_ppm_maps ) );
	pmsg( stdout, ( n_do_not_produce_unnecessary_site ) );
	pmsg( stdout, ( f_frequency_for_fresnel_zone ) );
	pmsg( stdout, ( r_modify_default_range_for_c_or_l ) );
	pmsg( stdout, ( sc_display_smooth_contour_levels ) );
	pmsg( stdout, ( db_threshold_will_not_be_displayed ) );
	pmsg( stdout, ( nf_do_not_plot_fresnel_zones ) );
	pmsg( stdout, ( fz_fresnel_zone_clearance_percentage ) );
	pmsg( stdout, ( gc_ground_clutter_height_feet_meters ) );
	pmsg( stdout, ( ngs_display_greyscale_topography ) );
	pmsg( stdout, ( erp_override_erp_in_lrp_file_watts ) );
	pmsg( stdout, ( ano_name_of_file ) );
	pmsg( stdout, ( ani_name_of_input_file ) );
	pmsg( stdout, ( udt_name_of_user_defined_terrain ) );
	pmsg( stdout, ( kml_generate_google_earth_kml ) );
	pmsg( stdout, ( geo_generate_an_xastir_geo ) );
	pmsg( stdout, ( dbm_plot_signal_power_level ) );
	pmsg( stdout, ( log_copy_command_line_string ) );
	pmsg( stdout, ( gpsav_preserve_gnuplot_files ) );
	pmsg( stdout, ( metric_employ_metric_units ) );
	pmsg( stdout, ( olditm_longleyrice ) );
	pmsg( stdout, ( if_that_flew_by_too_fast ) );

	if (HD_MODE==0)
		pmsg( stdout, (less) ,program_name);
	else
		pmsg( stdout, (hd_less) ,program_name);

	pmsg( stdout, (type_man_or_see_documentation) ,program_name);

	y=(int)sqrt((int)MAXPAGES);

	pmsg( stdout, (compilation_analysis_over_square) ,program_name,y);

	if (y==1)
		pmsg( stdout, (degree) );
	else
		pmsg( stdout, (degree) );

	pmsg( stdout, (of_terrain_default_signal_levels), ITWOMVersion());
	fflush(stdout);

	return;

	//
	// ------------------------------------------------------
}							// end of print help

//===============================================================
// main
//===============================================================

int main(int argc, char *argv[])
{
	int i;
	int	x;
	int y;
	int z=0;

	int min_lat;
	int min_lon;
	int max_lat;
	int max_lon;

	int rxlat;
	int rxlon;
	int txlat;
	int txlon;

	int west_min;
	int west_max;
	int lat_min;
	int lat_max;

	unsigned char coverage=0;
	unsigned char LRmap=0;
	unsigned char terrain_plot=0;
	unsigned char power_plot=0;				// mr: used to generate power plot
	unsigned char elevation_plot=0;
	unsigned char height_plot=0;
	unsigned char map=0;
	unsigned char longley_plot=0;
	unsigned char cities=0;
	unsigned char bfs=0;
	unsigned char txsites=0;
	unsigned char norm=0;
	unsigned char topomap=0;
	unsigned char geo=0;
	unsigned char kml=0;
	unsigned char pt2pt_mode=0;
	unsigned char area_mode=0;
	unsigned char max_txsites;
	unsigned char ngs=0;
	unsigned char nolospath=0;
	unsigned char nositereports=0;
	unsigned char fresnel_plot=1;
	unsigned char command_line_log=0;

	char mapfile[255];
	char header[80];
	char city_file[5][255];
	char elevation_file[255];
	char graph_height_file[255];
	// char graph_power_file[255];
	char longley_file[255];
	char terrain_file[255];
	char string[4096];
	char QthRxFilename[255];
	char *env=NULL;
	char QthTxFilename[255];
	char boundary_file[5][255];
	char udt_file[255];
	char rxsite=0;
	char ani_filename[255];
	char ano_filename[255];
	char ext[20];
	char logfile[255];

	double altitude=0.0;
	double altitudeLR=0.0;
	double tx_range=0.0;
	double rx_range=0.0;
	double deg_range=0.0;
	double deg_limit=0.0;
	double deg_range_lon;

	FILE *fd;

		
#ifdef DBG_ITWOM
	fMr = fopen("fmrlog.txt", "a");
#endif
	
	IniMapsData();			// initialize maps parameters
	
	for(i=0;i<32;i++)
		tx_site[i]=ZeroSiteData;
	rx_site=ZeroSiteData;

	InitProgramInfo();

	strncpy(dashes,"---------------------------------------------------------------------------\0",76);

	if (argc==1)
	{
		// ------------------------------------------------------
		// print help
		// ------------------------------------------------------
		PrintHelp();

		return 1;

		//
		// ------------------------------------------------------

	}							// end of print help

	//----------------------------------------
	// mr 15/05/2016
	InitDirUtility();			// initialize directory utility
	//----------------------------------------

	y=argc-1;

	PropagationModel = PROP_MODEL_LRICE;			// PropagationModel=PROP_MODEL_LRICE invoke Longley-Rice rather than the default ITWOM model
	kml=0;
	geo=0;
	dbm=0;
	gpsav=0;
	metric=0;
	QthRxFilename[0]=0;
	QthTxFilename[0]=0;
	string[0]=0;
	mapfile[0]=0;
	clutter=0.0;
	forced_EffRadiatedPower=-1.0;
	forced_freq=0.0;
	elevation_file[0]=0;
	terrain_file[0]=0;
	sdf_path[0]=0;
	udt_file[0]=0;
	path.length=0;
	max_txsites=30;
	fzone_clearance=0.6;
	contour_threshold=0;
	rx_site.lat=91.0;
	rx_site.lon=361.0;
	longley_file[0]=0;
	ano_filename[0]=0;
	ani_filename[0]=0;
	smooth_contours=0;
	

	sprintf(header,"\nWelcome To %s ver.%s\n\n", program_name, program_version);

	for (x=0; x<4; x++)
	{
		tx_site[x].lat=91.0;
		tx_site[x].lon=361.0;
	}


	// ------------------------------------------------------
	// Scan for command line arguments
	//
	for (x=1; x<=y; x++)
	{
		if (strcmp(argv[x],"-R")==0)
		{
			// modify default range for -c or -L (miles/km)
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				sscanf(argv[z],"%lf",&max_range);

				if (max_range<0.0)
				max_range=0.0;

				if (max_range>1000.0)
				max_range=1000.0;
			}
		}

		if (strcmp(argv[x],"-m")==0)
		{
			// earth radius multiplier
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				sscanf(argv[z],"%lf",&EarthRadiusMultiplier);
				if (EarthRadiusMultiplier<0.1)
				{
					EarthRadiusMultiplier=1.0;
				}
				if (EarthRadiusMultiplier>1.0e6)
				{
					EarthRadiusMultiplier=1.0e6;
				}
				earthRadiusFoot = earthRadiusFoot * EarthRadiusMultiplier;
			}
		}

		if (strcmp(argv[x],"-gc")==0)
		{
			// ground clutter height (feet/meters)
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				sscanf(argv[z],"%lf",&clutter);

				if (clutter<0.0)
				clutter=0.0;
			}
		}

		if (strcmp(argv[x],"-fz")==0)
		{
			// Fresnel zone clearance percentage (default = 60)
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				sscanf(argv[z],"%lf",&fzone_clearance);

				if (fzone_clearance<0.0 || fzone_clearance>100.0)
				fzone_clearance=60.0;

				fzone_clearance/=100.0;
			}
		}

		if (strcmp(argv[x],"-o")==0)
		{
			// filename of topographic map to generate (.ppm)
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			strncpy(mapfile,argv[z],253);
			map=1;
		}

		if (strcmp(argv[x],"-log")==0)
		{
			// copy command line string to this output file
			z=x+1;

			logfile[0]=0;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			strncpy(logfile,argv[z],253);

			command_line_log=1;
		}

		if (strcmp(argv[x],"-udt")==0)
		{
			// name of user defined terrain input file
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			strncpy(udt_file,argv[z],253);
		}

		if (strcmp(argv[x],"-c")==0)
		{
			// plot LOS coverage of TX(s) with RX antenna at X feet/meters AGL
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				sscanf(argv[z],"%lf",&altitude);
				map=1;
				coverage=1;
				area_mode=1;
				max_txsites=4;
			}
		}

		if (strcmp(argv[x],"-db")==0 || strcmp(argv[x],"-dB")==0)
		{
			// threshold beyond which contours will not be displayed
			z=x+1;

			if (z<=y && argv[z][0]) // A minus argument is legal here
			sscanf(argv[z],"%d",&contour_threshold);
		}

		if (strcmp(argv[x],"-p")==0)
		{
			// filename of terrain profile graph to plot
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				strncpy(terrain_file,argv[z],253);
				terrain_plot=1;
				pt2pt_mode=1;
			}
		}

		if (strcmp(argv[x],"-e")==0)
		{
			// filename of terrain elevation graph to plot
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				strncpy(elevation_file,argv[z],253);
				elevation_plot=1;
				pt2pt_mode=1;
			}
		}

		if (strcmp(argv[x],"-pw")==0 || strcmp(argv[x],"-PW")==0)
		{
			// ----------------------------------------------------------
			// -pw : generate power graph
			// ----------------------------------------------------------
			//
			char tmpDir[PATH_MAX];

			z=x+1;
			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				strncpy(graph_pow_file,argv[z],253);
				// strncpy(graph_pow_name,argv[z],253);
				power_plot=1;
			}

			//----------------------------------
			// modify 25/06: get the output dir from graph_power file
		fprintf(stdout,"graph:[%s]\n",graph_pow_file);
			csplitpath(&graph_pow_file[0],
					&PathOutputDir[0],
					NULL,
					&PathFileName[0],
					NULL);
		fprintf(stdout,"graph:[%s]\n",graph_pow_file);
		fprintf(stdout,"outdir:[%s]\n",PathOutputDir);
		fprintf(stdout,"name:[%s]\n",PathFileName);
		fflush(stdout);

			if( (PathOutputDir[0] != '/') && (PathOutputDir[0] != '.') )
			{
			fprintf(stdout,"PathCurrDir[%s]\n",PathCurrDir);
			fflush(stdout);
				snprintf(tmpDir, PATH_MAX, "%s/%s",	PathCurrDir, PathOutputDir);
				strncpy(PathOutputDir, tmpDir, PATH_MAX);
			fprintf(stdout,"if( (PathOutputDir[0] != '/'):[%s]\n",PathOutputDir);
			fflush(stdout);
			}
			else if( PathOutputDir[0] == '.' )
			{
				snprintf(tmpDir, PATH_MAX, "%s/%s",	PathCurrDir, &PathOutputDir[1]);
				strncpy(PathOutputDir, tmpDir, PATH_MAX);
			}

			// full path name
			snprintf(graph_pow_name, PATH_MAX, "%s/%s.png",PathOutputDir, PathFileName);
			fprintf(stdout,"graph_pow_name:[%s]\n",graph_pow_name);
			fflush(stdout);
			snprintf(graph_pow_file, PATH_MAX,"%s/%s.txt",PathOutputDir, PathFileName);
			fprintf(stdout,"graph_pow_file:[%s]\n",graph_pow_file);
			fflush(stdout);
			ReadDataPowerLink(graph_pow_file);
		}

		if (strcmp(argv[x],"-h")==0 || strcmp(argv[x],"-H")==0)
		{
			// ----------------------------------------------------------
			// -h switch: generate a graph illustrating terrain height
			// referenced to a line-of-sight path between the transmitter
			// and receiver.
			// -H switch: a graph plot normalized to the transmitter and receiver
			// antenna heights  can be obtained.
			// ----------------------------------------------------------
			//
			z=x+1;
			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				strncpy(graph_height_file,argv[z],253);
				height_plot=1;
				pt2pt_mode=1;
			}

			if (strcmp(argv[x],"-H")==0)
			{
				norm=1;
			}
			else
			{
				norm=0;
			}

			//----------------------------------
			// modify 25/06: get the output dir from graph_height file
			csplitpath(&graph_height_file[0],
					&PathOutputDir[0],
					NULL,
					NULL,
					NULL);

			if( (PathOutputDir[0] != '/') && (PathOutputDir[0] != '.') )
			{
				snprintf(string, PATH_MAX, "%s/%s",	PathCurrDir, PathOutputDir);
				strncpy(PathOutputDir, string, PATH_MAX);
			}
			else if( PathOutputDir[0] == '.' )
			{
				snprintf(string, PATH_MAX, "%s/%s",	PathCurrDir, &PathOutputDir[1]);
				strncpy(PathOutputDir, string, PATH_MAX);
			}

			//----------------------------------
		}

		if (strcmp(argv[x],"-metric")==0)
		{
			// employ metric rather than imperial units for all user I/O
			metric=1;
		}

		if (strcmp(argv[x],"-gpsav")==0)
		{
			// preserve gnuplot temporary working files after rfprobe execution
			gpsav=1;
		}

		if (strcmp(argv[x],"-geo")==0)
		{
			// generate an Xastir .geo georeference file (with .ppm output)
			geo=1;
		}

		if (strcmp(argv[x],"-kml")==0)
		{
			// generate Google Earth (.kml) compatible output
			kml=1;
		}

		if (strcmp(argv[x],"-nf")==0)
		{
			// do not plot Fresnel zones in height plots
			fresnel_plot=0;
		}

		if (strcmp(argv[x],"-ngs")==0)
		{
			// display greyscale topography as white in .ppm files
			ngs=1;
		}

		if (strcmp(argv[x],"-n")==0)
		{
			// do not plot LOS paths in .ppm maps
			nolospath=1;
		}

		if (strcmp(argv[x],"-dbm")==0)
		{
			// plot signal power level contours rather than field strength
			dbm=1;
		}

		if (strcmp(argv[x],"-sc")==0)
		{
			// display smooth rather than quantized contour levels
			smooth_contours=1;
		}

		if (strcmp(argv[x],"-olditm")==0)
		{
			// invoke older ITM propagation model rather than the newer ITWOM
			PropagationModel=1;
		}

		if (strcmp(argv[x],"-N")==0)
		{
			// do not produce unnecessary site or obstruction reports
			nolospath=1;
			nositereports=1;
		}

		if (strcmp(argv[x],"-d")==0)
		{
			// sdf file directory path (overrides path in ~/.splat_path file)
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				snprintf(sdf_path, PATH_MAX	, "%s/%s",				PathCurrDir, argv[z]);
				logUsr("sdf_path=[%s]\n", sdf_path);
			}
		}

		if (strcmp(argv[x],"-t")==0)
		{
			// Transmitter txsite(s).qth (max of 4 with -c, max of 30 with -L)

			z=x+1;

			while (z<=y && argv[z][0] && argv[z][0]!='-' && txsites<30)
			{
				strncpy(QthTxFilename,argv[z],253);

				logUsr("tx_site[txsites]=LoadQTH(%s)\n", QthTxFilename);

				tx_site[txsites]=LoadQTH(QthTxFilename);
				txsites++;
				z++;
			}

			z--;
		}

		if (strcmp(argv[x],"-L")==0)
		{
			// plot path loss map of TX based on an RX at X feet/meters AGL
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				sscanf(argv[z],"%lf",&altitudeLR);
				map=1;
				LRmap=1;
				area_mode=1;

				if (coverage)
				fprintf(stdout, "c and L are exclusive options, ignoring L.\n");
			}
		}

		if (strcmp(argv[x],"-l")==0)
		{
			// filename of path loss graph to plot
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				strncpy(longley_file,argv[z],253);
				longley_plot=1;
				pt2pt_mode=1;
			}
		}

		if (strcmp(argv[x],"-r")==0)
		{
			// Receiver rxsite.qth

			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				strncpy(QthRxFilename,argv[z],253);

				logUsr("rx_site=LoadQTH(%s)\n", QthRxFilename);

				rx_site=LoadQTH(QthRxFilename);
				rxsite=1;
				pt2pt_mode=1;
			}
		}

		if (strcmp(argv[x],"-s")==0)
		{
			// filename(s) of city/site file(s) to import (5 max)

			z=x+1;

			while (z<=y && argv[z][0] && argv[z][0]!='-' && cities<5)
			{
				strncpy(city_file[cities],argv[z],253);
				cities++;
				z++;
			}

			z--;
		}

		if (strcmp(argv[x],"-b")==0)
		{
			// filename(s) of cartographic boundary file(s) to import (5 max)

			z=x+1;

			while (z<=y && argv[z][0] && argv[z][0]!='-' && bfs<5)
			{
				strncpy(boundary_file[bfs],argv[z],253);
				bfs++;
				z++;
			}

			z--;
		}

		if (strcmp(argv[x],"-f")==0)
		{
			// frequency for Fresnel zone calculation (MHz)
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				sscanf(argv[z],"%lf",&forced_freq);

				if (forced_freq<20.0)
				forced_freq=0.0;

				if (forced_freq>20.0e3)
				forced_freq=20.0e3;
			}
		}

		if (strcmp(argv[x],"-erp")==0)
		{
			// override ERP in .lrp file (Watts)
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			{
				sscanf(argv[z],"%lf",&forced_EffRadiatedPower);

				if (forced_EffRadiatedPower<0.0)
				forced_EffRadiatedPower=-1.0;
			}
		}

		if (strcmp(argv[x],"-ano")==0)
		{
			// name of alphanumeric output file
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			strncpy(ano_filename,argv[z],253);
		}

		if (strcmp(argv[x],"-ani")==0)
		{
			// name of alphanumeric input file
			z=x+1;

			if (z<=y && argv[z][0] && argv[z][0]!='-')
			strncpy(ani_filename,argv[z],253);
		}
	}					// end of Scan for command line arguments

	//-------------------------------------
	// mr 25/06 initialize full path files
	//
	SetFilesWithAbsPath();		// full path of rfprobe files

	// user log utility
	ini_logUsr(LogUsr, "w");

	// open sqlite index map db
	dbidx=DbMapsOpen(fn_dbidx);
	if(dbidx==NULL)
	{
		// problem opening db
		return(1);
	}
	//-------------------------------------


	// ----------------------------------------------------------
	// Perform some error checking on the arguments and switches parsed from
	// the command-line. If an error is encountered, print a message and exit
	// gracefully.

	if (txsites==0)
	{
		logUsr("*** ERROR: No transmitter site(s) specified!\n");
		fprintf(stderr,"\n%c*** ERROR: No transmitter site(s) specified!\n\n",7);
		exit (-1);
	}

	for (x=0, y=0; x<txsites; x++)
	{
		logUsr("txsites: for (x=%i; x<=%i; x++)\n", x, txsites);
		logUsr("tx_site[%i].lat=[%f]\n", x, tx_site[x].lat);
		logUsr("tx_site[%i].lon=[%f]\n", x, tx_site[x].lon);

		if (tx_site[x].lat==91.0 && tx_site[x].lon==361.0)
		{
			logUsr("*** *** ERROR: Transmitter site #%d not found!\n",x+1);
			fprintf(stderr,"\n*** ERROR: Transmitter site #%d not found!",x+1);
			y++;
		}
	}

	if (y)
	{
		logUsr("if (y) exit\n");
		fprintf(stderr,"%c\n\n",7);
		exit (-1);
	}
	logUsr("if ((coverage+LRmap+ani_filename[0])==0 && rx_site.lat==91.0 && rx_site.lon==361.0)\n");

	if ((coverage+LRmap+ani_filename[0])==0 && rx_site.lat==91.0 && rx_site.lon==361.0)
	{
		if (max_range!=0.0 && txsites!=0)
		{
			// Plot topographic map of radius "max_range"

			map=0;
			topomap=1;
		}

		else
		{
			logUsr("*** ERROR: No receiver site found or specified!\n");
			fprintf(stderr,"\n%c*** ERROR: No receiver site found or specified!\n\n",7);
			exit (-1);
		}
	}

	// No major errors were detected.  Whew!  :-)

	// Adjust input parameters if -metric option is used
	logUsr("if (metric)\n");

	if (metric)
	{
		altitudeLR = mt2Foot(altitudeLR);	// meters --> feet
		max_range = Km2Miles(max_range);	// km --> miles
		altitude = mt2Foot(altitude);		// meters --> feet
		clutter = mt2Foot(clutter);			// meters --> feet
	}

	// If no SDF path was specified on the command line (-d), check
	// for a path specified in the $HOME/.splat_path file.  If the
	// file is not found, then sdf_path[] remains NULL, and the
	// current working directory is assumed to contain the SDF
	// files.

	logUsr("if (sdf_path[0]==0)\n");
	if (sdf_path[0]==0)
	{
		env=getenv("HOME");
		snprintf(string,253,"%s/.splat_path",env);
		fd=fopen(string,"r");

		if (fd!=NULL)
		{
			fgets(string,253,fd);

			// Remove <CR> and/or <LF> from string

			for (x=0; string[x]!='\r' && string[x]!='\n' && string[x]!=0 && x<253; x++);
			string[x]=0;

			strncpy(sdf_path,string,253);

			fclose(fd);
		}
	}

	// Ensure a trailing '/' is present in sdf_path

	logUsr("sdf_path prima di inserire / =[%s]\n", sdf_path);
	if (sdf_path[0])
	{
		x=strlen(sdf_path);

		if (sdf_path[x-1]!='/' && x!=0)
		{
			sdf_path[x]='/';
			sdf_path[x+1]=0;
		}
	}

	logUsr("sdf_path2=[%s]\n", sdf_path);

	fprintf(stdout, "%s",header);
	fflush(stdout);

	if (ani_filename[0])
	{
		ReadLRParm(tx_site[0],0); // Get ERP status
		y=LoadANO(ani_filename);

		for (x=0; x<txsites && x<max_txsites; x++)
		PlaceMarker(tx_site[x]);

		if (rxsite)
		PlaceMarker(rx_site);

		if (bfs)
		{
			for (x=0; x<bfs; x++)
			LoadBoundaries(boundary_file[x]);

			fprintf(stdout, "\n");
			fflush(stdout);
		}

		if (cities)
		{
			for (x=0; x<cities; x++)
			LoadCities(city_file[x]);

			fprintf(stdout, "\n");
			fflush(stdout);
		}

		if (LR.EffRadiatedPower==0.0)
		WritePPMLR(mapfile,geo,kml,ngs,tx_site,txsites);
		else
		{
			if (dbm)
			WritePPMDBM(mapfile,geo,kml,ngs,tx_site,txsites);
			else
			WritePPMSS(mapfile,geo,kml,ngs,tx_site,txsites);
		}

		exit(0);
	}

	x=0;
	y=0;

	min_lat=90;
	max_lat=-90;

	min_lon=(int)floor(tx_site[0].lon);
	max_lon=(int)floor(tx_site[0].lon);

	for (y=0, z=0; z<txsites && z<max_txsites; z++)
	{
		// --------------------------------------
		// mr 08/07
		// check the distance limit
		double chkdist;
		chkdist = DistanceKm(tx_site[z], rx_site);
		if(chkdist > 600.0)
		{
			// distance is greater 600Km
			snprintf(string,250,"Distance from %s to %s is greater than 600km.",
					tx_site[z].name, rx_site.name );
			GenerateErrorReport(tx_site[z],rx_site, string);
			return(1);
		}
		// --------------------------------------

		txlat=(int)floor(tx_site[z].lat);
		txlon=(int)floor(tx_site[z].lon);
		UpdateMinMaxCoord(txlat, txlon);
	}

	if (rxsite)
	{
		rxlat=(int)floor(rx_site.lat);
		rxlon=(int)floor(rx_site.lon);
		UpdateMinMaxCoord(rxlat, rxlon);
	}

	//----------------------------------------
	// Load the maps in ram
	logUsr("LoadTopoData (load maps in ram) START\n");
	if(LoadZoneMaps(min_lat, max_lat, min_lon, max_lon) == 0)
	{
		//  n. maps greater than limits
		if (nositereports==0)
		{
			// print report:
			y = chkNMapsUsed(min_lat, max_lat, min_lon, max_lon);

			snprintf(string,250,"To perform this analysis are required %i maps. This exceeds the program limit (%i).",y,MAXPAGES);
			GenerateErrorReport(tx_site[0],rx_site, string);
		}
		return(1);
	}
	logUsr("LoadTopoData 1 end\n");
	//----------------------------------------

	if (area_mode || topomap)
	{
		for (z=0; z<txsites && z<max_txsites; z++)
		{
			// "Ball park" estimates used to load any additional
			// SDF files required to conduct this analysis.

			tx_range=sqrt(1.5*(tx_site[z].alt+GetElevation(tx_site[z])));

			if (LRmap)
			{
				rx_range=sqrt(1.5*altitudeLR);
			}
			else
			{
				rx_range=sqrt(1.5*altitude);
			}

			// deg_range determines the maximum amount of topo data we read

			deg_range=(tx_range+rx_range)/57.0;

			// max_range regulates the size of the
			// analysis.  A small, non-zero amount can
			// be used to shrink the size of the analysis
			// and limit the amount of topo data read by
			// rfprobe.  A large number will increase the
			// width of the analysis and the size of
			// the map.

			if (max_range==0.0)
			{
				max_range=tx_range+rx_range;
			}

			deg_range=max_range/57.0;

			// Prevent the demand for a really wide coverage
			// from allocating more "pages" than are available
			// in memory.

			switch (MAXPAGES)
			{
			case 1: deg_limit=0.125;
				break;

			case 2: deg_limit=0.25;
				break;

			case 4: deg_limit=0.5;
				break;

			case 9: deg_limit=1.0;
				break;

			case 16: deg_limit=1.5;  // WAS 2.0
				break;

			case 25: deg_limit=2.0;  // WAS 3.0
				break;

			case 36: deg_limit=2.5;	 // New!
				break;

			case 49: deg_limit=3.0;  // New!
				break;

			case 64: deg_limit=3.5;  // New!
				break;
			}

			if (fabs(tx_site[z].lat)<70.0)
			{
				deg_range_lon=deg_range/cos(RADIANS(tx_site[z].lat));
			}
			else
			{
				deg_range_lon=deg_range/cos(RADIANS(70.0));
			}

			// Correct for squares in degrees not being square in miles
			if (deg_range>deg_limit)
			{
				deg_range=deg_limit;
			}
			if (deg_range_lon>deg_limit)
			{
				deg_range_lon=deg_limit;
			}

			lat_min=(int)floor(tx_site[z].lat-deg_range);
			lat_max=(int)floor(tx_site[z].lat+deg_range);

			west_min=(int)floor(tx_site[z].lon-deg_range_lon);
			Degrees2FirstRound(west_min);

			west_max=(int)floor(tx_site[z].lon+deg_range_lon);
			Degrees2FirstRound(west_max);

			if (lat_min<min_lat)
			{
				min_lat=lat_min;
			}
			if (lat_max>max_lat)
			{
				max_lat=lat_max;
			}
			if (DeltaLongitude(west_min,min_lon)<0.0)
			{
				min_lon=west_min;
			}
			if (DeltaLongitude(west_max,max_lon)>=0.0)
			{
				max_lon=west_max;
			}
		}

		// Load any additional SDF files, if required

		printf("START LoadTopoData if (area_mode || topomap)\n");

		LoadZoneMaps(min_lat, max_lat, min_lon, max_lon);

		printf("END LoadTopoData\n if (area_mode || topomap)");
	}

	if (udt_file[0])
	{
		LoadUDT(udt_file);
	}

	// ***** begin analysis *****

	if (pt2pt_mode)
	{
		PlaceMarker(rx_site);

		if (terrain_plot)
		{
			// Extract extension (if present) from "terrain_file"

			y=strlen(terrain_file);

			for (x=y-1; x>0 && terrain_file[x]!='.'; x--);

			if (x>0)  // Extension found
			{
				for (z=x+1; z<=y && (z-(x+1))<10; z++)
				ext[z-(x+1)]=tolower(terrain_file[z]);

				ext[z-(x+1)]=0;	    // Ensure an ending 0
				terrain_file[x]=0;  // Chop off extension
			}

			else
			strncpy(ext,"png\0",4);
		}

		if (elevation_plot)
		{
			// Extract extension (if present) from "elevation_file"

			y=strlen(elevation_file);

			for (x=y-1; x>0 && elevation_file[x]!='.'; x--);

			if (x>0)  // Extension found
			{
				for (z=x+1; z<=y && (z-(x+1))<10; z++)
				ext[z-(x+1)]=tolower(elevation_file[z]);

				ext[z-(x+1)]=0;       // Ensure an ending 0
				elevation_file[x]=0;  // Chop off extension
			}

			else
			strncpy(ext,"png\0",4);
		}

		if (height_plot)
		{
			// Extract extension (if present) from "graph_height_file"

			y=strlen(graph_height_file);

			for (x=y-1; x>0 && graph_height_file[x]!='.'; x--);

			if (x>0)  // Extension found
			{
				for (z=x+1; z<=y && (z-(x+1))<10; z++)
				ext[z-(x+1)]=tolower(graph_height_file[z]);

				ext[z-(x+1)]=0;    // Ensure an ending 0
				graph_height_file[x]=0;  // Chop off extension
			}
			else
			{
				strncpy(ext,"png\0",4);
			}
		}

		if (longley_plot)
		{
			// Extract extension (if present) from "longley_file"

			y=strlen(longley_file);

			for (x=y-1; x>0 && longley_file[x]!='.'; x--);

			if (x>0)  // Extension found
			{
				for (z=x+1; z<=y && (z-(x+1))<10; z++)
				ext[z-(x+1)]=tolower(longley_file[z]);

				ext[z-(x+1)]=0;     // Ensure an ending 0
				longley_file[x]=0;  // Chop off extension
			}

			else
			strncpy(ext,"png\0",4);
		}

		for (x=0; x<txsites && x<4; x++)
		{
			PlaceMarker(tx_site[x]);

			// --------------------------------
			// new commands:
			// added 27/07
			if (power_plot)
			{
				printf("START PowerPlot\n");
				ReadLRParm(tx_site[x],0);
				GeneratePowerPlot(tx_site[x],rx_site,string);
				printf("END PowerPlot\n");
				continue;			// to end cycle
			}

			// --------------------------------
			// original commands:
			//
			if (nolospath==0)
			{
				switch (x)
				{
				case 0:
					PlotPath(tx_site[x],rx_site,1);
					break;

				case 1:
					PlotPath(tx_site[x],rx_site,8);
					break;

				case 2:
					PlotPath(tx_site[x],rx_site,16);
					break;

				case 3:
					PlotPath(tx_site[x],rx_site,32);
				}
			}

			if (nositereports==0)
			{
				printf("START SiteReport\n");

				SiteReport(tx_site[x]);

				printf("END SiteReport\n");
			}

			if (kml)
			{
				WriteKML(tx_site[x],rx_site);
			}

			if (txsites>1)
			{
				snprintf(string,250,"%s-%c.%s%c",longley_file,'1'+x,ext,0);
			}
			else
			{
				snprintf(string,250,"%s.%s%c",longley_file,ext,0);
			}

			if (nositereports==0)
			{
				if (longley_file[0]==0)
				{
					printf("START PathReport if (longley_file[0]==0)\n");
	//-------------------------
	// @@@ per debug: forza elevazione
	dbgTxsite = tx_site[x];
	dbgRxsite = rx_site;
	//-------------------------

					ReadLRParm(tx_site[x],0);

					GeneratePathReport(tx_site[x],rx_site,string,0);

					printf("END PathReport if (longley_file[0]==0)\n");
				}

				else
				{
					printf("START PathReport else\n");

	//-------------------------
	dbgTxsite = tx_site[x];
	dbgRxsite = rx_site;
	//-------------------------

					ReadLRParm(tx_site[x],1);
					GeneratePathReport(tx_site[x],rx_site,string,longley_file[0]);

					printf("END PathReport else)\n");
				}
			}

			if (terrain_plot)
			{
				if (txsites>1)
				{
					snprintf(string,250,"%s-%c.%s%c",terrain_file,'1'+x,ext,0);
				}
				else
				{
					snprintf(string,250,"%s.%s%c",terrain_file,ext,0);
				}
				GraphTerrain(tx_site[x],rx_site,string);
			}

			if(elevation_plot)
			{
				printf("--->if(elevation_plot)\n");

				if (txsites>1)
				{
					snprintf(string,250,"%s-%c.%s%c",elevation_file,'1'+x,ext,0);
				}
				else
				{
					snprintf(string,250,"%s.%s%c",elevation_file,ext,0);
				}
				GraphElevation(tx_site[x],rx_site,string);
			}

			if (height_plot)
			{
				if (txsites>1)
				{
					snprintf(string,250,"%s-%c.%s%c",graph_height_file,'1'+x,ext,0);
				}
				else
				{
					snprintf(string,250,"%s.%s%c",graph_height_file,ext,0);
				}
				// generate the Elevation graph (-h or -H option)
				GenerateElevationGraph(tx_site[x],rx_site,string,fresnel_plot,norm);
			}
		}
	}

	if (area_mode && topomap==0)
	{
		for (x=0; x<txsites && x<max_txsites; x++)
		{
			if (coverage)
			{
				PlotLOSMap(tx_site[x],altitude);
			}
			else if (ReadLRParm(tx_site[x],1))
			{
				PlotLRMap(tx_site[x],altitudeLR,ano_filename);
			}
			SiteReport(tx_site[x]);
		}
	}

	if (map || topomap)
	{
		// Label the map
		if (kml==0)
		{
			for (x=0; x<txsites && x<max_txsites; x++)
			{
				PlaceMarker(tx_site[x]);
			}
		}

		if (cities)
		{
			for (y=0; y<cities; y++)
			{
				LoadCities(city_file[y]);
			}
			fprintf(stdout, "\n");
			fflush(stdout);
		}

		// Load city and county boundary data files
		if (bfs)
		{
			for (y=0; y<bfs; y++)
			{
				LoadBoundaries(boundary_file[y]);
			}

			fprintf(stdout, "\n");
			fflush(stdout);
		}

		// Plot the map

		if (coverage || pt2pt_mode || topomap)
		{
			WritePPM(mapfile,geo,kml,ngs,tx_site,txsites);
		}
		else
		{
			if (LR.EffRadiatedPower==0.0)
			{
				WritePPMLR(mapfile,geo,kml,ngs,tx_site,txsites);
			}
			else if (dbm)
			{
				WritePPMDBM(mapfile,geo,kml,ngs,tx_site,txsites);
			}
			else
			{
				WritePPMSS(mapfile,geo,kml,ngs,tx_site,txsites);
			}
		}
	}

	if (command_line_log && strlen(logfile)>0)
	{
		fd=fopenFullPath(logfile,"w");

		if (fd!=NULL)
		{
			for (x=0; x<argc; x++)
			{
				fprintf(fd,"%s ",argv[x]);
			}
			fprintf(fd,"\n");
			fclose(fd);

			fprintf(stdout, "\n");
			fprintf(stdout, "Command-line parameter log written to: \"%s\"\n",logfile);
		}
	}

	printf("\n");

	//--------------------------
	// close logUsr
	end_logUsr();

	//--------------------------
	// close sqlite map db
	sqlite3_close(dbidx);			// Close database
	//--------------------------

	// That's all, folks!
		
#ifdef DBG_ITWOM
	fclose(fMr);
#endif

	return 0;
}
