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
#ifndef MAPS_H_INCLUDED
#define MAPS_H_INCLUDED

#include "external.h"   // Support macros EXTERN, INITIALIZE

#include "rfprobe.h"

#if HD_MODE==0
	#if MAXPAGES==4
	#define ARRAYSIZE 4950
	#endif

	#if MAXPAGES==9
	#define ARRAYSIZE 10870
	#endif

	#if MAXPAGES==16
	#define ARRAYSIZE 19240
	#endif

	#if MAXPAGES==25
	#define ARRAYSIZE 30025
	#endif

	#if MAXPAGES==36
	#define ARRAYSIZE 43217
	#endif

	#if MAXPAGES==49
	#define ARRAYSIZE 58813
	#endif

	#if MAXPAGES==64
	#define ARRAYSIZE 76810
	#endif

	#define IPPD 1200
#endif

#if HD_MODE==1
	#if MAXPAGES==1
	#define ARRAYSIZE 5092
	#endif

	#if MAXPAGES==4
	#define ARRAYSIZE 14844
	#endif

	#if MAXPAGES==9
	#define ARRAYSIZE 32600
	#endif

	#if MAXPAGES==16
	#define ARRAYSIZE 57713
	#endif

	#if MAXPAGES==25
	#define ARRAYSIZE 90072
	#endif

	#if MAXPAGES==36
	#define ARRAYSIZE 129650
	#endif

	#if MAXPAGES==49
	#define ARRAYSIZE 176437
	#endif

	#if MAXPAGES==64
	#define ARRAYSIZE 230430
	#endif

	#define IPPD 3600
#endif

#ifndef PI
#define PI 3.141592653589793
#endif

#ifndef TWOPI
#define TWOPI 6.283185307179586
#endif

#ifndef HALFPI
#define HALFPI 1.570796326794896
#endif

#define FOUR_THIRDS 1.3333333333333

#define EARTH_RADIUS_MILES	3959.0		// raggio terrestre in miglia
#define EARTHRADIUS_FOOT 20902230.97	// raggio terrestre in piedi (6370999.999656m)

// -------------------
// elevation constant for no land in feet
#define	NO_LAND_HEIGHT		-5000.0			// in feet

// -------------------
// if max_lat element is set to -90, the map element is free
#define FREE_MAP			-90

// -----------------------------------

// macro to search in maps
// versione originale:
#define SEARCH_POINTMAP( lat, lon) {													\
		for (indx=0, found=0; indx<MAXPAGES && found==0;)                               \
		{                                                                               \
			x0=(int)rint( ppd*( (lat) - (double)dem[indx].min_lat ) );              	\
			y0=mpi-(int)rint(ppd * ( DeltaLongitude( (double)dem[indx].max_lon, (lon) )));  	\
			if (x0>=0 && x0<=mpi && y0>=0 && y0<=mpi)                                   \
			found=1;                                                                	\
			else                                                                        \
			indx++;                                                  					\
		}                                                                               \
	}


// set the elevation limits from the map index
#define setElevationLimits(indx) 			\
{                                           \
	if (dem[indx].min_el<min_elevation)     \
	{                                       \
		min_elevation=dem[indx].min_el;     \
	}                                       \
	if (dem[indx].max_el>max_elevation)     \
	{                                       \
		max_elevation=dem[indx].max_el;     \
	}                                       \
}

#define MinMaxElevationRange()				((double)(max_elevation-min_elevation))
#define DeltaMinElevation(indx, x0, y0)		((double)(getval_elevation(indx, x0, y0) - min_elevation))

// n. points latitude and longitude
#define NPointsLatitude() 				(unsigned)(NumPointsDegree*ReduceAngle(max_lat-min_lat))
#define NPointsLongitude()				(unsigned)(NumPointsDegree*ReduceAngle(max_lon-min_lon))
// get elevation.

// Copy elevations plus clutter along path into the elev[] array.
#define COPY_ELEVATIONS_PLUS_CLUTTER()										\
{                                                                           \
	for (x=1; x<path.length-1; x++)                                         \
	{                                                                       \
		elev[x+2]=fo2Meters(path.elevation[x]==0.0?                         \
							path.elevation[x]:                              \
							(clutter+path.elevation[x]));                   \
	}                                                                       \
	elev[2]=fo2Meters(path.elevation[0]);									\
	elev[path.length+1]=fo2Meters(path.elevation[path.length-1]);      		\
}

// used to set latitude or longitude
#define SET_LATITUDE_TODOWN(idxLat)		( maxLat - ( DegreesPerPoint * (double)(idxLat) ) )
#define SET_LATITUDE_TOUP(idxLat)		( (double)min_lat + ( DegreesPerPoint * (double)(idxLat) ) )
#define SET_LONGITUDE_TODOWN(idxLon)	( (double)max_lon - (DegreesPerPoint*(double)(idxLon) ) )
#define SET_LONGITUDE_TOUP(idxLon)		( minLon + ( DegreesPerPoint * (double)(idxLon) ) )

// angle conversion

#define SetLatitudeLimits(indx)	{			\
	if (max_lat==-90)                       \
	{                                       \
		max_lat=dem[indx].max_lat;          \
	}                                       \
	else if (dem[indx].max_lat>max_lat)     \
	{                                       \
		max_lat=dem[indx].max_lat;          \
	}                                       \
	if (min_lat==90)                        \
	{                                       \
		min_lat=dem[indx].min_lat;          \
	}                                       \
	else if (dem[indx].min_lat<min_lat)     \
	{                                       \
		min_lat=dem[indx].min_lat;          \
	}                                       \
}
#define SetLongitudeLimits(indx)	{						\
	if (max_lon==-1)                                        \
	{                                                       \
		max_lon=dem[indx].max_lon;                          \
	}                                                       \
	else                                                    \
	{                                                       \
		if (abs(dem[indx].max_lon-max_lon)<180)             \
		{                                                   \
			if (dem[indx].max_lon>max_lon)                  \
			{                                               \
				max_lon=dem[indx].max_lon;                  \
			}                                               \
		}                                                   \
		else                                                \
		{                                                   \
			if (dem[indx].max_lon<max_lon)                  \
			{                                               \
				max_lon=dem[indx].max_lon;                  \
			}                                               \
		}                                                   \
	}                                                       \
	if (min_lon==360)                                       \
	{                                                       \
		min_lon=dem[indx].min_lon;                          \
	}                                                       \
	else                                                    \
	{                                                       \
		if (abs(dem[indx].min_lon-min_lon)<180)             \
		{                                                   \
			if (dem[indx].min_lon<min_lon)                  \
			{                                               \
				min_lon=dem[indx].min_lon;                  \
			}                                               \
		}                                                   \
		else                                                \
		{                                                   \
			if (dem[indx].min_lon>min_lon)                  \
			{                                               \
				min_lon=dem[indx].min_lon;                  \
			}                                               \
		}                                                   \
	}                                                       \
}

// normalyze longitude. Negative longitude is converted to positive
#define	NormalLongitude(longitude)		{	\
	if ( (longitude)<0.0 )				    \
		(longitude)+=360.0;                 \
}

// format longitude
#define FMT_LONGITUDE(longitude) 	((longitude) <180.0?-(longitude):360.0-(longitude))

// update (min_lat, max_lat, min_lon, max_lon)
#define UpdateMinMaxCoord(latitude, longitude)			\
{                                                       \
	if ((latitude)<min_lat)                             \
		min_lat=(latitude);                             \
	if ((latitude)>max_lat)                             \
		max_lat=(latitude);                             \
	if (DeltaLongitude((longitude),min_lon)<0.0)        \
		min_lon=(longitude);                            \
	if (DeltaLongitude((longitude),max_lon)>=0.0)       \
		max_lon=(longitude);                            \
}


// -------------------
// http://www.movable-type.co.uk/scripts/latlong.html
// http://mathworld.wolfram.com/SphericalTrigonometry.html
// http://www.robertobigoni.it/Matematica/Sferica/sferica.html
// Spherical Law of Cosines:
// http://www.robertobigoni.it/Matematica/Sferica/sferica.html
// 5. distance between two Distanza tra due punti.
// d = acos( sin alfa1 * sin alfa2 + cos alfa1 * cos alfa2 * cos(alfa1-alfa2) ) * R
// angle center earth = acos( sin alfa1 * sin alfa2 + cos alfa1 * cos alfa2 * cos(alfa1-alfa2) )
//
#define ARC_2_POINTS(lat1, lon1, lat2, lon2) 			\
	( acos(sin(lat1)*sin(lat2) + cos(lat1)*cos(lat2)*cos((lon1)-(lon2))) )

#define DISTANCE_2_POINTS(lat1, lon1, lat2, lon2) 			\
	( EARTH_RADIUS_MILES * ARC_2_POINTS(lat1, lon1, lat2, lon2) )

#define DISTANCE_2_ARC(distance)		( distance / EARTH_RADIUS_MILES )

typedef struct {
	double lat;
	double lon;
	float alt;
	char name[50];
	char filename[255];
} site_data;

typedef struct
{
	int min_lat;
	int max_lat;
	int min_lon;
	int max_lon;

	int max_el;
	int min_el;

	// short elevation[IPPD][IPPD];		// portato in map
	unsigned char mask[IPPD][IPPD];
	unsigned char signal[IPPD][IPPD];

	//----------------------------
	// add mr
	unsigned char sea_level;			// 1 if region assumed as sea level
	//----------------------------
} dem_data;

typedef struct
{
	int indx;							// index of map actually enabled
	short elevation[IPPD][IPPD];
} map_data;

#if !defined(DEFINE_VARIABLES) || !defined(MAPS_H_DEFINITIONS)
// ======================================================
// Global variable declarations / definitions

EXTERN double earthRadiusFoot;						// earth radius moltiplied EarthRadiusMultiplier
EXTERN double EarthRadiusMultiplier;
EXTERN double four_thirds_earthRadiusFoot;			// four_thirds_earthRadiusFoot=FOUR_THIRDS*EARTHRADIUS_FOOT;

EXTERN dem_data dem[MAXPAGES];
EXTERN map_data map[MAXPAGES];						// store the map info
EXTERN char sdf_path[PATH_MAX];

// points for maps SRTM
EXTERN int NumPointsDegree;
EXTERN int idxLastPointDegree;
EXTERN double DegreesPerPoint;		// n. degrees for each point of latitude/longitude
EXTERN double ppd;

EXTERN int min_lat INITIALIZE(90);
EXTERN int max_lat INITIALIZE(-90);
EXTERN int min_lon INITIALIZE(360);
EXTERN int max_lon INITIALIZE(-1);

// ----------------------------------------
// elevation limits
EXTERN int max_elevation INITIALIZE(-32768);
EXTERN int min_elevation INITIALIZE(32768);

// used to initialize site_data
EXTERN site_data ZeroSiteData INITIALIZE({			\
	0.0,                                            \
	0.0,                                            \
	0.0,                                            \
	{0},                                            \
	{0}                                             \
});

//------------------------------------------------------------
// USED IN POINT MAP COLOR UTILITIES

#define GAMMA 2.5

EXTERN double KmoltGreyLevel;
EXTERN double one_over_gamma;
EXTERN unsigned terrainGreyLevel;

// ======================================================
#endif // !DEFINE_VARIABLES || !MAPS_H_DEFINITIONS

// initialize maps parameters
void IniMapsData(void);

int CountMapsUsed(int maxlon, int minlon, int maxlat, int minlat);
// MAP STRUCTURE UTILITY
void setval_elevation(int indx_map, int coordX, int coordY, short value);
short getval_elevation(int indx_map, int coordX, int coordY);
void addval_elevation(int indx_map, int coordX, int coordY, short value);
// return true if the point is at sea level
int isPointSeaLevel(int indx_map, int coordX, int coordY);

// return the height converted from the data pointed from ptrBuffer
int heightSRTM(unsigned char *ptrBuffer);
int MapHminmaxholes(char *mapName, int *min, int *max, long *n_holes);
double ArcCos(double x, double y);
int ReduceAngle(double angle);
double DeltaLongitude(double lon1, double lon2);
double AzimuthSites(site_data source, site_data destination);

// added 10/08: come ultimo SearchPointMap, ma restituisce gli indici
// x0, y0 in double. Viene usato per linearizzare
int SearchPointMapIdxDouble(double lat, double lon, int &indx, double &x0, double &y0);
// mr: modifica reso piu' preciso
int SearchPointMap(double lat, double lon, int &indx, int &x0, int &y0);

int PutMask(double lat, double lon, int value);
int OrMask(double lat, double lon, int value);
int GetMask(double lat, double lon);

int PutSignal(double lat, double lon, unsigned char signal);
unsigned char GetSignal(double lat, double lon);

char *dec2dms(double decimal);
// original elevation.
double GetElevation(site_data location);
int AddHeightToElevation(double lat, double lon, double height);
double Distance(site_data site1, site_data site2);
// mr: calc distance in Km
double DistanceKm(site_data site1, site_data site2);
// return -1 if no there is a map in memory
// or indx if the map is stored in memory
int FindMapInMemory(int minlat, int maxlat, int minlon, int maxlon);
int FindFirstFreeMapinMemory(void);
// int LoadMap_SDF_BZ(char *name)
int LoadMap_SDF_BZ(int minlat, int maxlat, int minlon, int maxlon);
// from coordinate, get the original SRTM map name
void Coord2SRTMname(int minlat,	int maxlat, int minlon, int maxlon,	char *nSrtm);
// from splat map name, get the original SRTM map name
void splat2SRTMname(char *name_splat, char *nSrtm);
// unzip map
int UnzipMapHgt(char *zipArchive, char *PathMapHgt, char *MapHgt, char *ext);
// correct the endian for a short array
// fEndian=1: big endian fEndian=0: little endian
void correctEndianShort(short *ptrShort, long count, int fEndian);
// load a map SRTMI in memory
int LoadMap_SRTMInDem(dbc_map *ptr_dbcmap, int indx);
// This function reads SRTM map containing digital elevation model data into memory.
// Elevation data, maximum and minimum elevations, and quadrangle limits are stored
// in the first available dem[] structure.
//
int LoadMap_SRTM_hgt(sqlite3 *dbMapIndex, int minlat, int maxlat, int minlon, int maxlon);
// each map cover 1 degree of latitude and 1 degree of longitude.
int LoadSrtmMap(int minlat, int maxlat, int minlon, int maxlon);

//------------------------------------------------------------
// POINT MAP COLOR UTILITIES
// initialize the grey level points
void IniGreyLevelPoints(void);
// set the grey level color of the map point
unsigned SetTerrainGreyLevel(int indx, int x0, int y0);

// Standard epilogue
#ifdef DEFINE_VARIABLES
#define MAPS_H_DEFINITIONS
#endif // DEFINE_VARIABLES

#endif // MAPS_H_INCLUDED