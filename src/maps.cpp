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
// maps utility
// *****************************************************************************
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include "fileutil.h"
#include "conversion.h"
#include "dbmaps.h"
#include "languages.h"

#define DEFINE_VARIABLES
#include "maps.h"

// clear all data in array
extern void ClearArray(void *ptrArray, size_t n_bytes);

extern char fn_zippath_srtmmaps[PATH_MAX];		// absolute path of srtm zip maps

//---------------------------
// database with map index
extern sqlite3 *dbidx;

extern int bzerror;

// check if the machine is big endian or little endian
// check: is_big_endian() ? "big" : "little"
int is_big_endian(void)
{
    union {
        uint16_t i;
        char c[2];
    } e = { 0x0100 };
    return (e.c[0]);
}

// ---------------------------------------------
// initialize maps parameters
void IniMapsData(void)
{
	int x;

	earthRadiusFoot=EARTHRADIUS_FOOT;
	four_thirds_earthRadiusFoot=FOUR_THIRDS*EARTHRADIUS_FOOT;

	NumPointsDegree = IPPD;					// n. points per degree (integer)
	ppd = (double)NumPointsDegree;			// n. points per degree (double)
	DegreesPerPoint = 1.0/ppd;							// degrees per point
	idxLastPointDegree = NumPointsDegree-1;	// maximum point index per degree

	for (x=0; x<MAXPAGES; x++)
	{
		dem[x].min_el=32768;
		dem[x].max_el=-32768;
		dem[x].min_lat=90;
		dem[x].max_lat=FREE_MAP;
		dem[x].min_lon=360;
		dem[x].max_lon=-1;

		//----------------------------
		// add mr
		dem[x].sea_level = 0;			// 1 if region assumed as sea level
	}
}

// ---------------------------------------------
// count n. maps to load in memory
// ---------------------------------------------
//
// Number of unit squares that meet a given diagonal line segment in more than one point
//
// Solution:
// Count = DeltaX + DeltaY - GCD(DeltaX , DeltaY)
// where:
// GCD = Greatest Common Divisor of the two coordinate deltas
//

//----------------------------
// GCD algorithm
//  Returns the greatest common denominator of a and b using the
//  Euclid's algorithm division-based.
// Parameters:
//  a,b - two positive integers
// Returns:
//  the greatest common denominator
//
int GreatCommonDenominator(int a, int b)
{
    int temp;
    while(b!=0)
    {
        temp = b;
        b = a % b;
        a = temp;
    }
    return abs(a);
}

// count n. maps to load in memory using the formula:
// Count = DeltaX + DeltaY - GCD(DeltaX , DeltaY)
// DeltaX = (max_lon - min_lon)
// DeltaY = (max_lat - min_lat)
int CountMapsUsed(int maxlon, int minlon, int maxlat, int minlat)
{
	int DeltaX, DeltaY;
	int Count;

	DeltaX = (maxlon - minlon);
	DeltaY = (maxlat - minlat);
	Count = DeltaX + DeltaY - GreatCommonDenominator(DeltaX , DeltaY);
	return(Count);
}

//---------------------------------------------------
// MAP STRUCTURE UTILITY
//---------------------------------------------------
void setval_elevation(int indx_map, int coordX, int coordY, short value)
{
	map[indx_map].elevation[coordX][coordY] = value;
}

short getval_elevation(int indx_map, int coordX, int coordY)
{
	return(map[indx_map].elevation[coordX][coordY]);
}

void addval_elevation(int indx_map, int coordX, int coordY, short value)
{
	map[indx_map].elevation[coordX][coordY] += value;
}

// return true if the point is at sea level
int isPointSeaLevel(int indx_map, int coordX, int coordY)
{ 	
	return(getval_elevation(indx_map, coordX, coordY)==0);
}

//---------------------------------------------------
// SRTM MAP UTILITY
//---------------------------------------------------

// return the height converted from the data pointed from ptrBuffer
int heightSRTM(unsigned char *ptrBuffer)
{
	int value;
	value = ((int)(*ptrBuffer) & 0xFF);
	value = value << 8;
	ptrBuffer++;
	value = value | ((int)(*ptrBuffer) & 0xFF);
	if(value & 0x8000)
	{
		value = value - 0x10000;
	}
	return(value);
}

int MapHminmaxholes(char *mapName, int *min, int *max, long *n_holes)
{
	FILE *fileMap;
	unsigned char read_data[2];
	int i;
	int value;
	long addr;

	// init with dummy values
	*min=20000;
	*max=-1;
	*n_holes=0;

	fileMap=fopen(mapName,"rb");
	if(fileMap==NULL)
	{
		return(0);
	}
	i=0;
	addr= 0;
	while(!feof(fileMap))
	{
		if(fread(&read_data[0], 1, 2, fileMap) < 2)
		{
			break;
		}
		value = heightSRTM(&read_data[0]);
		if(value==-32768)
		{
			(*n_holes)++;
			printf("[%s][% 8li(%08lX)]=[%i]\n", mapName, addr, addr, value);
		}
		else
		{
			if(value > *max)
			*max = value;
			if(value < *min)
			*min = value;
		}
		addr = addr + 2;
		i++;
	}
	fclose(fileMap);

	if(*max < *min)
	{
		*max=0;
		*min=0;
	}

	return(1);
}

double ArcCos(double x, double y)
{
	// This function implements the arc cosine function,
	// returning a value between 0 and TWOPI.
	double result=0.0;

	if(y>0.0)
	{
		result=acos(x/y);
	}
	if(y<0.0)
	{
		result=PI+acos(x/y);
	}

	return result;
}

int ReduceAngle(double angle)
{
	// This function normalizes the argument to	an integer angle between 0 and 180 degrees

	double temp;

	temp=acos(cos(RADIANS(angle)));
	return (int)rint(DEGREES(temp));
}

double DeltaLongitude(double lon1, double lon2)
{
	// This function returns the short path longitudinal
	// difference between longitude1 and longitude2
	// as an angle between -180.0 and +180.0 degrees.
	// If lon1 is west of lon2, the result is positive.
	// If lon1 is east of lon2, the result is negative.

	double diff;

	diff=lon1-lon2;

	if (diff<=-180.0)
	{
		diff+=360.0;
	}
	if (diff>=180.0)
	{
		diff-=360.0;
	}
	return diff;
}

#define AZIMUTH(azimuth, src_lat, src_lon, dest_lat, dest_lon)	{					\
		beta = ARC_2_POINTS(src_lat, src_lon, dest_lat, dest_lon);          \
		num=sin(dest_lat)-(sin(src_lat)*cos(beta));                         \
		den=cos(src_lat)*sin(beta);                                         \
		fraction=num/den;                                                   \
		if (fraction>=1.0)                                                  \
		{                                                                   \
			fraction=1.0;                                                   \
		}                                                                   \
		if (fraction<=-1.0)                                                 \
		{                                                                   \
			fraction=-1.0;                                                  \
		}                                                                   \
		azimuth = acos(fraction);														\
	}

// Receive the coordinate in radians.
// Return the azimuth in radians
double AzimuthRad(
	double src_lat,
	double src_lon, 
	double dest_lat, 
	double dest_lon
	)
{
	double	beta, azimuth, num, den, fraction;
	
	// Calculate distance on sferical surface
	beta = ARC_2_POINTS(src_lat, src_lon, dest_lat, dest_lon);
	// Calculate Azimuth
	num=sin(dest_lat)-(sin(src_lat)*cos(beta));
	den=cos(src_lat)*sin(beta);
	fraction=num/den;
	// Trap potential problems in acos() due to rounding
	if (fraction>=1.0)
	{
		fraction=1.0;
	}
	if (fraction<=-1.0)
	{
		fraction=-1.0;
	}
	// Calculate azimuth
	azimuth=acos(fraction);
	return(azimuth);
}	
	
double AzimuthSites(site_data source, site_data destination)
{
	// This function returns the azimuth (in degrees) to the
	// destination as seen from the location of the source.

	double	dest_lat, dest_lon, src_lat, src_lon;
	double azimuth, diff;
	
	dest_lat=RADIANS(destination.lat);
	dest_lon=RADIANS(destination.lon);
	src_lat =RADIANS(source.lat);
	src_lon =RADIANS(source.lon);

	azimuth = AzimuthRad(src_lat, src_lon, dest_lat, dest_lon);
	
	// Reference it to True North
	diff=dest_lon-src_lon;
	if (diff<=-PI)
	{
		diff+=TWOPI;
	}
	if (diff>=PI)
	{
		diff-=TWOPI;
	}
	if (diff>0.0)
	{
		azimuth=TWOPI-azimuth;
	}

	return (DEGREES(azimuth));
}

//----------------------------------------------
// search point in map

// added 10/08: come ultimo SearchPointMap, ma restituisce gli indici
// x0, y0 in double. Viene usato per linearizzare
int SearchPointMapIdxDouble(double lat, double lon, int &indx, double &x0, double &y0)
{
	double deltaLat, deltaLon;
	for (indx=0; indx<MAXPAGES; indx++)
	{
		deltaLat = ( (lat) - (double)dem[indx].min_lat );
		if(deltaLat < 0.0)
			continue;
		if(deltaLat >= 1.0)
			continue;
		// deltaLon = ( DeltaLongitude( (double)dem[indx].max_lon, (lon) ));
		// deltaLon = ( (double)dem[indx].max_lon - (lon) );
		// 0.0 <= deltaLon < 1.0
		// 0.0 >= -deltaLon > -1.0
		// if we add 1.0 to all members:
		// 1.0 + 0.0 >= 1.0 -deltaLon > 1.0 -1.0
		// 0.0 < 1.0 -deltaLon <= 1.0
		// ori1: deltaLon = ( (double)dem[indx].max_lon - (lon) );
		deltaLon = ( (lon) - ( (double)(dem[indx].max_lon -1) ) );
		if(deltaLon < 0.0)
			continue;
		if(deltaLon >= 1.0)
			continue;

		// calculate the table indexes x0, y0
		// modifica mr 10/08: tolti i round e convertiti gli indici in int.
		x0=( ppd * deltaLat );

		y0=( ppd * deltaLon );
		return(1);
	}
	// not found
	return(0);
}

int SearchPointMap(double lat, double lon, int &indx, int &x0, int &y0)
{
	double deltaLat, deltaLon;
	for (indx=0; indx<MAXPAGES; indx++)
	{
		deltaLat = ( (lat) - (double)dem[indx].min_lat );
		if(deltaLat < 0.0)
			continue;
		if(deltaLat >= 1.0)
			continue;
		// deltaLon = ( DeltaLongitude( (double)dem[indx].max_lon, (lon) ));
		// deltaLon = ( (double)dem[indx].max_lon - (lon) );
		// 0.0 <= deltaLon < 1.0
		// 0.0 >= -deltaLon > -1.0
		// if we add 1.0 to all members:
		// 1.0 + 0.0 >= 1.0 -deltaLon > 1.0 -1.0
		// 0.0 < 1.0 -deltaLon <= 1.0
		// ori1: deltaLon = ( (double)dem[indx].max_lon - (lon) );
		// deltaLon = 1.0 - ( (double)dem[indx].max_lon - (lon) );
		deltaLon = ( (lon) - ( (double)(dem[indx].max_lon -1) ) );
		if(deltaLon < 0.0)
			continue;
		if(deltaLon >= 1.0)
			continue;

		// calculate the table indexes x0, y0
		// modifica mr 10/08: tolti i round e convertiti gli indici in int.
		x0=(int)( ppd * deltaLat );
		// y0=idxLastPointDegree-(int)round(ppd * deltaLon);
		// y0=(int)(ppd - 1.0) -(int)round(ppd * deltaLon);
		// y0=(int)(ppd) - 1 -(int)round(ppd * deltaLon);
		// ok y0=(int)round(ppd * deltaLon) - 1;
		// ok, originale va sottratto 1: y0=(int)( ppd * deltaLon )-1;
		// ulteriore modifica (che mi pare corretta)
		y0=(int)( ppd * deltaLon );
		return(1);
	}
	// not found
	return(0);
}

int new1SearchPointMap(double lat, double lon, int &indx, int &x0, int &y0)
{
	double deltaLat, deltaLon;
	for (indx=0; indx<MAXPAGES; indx++)
	{
		deltaLat = ( (lat) - (double)dem[indx].min_lat );
		if(deltaLat < 0.0)
			continue;
		if(deltaLat >= 1.0)
			continue;
		deltaLon = ( DeltaLongitude( (double)dem[indx].max_lon, (lon) ));
		if(deltaLon < 0.0)
			continue;
		if(deltaLon >= 1.0)
			continue;
		deltaLon = (1.0 - deltaLon);
		// ok x0=(int)round( ppd * deltaLat );

		// calculate the table indexes x0, y0
		// modifica mr 10/08: tolti i round e convertiti gli indici in int.
		x0=(int)( ppd * deltaLat );
		// y0=idxLastPointDegree-(int)round(ppd * deltaLon);
		// y0=(int)(ppd - 1.0) -(int)round(ppd * deltaLon);
		// y0=(int)(ppd) - 1 -(int)round(ppd * deltaLon);
		// ok y0=(int)round(ppd * deltaLon) - 1;
		// ok, originale va sottratto 1: y0=(int)( ppd * deltaLon )-1;
		// ulteriore modifica (che mi pare corretta)
		y0=(int)( ppd * deltaLon );
		return(1);
	}
	// not found
	return(0);
}

int oriSearchPointMap(double lat, double lon, int &indx, int &x0, int &y0)
{
	for (indx=0; indx<MAXPAGES; indx++)
	{
		x0=(int)rint( ppd * ( (lat) - (double)dem[indx].min_lat ) );
		y0=idxLastPointDegree-(int)rint(ppd * ( DeltaLongitude( (double)dem[indx].max_lon, (lon) )));
		if (x0>=0 && x0<=idxLastPointDegree && y0>=0 && y0<=idxLastPointDegree)
		{
			return(1);
		}
	}
	// not found
	return(0);
}

//----------------------------------------------

int PutMask(double lat, double lon, int value)
{
	// Lines, text, markings, and coverage areas are stored in a
	// mask that is combined with topology data when topographic
	// maps are generated by rfprobe.  This function sets and resets
	// bits in the mask based on the latitude and longitude of the
	// area pointed to.

	int	x0, y0, indx;
	char	found;

	// SEARCH_POINTMAP( lat, lon );
	found = SearchPointMap( lat, lon , indx, x0, y0);
	if (found)
	{
		dem[indx].mask[x0][y0]=value;
		return ((int)dem[indx].mask[x0][y0]);
	}
	else
	{
		return -1;
	}
}

int OrMask(double lat, double lon, int value)
{
	// Lines, text, markings, and coverage areas are stored in a
	// mask that is combined with topology data when topographic
	// maps are generated by rfprobe.  This function sets bits in
	// the mask based on the latitude and longitude of the area
	// pointed to.

	int	x0, y0, indx;
	char	found;

	// SEARCH_POINTMAP( lat, lon );
	found = SearchPointMap( lat, lon , indx, x0, y0);
	if (found)
	{
		dem[indx].mask[x0][y0]|=value;
		return ((int)dem[indx].mask[x0][y0]);
	}
	else
	{
		return -1;
	}
}

int GetMask(double lat, double lon)
{
	// This function returns the mask bits based on the latitude and longitude given.

	return (OrMask(lat,lon,0));
}

int PutSignal(double lat, double lon, unsigned char signal)
{
	// This function writes a signal level (0-255)
	// at the specified location for later recall.

	int	x0, y0, indx;
	char	found;

	// SEARCH_POINTMAP( lat, lon );
	found = SearchPointMap( lat, lon , indx, x0, y0);
	if (found)
	{
		dem[indx].signal[x0][y0]=signal;
		return (dem[indx].signal[x0][y0]);
	}
	else
	{
		return 0;
	}
}

unsigned char GetSignal(double lat, double lon)
{
	// This function reads the signal level (0-255) at the
	// specified location that was previously written by the
	// complimentary PutSignal() function.

	int	x0, y0, indx;
	char	found;

	// SEARCH_POINTMAP( lat, lon );
	found = SearchPointMap( lat, lon , indx, x0, y0);
	if (found)
	{
		return (dem[indx].signal[x0][y0]);
	}
	else
	{
		return 0;
	}
}

//----------------------------------------------

char str_dec2dms[32];			// used to dec2dms conversion
char *dec2dms(double decimal)
{
	// Converts decimal degrees to degrees, minutes, seconds,
	// (DMS) and returns the result as a character string.

	char sign;
	int	degrees;
	int minutes;
	int seconds;
	double a;
	double b;
	double c;
	double d;

	str_dec2dms[0]=0;

	if (decimal<0.0)
	{
		decimal=-decimal;
		sign=-1;
	}
	else
	{
		sign=1;
	}
	a=floor(decimal);
	b=60.0*(decimal-a);
	c=floor(b);
	d=60.0*(b-c);

	degrees=(int)a;
	minutes=(int)c;
	seconds=(int)d;

	if (seconds<0)
	seconds=0;

	if (seconds>59)
	seconds=59;

	snprintf(str_dec2dms,32,"%+d%c%d\'%d\"", degrees*sign, 32, minutes, seconds);
	return(&str_dec2dms[0]);
}

double newGetElevation(site_data location)
{
	// This function returns the elevation (in feet) of any location
	// represented by the digital elevation model data in memory.
	// Function returns NO_LAND_HEIGHT for locations not found in memory.

	char	found;
	unsigned char useLeftUpPoints;
	double	dblX0, dblY0;		// idx with remaining
	double	deltaX0, deltaY0;	// remaining
	int	x0, y0, indx;
	double	elevation;
	double  elev0, elev1, m, risEl0, risEl1;

	// found = SearchPointMap( location.lat, location.lon , indx, x0, y0);
	found = SearchPointMapIdxDouble( location.lat, location.lon , indx, dblX0, dblY0);
	if (found==0)
	{
		return(NO_LAND_HEIGHT);
	}

	// found elevation
	x0 = (int)dblX0;
	y0 = (int)dblY0;
	deltaX0 = dblX0 - (double)x0;
	deltaY0 = dblY0 - (double)y0;

	//
	// use height linearization. Choice points from these:
	//  h(x0-1,y0-1) h(x0-1,y0) h(x0-1,y0+1)
	//  h(  x0,y0-1)   h(x0,y0)   h(x0,y0+1)
	//  h(x0+1,y0-1) h(x0+1,y0) h(x0+1,y0+1)
	useLeftUpPoints = 0x00;
	if(x0>=idxLastPointDegree)
	{
		// for other point use left point from h(x0,y0)
		useLeftUpPoints = 0x01;
	}
	if(y0>=idxLastPointDegree)
	{
		// for other point use up point from h(x0,y0)
		useLeftUpPoints = useLeftUpPoints | 0x02;
	}
	// get the elevation h(x0,y0)
	elev0=(double)getval_elevation(indx, x0, y0);
	// 1) linearize elevation in the same row with y=y0
	if(useLeftUpPoints & 0x01)
	{
		// for linearization use left point
		elev1=(double)getval_elevation(indx, x0-1, y0);
		m = (elev0-elev1);				// m = (h1-h0)/1.0
		risEl0 = elev1 + m * (deltaX0 + 1.0);
	}
	else
	{
		// for linearization use right point
		elev1=(double)getval_elevation(indx, x0+1, y0);
		m = (elev1-elev0);				// m = (h1-h0)/1.0
		risEl0 = elev0 + m * deltaX0;
	}

	// 2) linearize elevation in the same row with y=y0+1 or y=y0-1
	if(useLeftUpPoints & 0x02)
	{
		// for linearization use up point
		// get the elevation h(x0,y0-1)
		elev0=(double)getval_elevation(indx, x0, y0-1);
		// 1) linearize elevation in the same row with y=y0-1
		if(useLeftUpPoints & 0x01)
		{
			// for linearization use left point
			elev1=(double)getval_elevation(indx, x0-1, y0-1);
			m = (elev0-elev1);				// m = (h1-h0)/1.0
			risEl1 = elev1 + m * (deltaX0+1.0);
		}
		else
		{
			// for linearization use right point
			elev1=(double)getval_elevation(indx, x0+1, y0-1);
			m = (elev1-elev0);				// m = (h1-h0)/1.0
			risEl1 =  elev0 + m * deltaX0;
		}
	}
	else
	{
		// for linearization use down point
		// get the elevation h(x0,y0-1)
		elev0=(double)getval_elevation(indx, x0, y0+1);
		// 1) linearize elevation in the same row with y=y0+1
		if(useLeftUpPoints & 0x01)
		{
			// for linearization use left point
			elev1=(double)getval_elevation(indx, x0-1, y0+1);
			m = (elev0-elev1);				// m = (h1-h0)/1.0
			risEl1 = elev1 + m * (deltaX0+1.0);
		}
		else
		{
			// for linearization use right point
			elev1=(double)getval_elevation(indx, x0+1, y0+1);
			m = (elev1-elev0);				// m = (h1-h0)/1.0
			risEl1 =  elev0 + m * deltaX0;
		}
	}

	// 3) linearize elevation in the same column with x=x0+deltaX0
	// get the elevation h(x0+deltaX0,y0-1)
	elev0=risEl0;
	// 1) linearize elevation in the same row with y=y0-1
	if(useLeftUpPoints & 0x02)
	{
		// for linearization use up point
		elev1=risEl1;
		m = (elev0-elev1);				// m = (h1-h0)/1.0
		elevation = elev1 + m * (deltaY0+1.0);
	}
	else
	{
		// for linearization use down point
		elev1=risEl1;
		m = (elev1-elev0);				// m = (h1-h0)/1.0
		elevation = elev0 + m * deltaY0;
	}

	elevation=mt2Foot( elevation );

	return elevation;
}

double GetElevation(site_data location)
{
	// This function returns the elevation (in feet) of any location
	// represented by the digital elevation model data in memory.
	// Function returns NO_LAND_HEIGHT for locations not found in memory.

	char	found;
	int	x0, y0, indx;
	double	elevation;

	found = SearchPointMap( location.lat, location.lon , indx, x0, y0);
	if (found)
	{
		// double tmp=getval_elevation(indx, x0, y0);
		// if(maxElevation<tmp)
		// {
		// 	maxElevation=tmp;
		// }
		// logUsr("GetElevation[%lf][%i][%i][%i](m)=[%lf]\n",maxElevation,indx,x0,y0,tmp);
		/**
		// FORCE ALTITUDE.
		// ENABLE ONLY FOR DEBUG
		if((dbgRxsite.lat==location.lat)&&(dbgRxsite.lon==location.lon))
		{
			// force altitude
			elevation=FOOT_PER_METERS*1058.4;
		}
		else if((dbgTxsite.lat==location.lat)&&(dbgTxsite.lon==location.lon))
		{
			// force altitude
			elevation=FOOT_PER_METERS*25.6;
		}
		else
		**/
		{
			// original
			elevation=FOOT_PER_METERS*getval_elevation(indx, x0, y0);
		}
		//	elevation=FOOT_PER_METERS*1058.4;
	}
	else
	{
		elevation=NO_LAND_HEIGHT;
	}

	return elevation;
}

int AddHeightToElevation(double lat, double lon, double height)
{
	// This function adds a user-defined terrain feature
	// (in meters AGL) to the digital elevation model data
	// in memory.  Does nothing and returns 0 for locations
	// not found in memory.

	char	found;
	int	x0, y0, indx;

	// SEARCH_POINTMAP( lat, lon );
	found = SearchPointMap( lat, lon , indx, x0, y0);
	if (found)
	{
		addval_elevation(indx, x0, y0, (short)rint(height));
	}

	return found;
}

double Distance(site_data site1, site_data site2)
{
	// This function returns the great circle distance
	// in miles between any two site locations.

	double	lat1, lon1, lat2, lon2, distance;

	lat1=RADIANS(site1.lat);
	lon1=RADIANS(site1.lon);
	lat2=RADIANS(site2.lat);
	lon2=RADIANS(site2.lon);

	// http://www.movable-type.co.uk/scripts/latlong.html
	// Law of cosines:	d = acos( sin alfa1 * sin alfa2 + cos alfa1 * cos alfa2 * cos(alfa1-alfa2) ) * R
	distance= DISTANCE_2_POINTS(lat1, lon1, lat2, lon2);

	return distance;
}

// calc distance in Km
double DistanceKm(site_data site1, site_data site2)
{
	// This function returns the great circle distance
	// in miles between any two site locations.

	double	lat1, lon1, lat2, lon2, distance;

	lat1=RADIANS(site1.lat);
	lon1=RADIANS(site1.lon);
	lat2=RADIANS(site2.lat);
	lon2=RADIANS(site2.lon);

	// Spherical Law of Cosines:
	distance=DISTANCE_2_POINTS( lat1, lon1, lat2, lon2 );
	distance=mi2Km(distance);
	return distance;
}

//-------------------------------------------------------

// return -1 if no there is a map in memory
// or indx if the map is stored in memory
int FindMapInMemory(int minlat, int maxlat, int minlon, int maxlon)
{
	int indx;

	for (indx=0; indx<MAXPAGES; indx++)
	{
		if (minlat==dem[indx].min_lat &&
			minlon==dem[indx].min_lon &&
			maxlat==dem[indx].max_lat &&
			maxlon==dem[indx].max_lon)
		{
			return(indx);
		}
	}
	return(-1);			// map not found in memory
}

int FindFirstFreeMapinMemory(void)
{
	int indx;

	for (indx=0; indx<MAXPAGES; indx++)
	{
		if (dem[indx].max_lat==FREE_MAP)
		{
			return(indx);
		}
	}
	return(-1);			// map not found in memory
}

// int LoadMap_SDF_BZ(char *name)
// Return indx value: -1 if the map is not loaded
//
int LoadMap_SDF_BZ(int minlat, int maxlat, int minlon, int maxlon)
{
	// This function reads .bz2 compressed rfprobe Data Files containing
	// digital elevation model data into memory.  Elevation data,
	// maximum and minimum elevations, and quadrangle limits are
	// stored in the first available dem[] structure.

	int	x, y, data, indx;
	char fFoundFreePage=0;
	char sdf_file[255];
	char path_plus_name[512];
	char *string;
	FILE	*fd;
	BZFILE	*bzfd;


	// the map it already in memory ?
	indx = FindMapInMemory(minlat, maxlat, minlon, maxlon);
	if(indx>=0)
	{
		// the map is already in memory
		return(indx);
	}
	
	// the map is not in memory
	// Is room available to load it?
	indx = FindFirstFreeMapinMemory();
	if(indx<0)
	{
		// no room to load map
		fprintf(stdout, "--- No room to load map!!!\n");
		fflush(stdout);
		return(-1);
	}

	snprintf(sdf_file,16,"%d:%d:%d:%d", minlat, maxlat, minlon, maxlon);

	// Parse sdf_file name for minimum latitude and longitude values
	x=strlen(sdf_file);
	sdf_file[x]='.';
	sdf_file[x+1]='s';
	sdf_file[x+2]='d';
	sdf_file[x+3]='f';
	sdf_file[x+4]='.';
	sdf_file[x+5]='b';
	sdf_file[x+6]='z';
	sdf_file[x+7]='2';
	sdf_file[x+8]=0;
	
	fFoundFreePage=1;

	// Search for SDF file in current working directory first
	strncpy(path_plus_name,sdf_file,255);
	fd=fopenFullPath(path_plus_name,"rb");
	bzfd=BZ2_bzReadOpen(&bzerror,fd,0,0,NULL,0);
	if (fd==NULL || bzerror!=BZ_OK)
	{
		// Next, try loading SDF file from path specified
		// in $HOME/.splat_path file or by -d argument

		strncpy(path_plus_name,sdf_path,255);
		strncat(path_plus_name,sdf_file,254);

		fd=fopenFullPath(path_plus_name,"rb");
		bzfd=BZ2_bzReadOpen(&bzerror,fd,0,0,NULL,0);
	}

	if (fd!=NULL && bzerror==BZ_OK)
	{
		pmsg( stdout, (loading_sdf_into_page), path_plus_name,indx+1);
		fflush(stdout);

		sscanf(BZfgets(bzfd,255),"%d",&dem[indx].max_lon);
		sscanf(BZfgets(bzfd,255),"%d",&dem[indx].min_lat);
		sscanf(BZfgets(bzfd,255),"%d",&dem[indx].min_lon);
		sscanf(BZfgets(bzfd,255),"%d",&dem[indx].max_lat);

		for (x=0; x<NumPointsDegree; x++)
		{
			for (y=0; y<NumPointsDegree; y++)
			{
				string=BZfgets(bzfd,20);
				data=atoi(string);

				setval_elevation(indx, x, y, data);

				dem[indx].signal[x][y]=0;
				dem[indx].mask[x][y]=0;

				if (data>dem[indx].max_el)
				dem[indx].max_el=data;

				if (data<dem[indx].min_el)
				dem[indx].min_el=data;
			}
		}

		fclose(fd);

		BZ2_bzReadClose(&bzerror,bzfd);

		setElevationLimits(indx);
		SetLatitudeLimits(indx);
		SetLongitudeLimits(indx);

		pmsg( stdout, (sdf_done) );
		fflush(stdout);

		return (indx);
	}
	else
		return -1;

	return(indx);
}		// int LoadMap_SDF_BZ(char *name)

// from coordinate, get the original SRTM map name
void Coord2SRTMname(int minlat,	int maxlat, int minlon, int maxlon,	char *nSrtm)
{
	int dato;

	// insert map latitude
	dato = minlat;
	if(minlat>=0)
	{
		// latitudine nord
		nSrtm[0]='N';
	}
	else
	{
		dato = -dato;
		nSrtm[0]='S';
	}
	nSrtm[1]='0' + ((dato/10) & 0x0F);
	nSrtm[2]='0' + ((dato%10) & 0x0F);

	// insert map longitude
	dato = maxlon;
	if(maxlon>=180)
	{
		dato = 360-dato;
		nSrtm[3]='E';
	}
	else
	{
		nSrtm[3]='W';
	}
	nSrtm[4]='0' + ((dato/100) & 0x0F);
	dato = dato % 100;
	nSrtm[5]='0' + ((dato/10) & 0x0F);
	nSrtm[6]='0' + ((dato%10) & 0x0F);

	nSrtm[7]=0;
}

// from splat map name, get the original SRTM map name
void splat2SRTMname(char *name_splat, char *nSrtm)
{
	int minlat;
	int minlon;
	int maxlat;
	int maxlon;

	int dato;

	sscanf(name_splat,"%d:%d:%d:%d",&minlat,&maxlat,&minlon,&maxlon);

	// insert map latitude
	dato = minlat;
	if(minlat>=0)
	{
		// latitudine nord
		nSrtm[0]='N';
	}
	else
	{
		dato = -dato;
		nSrtm[0]='S';
	}
	nSrtm[1]='0' + ((dato/10) & 0x0F);
	nSrtm[2]='0' + ((dato%10) & 0x0F);

	// insert map longitude
	dato = maxlon;
	if(maxlon>=180)
	{
		dato = 360-dato;
		nSrtm[3]='E';
	}
	else
	{
		nSrtm[3]='W';
	}
	nSrtm[4]='0' + ((dato/100) & 0x0F);
	dato = dato % 100;
	nSrtm[5]='0' + ((dato/10) & 0x0F);
	nSrtm[6]='0' + ((dato%10) & 0x0F);

	nSrtm[7]=0;
}

//------------------------------------------------------------

// unzip map
int UnzipMapHgt(char *zipArchive, char *PathMapHgt, char *MapHgt, char *ext)
{
	char zipname[PATH_MAX];
	char mapname[PATH_MAX];


	strcpy(zipname, zipArchive);

	// recreate full name to extract command
	if( (PathMapHgt[0]==0)||( (PathMapHgt[0]=='.')&&(PathMapHgt[1]==0) ) )
	{
		sprintf(mapname, "%s.%s", MapHgt, ext);
	}
	else
	{
		sprintf(mapname, "%s/%s.%s", PathMapHgt, MapHgt, ext);
	}
	// original
	// extract7z_noPath(zipArchive, "." , mapname);
	// modify 15/06
	extract7z_noPath(zipArchive, sdf_path , mapname);
	return(1);
}


// correct the endian for a short array
// fEndian=1: big endian fEndian=0: little endian
void correctEndianShort(short *ptrShort, long count, int fEndian)
{
	unsigned short val1, val2;
	long i;
	int machine_endian;

	machine_endian = is_big_endian();
	if(machine_endian == fEndian)
	{
		// ok, same endian for machine and buffer
		return;
	}
	// the endianess of buffer and the machine are different.
	// Invert posizion of each bytes of short
	// printf("inversione endian!!!!\n");
	for(i=0;i<count;i++)
	{
		// swap each byte
		val1 = (unsigned short)(*ptrShort);
		val2 = val1;
		val1 = val1 >> 8;
		val2 = val2 << 8;
		val1 = val1 | val2;
		*ptrShort = (short)val1;
		ptrShort++;
	}
}

void PrintDbc_map(dbc_map *ptr_dbcmap)
{
	printf("??????????????????????????\n");
	printf("id			: [%i]\n",  ptr_dbcmap->id			);					// INTEGER PRIMARY KEY
	printf("zip_id		: [%i]\n",  ptr_dbcmap->zip_id		);				// INTEGER
	printf("filetime	: [%s]\n",  ptr_dbcmap->filetime		);			// DATETIME CREATION ID
	printf("path		: [%s]\n",  ptr_dbcmap->path			);	// TEXT
	printf("name		: [%s]\n",  ptr_dbcmap->name			);	// TEXT
	printf("ext			: [%s]\n",  ptr_dbcmap->ext			);				// TEXT
	printf("size		: [%i]\n",  ptr_dbcmap->size			);					// INTEGER
	printf("sha1		: [%s]\n",  ptr_dbcmap->sha1			);				// TEXT
	printf("type		: [%i]\n",  ptr_dbcmap->type			);					// INTEGER
	printf("rows_cols	: [%i]\n",  ptr_dbcmap->rows_cols		);				// INTEGER
	printf("lat_code	: [%s]\n",  ptr_dbcmap->lat_code		);			// TEXT
	printf("lat_value	: [%i]\n",  ptr_dbcmap->lat_value		);				// INTEGER
	printf("lon_code	: [%s]\n",  ptr_dbcmap->lon_code		);			// TEXT
	printf("lon_value	: [%i]\n",  ptr_dbcmap->lon_value		);				// INTEGER
	printf("n_holes		: [%li]\n", ptr_dbcmap->n_holes		);				// INTEGER
	printf("min_h		: [%i]\n",  ptr_dbcmap->min_h			);					// INTEGER
	printf("max_h		: [%i]\n",  ptr_dbcmap->max_h			);					// INTEGER
	printf("min_lat		: [%lf]\n", ptr_dbcmap->min_lat		);				// REAL
	printf("min_lon		: [%lf]\n", ptr_dbcmap->min_lon		);				// REAL
	printf("??????????????????????????\n");

}

// load a map SRTMI in memory
int LoadMap_SRTMInDem(dbc_map *ptr_dbcmap, int indx)
{
	FILE *fd;
	long address;
	long rowBytes;
	long n_bytes;
	short buffer[3601];				// buffer to load max dim of srtm row map
	size_t result;
	// int idxLat;
	int x0, y0;
	int NElements;

	char fn_mapfullpath[PATH_MAX];	// full path map file name

	unsigned short val1, val2;
	int machine_endian;
	char fSwapBytes;

	// @@@ debug
	// PrintDbc_map(ptr_dbcmap);
	if(ptr_dbcmap==NULL)
	{
		printf("LoadMap_SRTMInDem if(ptr_dbcmap==NULL)\n");
		return(0);
	}

	// check if the machine has a big endian format
	machine_endian = is_big_endian();
	fSwapBytes = 0;
	if(machine_endian != 1)
	{
		// different endian for machine and buffer
		fSwapBytes = 1;
	}
	sprintf(fn_mapfullpath, "%s/%s.%s",
		sdf_path, ptr_dbcmap->name, ptr_dbcmap->ext);

	fd=fopenFullPath(fn_mapfullpath,"rb");

	if(fd == NULL)
	{
		return(0);
	}

	// n. bytes of a map row
	// For SRTM3: 1201 * 2 = 2402
	// For SRTM1: 3601 * 2 = 7202
	rowBytes = ( (long)ptr_dbcmap->rows_cols * 2l );
	// calculate the n. bytes to read for each row
	n_bytes = rowBytes - 2;
	NElements = n_bytes / 2;	// n. elements to read

	// calculate the start address
	address = (long)(ptr_dbcmap->rows_cols -1) * rowBytes + 2;

	x0 = 0;
	for(; address >= 2; address = address - rowBytes)
	{
		fseek( fd , address , SEEK_SET );
		// copy the file into the buffer:
		result = fread((void *)&buffer[0], 1, n_bytes, fd);

		if(result != n_bytes)
		{
			// Reading error
			fclose(fd);
			return(-1);
		}
		// copy the array
		for(y0=0;y0<NElements;y0++)
		{
			val1 = buffer[NElements-1-y0];
			if(fSwapBytes)
			{
				val2 = val1;
				val1 = val1 >> 8;
				val2 = val2 << 8;
				val1 = val1 | val2;
			}
			setval_elevation(indx, x0, y0, val1);
		}
		x0++;
	}
	// terminate
	fclose(fd);
	return(1);
}

// This function reads SRTM map containing digital elevation model data into memory.
// Elevation data, maximum and minimum elevations, and quadrangle limits are stored
// in the first available dem[] structure.
// Return indx value: -1 if the map is not loaded
//
int LoadMap_SRTM_hgt(sqlite3 *dbMapIndex, int minlat, int maxlat, int minlon, int maxlon)
{
	int indx;
	char hgt_file[255];
	char zip_file[255];
	char path_plus_name[512];

	dbc_map dbcMap;
	dbc_zip dbcZip;

	// FILE	*fd;
	int  result;

	indx = -1;
	// the map is already in memory?
	indx = FindMapInMemory(minlat, maxlat, minlon, maxlon);
	if(indx>=0)
	{
		// the map is already in memory
		return(indx);
	}

	// Is room available to load it?
	indx = FindFirstFreeMapinMemory();
	if(indx<0)
	{
		// no room to load map
		fprintf(stdout, "--- No room to load map!!!\n");
		fflush(stdout);
		return(-1);
	}
	
	// The map is not in memory and there is space to load it
	// From coordinate, get the original SRTM map name
	Coord2SRTMname(minlat, maxlat, minlon, maxlon, hgt_file);

	printf("hgt_file[%s]indx[%i]\n", hgt_file, indx);

	// check if map exist 
	if(DbMapInfoReadFromMapName(dbMapIndex, hgt_file, &dbcMap)==0)
	{
		// map not found
		fprintf(stdout, "--- map not found!!!\n");
		fflush(stdout);
		return(-1);
	}
	
	// found map in db
	result = DbZipInfoRead(dbMapIndex, dbcMap.zip_id, &dbcZip);
	
	if(result == 0)
	{
		// map not found
		fprintf(stdout, "--- map not found!!!\n");
		fflush(stdout);
		return(-1);
	}

	// -------------------------
	// uncompress the map using the info in db index
	sprintf(zip_file, "%s/%s.zip", fn_zippath_srtmmaps, dbcZip.name);

	UnzipMapHgt(zip_file, dbcMap.path,
		dbcMap.name, dbcMap.ext);

	dem[indx].min_lat = minlat;
	dem[indx].min_lon = minlon;
	dem[indx].max_lat = maxlat;
	dem[indx].max_lon = maxlon;

	// clear all data in matrix signal and mask
	ClearArray((void *)&dem[indx].signal[0][0],
			sizeof(dem[indx].signal) / sizeof(char));

	ClearArray((void *)&dem[indx].mask[0][0],
			sizeof(dem[indx].mask) / sizeof(char));

	result = LoadMap_SRTMInDem(&dbcMap, indx);
	if(result<1)
	{
		return(-1);				// error
	}
	// save elevation data for debug
	sprintf(path_plus_name,"./%s_buf.bin", hgt_file);
	// SaveBinFile(path_plus_name, (void *)&(dem[indx].elevation[0][0]), (sizeof(dem[indx].elevation)/sizeof(unsigned char)));
	// -------------------------
	
	// set max and min height
	dem[indx].min_el = dbcMap.min_h;
	dem[indx].max_el = dbcMap.max_h;

	return(indx);
}	// int LoadMap_SRTM_hgt()

//------------------------------------------------------------
// NOTE:
// each map cover 1 degree of latitude and 1 degree of longitude.
// http://www.csgnetwork.com/degreelenllavcalc.html
// 1 degree correspond:
// Length Of A Degree Of Latitude In Meters:  110574.61m (110.5Km)
// Length Of A Degree Of Longitude In Meters: 111302.62m (111.3Km)
// The function return:
// indx: map index loaded
//
int LoadSrtmMap(int minlat, int maxlat, int minlon, int maxlon)
{
	// This function loads the requested SRTM map from the filesystem.
	// It first tries to invoke the LoadMap_SRTM_hgt() function to load an
	// uncompressed hgt file (since uncompressed files load slightly
	// faster).  If that attempt fails, then it tries to load a
	// compressed SDF file by invoking the LoadMap_SDF_BZ() function.
	// If that fails, then we can assume that no elevation data
	// exists for the region requested, and that the region
	// requested must be entirely over water.

	int	x;
	int y;
	int indx;
	char string[255];
	char fFoundFreePage=0;
	int	return_value;

	// Try to load an uncompressed SDF first.

	// modify 03/06/2016
	snprintf(string,16,"%d:%d:%d:%d", minlat, maxlat, minlon, maxlon);
	pmsg( stdout, (loadmapsrtm_hgt_name), string);
	fflush(stdout);

	// queries the database index to load map hgt format 
	indx = LoadMap_SRTM_hgt(dbidx, minlat, maxlat, minlon, maxlon);
	pmsg( stdout, (loadmapsrtm_hgt_return), indx);
	fflush(stdout);

	// if indx < 0 the load is failed.
	// try loading a compressed SDF.
	if (indx<0)
	{
		indx=LoadMap_SDF_BZ(minlat, maxlat, minlon, maxlon);
	}
	
	// If neither format can be found, then assume the area is water.
	if (indx<0)
	{
		//--------------------------------------
		// area is water
		//--------------------------------------
		//
		// Is it already in memory?
		indx = FindMapInMemory(minlat, maxlat, minlon, maxlon);
		if(indx<0)
		{
			// the map is not in memory
			// Is room available to load it?
			indx = FindFirstFreeMapinMemory();
			if(indx>=0)
				fFoundFreePage=1;
		}
		if (fFoundFreePage && indx>=0 && indx<MAXPAGES)
		{
			// region assumed as sea level
			pmsg( stdout, (region_assumed_as_sea_level),string, indx+1);
			fflush(stdout);

			dem[indx].sea_level = 1;		// region at sea level
			
			dem[indx].max_lon=maxlon;
			dem[indx].min_lat=minlat;
			dem[indx].min_lon=minlon;
			dem[indx].max_lat=maxlat;

			// Fill DEM with sea-level topography

			for (x=0; x<NumPointsDegree; x++)
			{
				for (y=0; y<NumPointsDegree; y++)
				{
					setval_elevation(indx, x, y, 0);
					dem[indx].signal[x][y]=0;
					dem[indx].mask[x][y]=0;
					if (dem[indx].min_el>0)
					{
						dem[indx].min_el=0;
					}
				}
			}

			// return_value=1;
		}
		//--------------------------------------
		// end area is water
		//--------------------------------------
	}
	
	if (indx>=0)
	{
		// the map is loaded in memory
			// fprintf(stdout, "--- end if(FindMapInMemory Indx:[%i]\n", indx);
			// fflush(stdout);

		setElevationLimits(indx);
		SetLatitudeLimits(indx);
		SetLongitudeLimits(indx);

		pmsg( stdout, (sdf_done) );
		fflush(stdout);
	}
	
	return (indx);
}		// char LoadSrtmMap(char *name)


//------------------------------------------------------------
// POINT MAP COLOR UTILITIES
//

// initialize the grey level points
void IniGreyLevelPoints(void)
{
	terrainGreyLevel = 0;
	one_over_gamma=1.0/GAMMA;
	KmoltGreyLevel=255.0/pow(MinMaxElevationRange(),one_over_gamma);
}

// set the grey level color of the map point
unsigned SetTerrainGreyLevel(int indx, int x0, int y0)
{
	terrainGreyLevel=(unsigned)(0.5+pow(DeltaMinElevation(indx, x0, y0), one_over_gamma)*KmoltGreyLevel);
	return(terrainGreyLevel);
}

// end of maps.cpp