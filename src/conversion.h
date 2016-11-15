#ifndef CONVERSION_H
#define CONVERSION_H

//--------------------------------------------------------------------
// vedere: http://www.metric-conversions.org/
#define	METERS_PER_MILE 	1609.344
#define	KM_PER_MILE 		1.609344
#define METERS_PER_FOOT 	0.3048									// meters for each foot
#define	FOOT_PER_METERS		3.2808398950131233595800524934383		// foot for each meter (1/METERS_PER_FOOT)
#define	FOOT_PER_MILES		5280.0

//--------------------------------------------------------------------
// da:
// http://www.leapsecond.com/tools/rel.c
// Define unit conversion macros.

#define inch_to_meter(x) ( (x) * 0.0254 )
#define meter_to_inch(x) ( (x) / 0.0254 )

#define foot_to_meter(x) ( inch_to_meter((x) * 12.0) )
#define meter_to_foot(x) ( meter_to_inch((x) / 12.0) )

#define mile_to_meter(x) ( foot_to_meter((x) * 5280.0) )
#define meter_to_mile(x) ( meter_to_foot((x) / 5280.0) )

#define km_to_meter(x)  ( (x) * 1000.0 )
#define meter_to_km(x)  ( (x) / 1000.0 )

#define kt_to_kmph(v)    ( (v) * 1.852 )
#define kmph_to_kt(v)    ( (v) / 1.852 )

#define mach_to_ms(v)    ( (v) * 340.3 )
#define ms_to_mach(v)    ( (v) / 340.3 )

// 1 miglio = 5280 piedi
// miles to foot
#define mi2Foot(miles) 			( FOOT_PER_MILES * (miles) )
// foot to miles
#define fo2Miles(foot) 			( (foot) / FOOT_PER_MILES )
// foot to meters
#define fo2Meters(foot) 		( METERS_PER_FOOT * (foot) )
// meters to foot
#define mt2Foot(meters) 		( (meters) / METERS_PER_FOOT )
// miles to Km
#define mi2Km(miles) 			( KM_PER_MILE * (miles) )
// Km to miles
#define Km2Miles(km) 			( (km) / KM_PER_MILE )
// foot to Km
#define fo2Km(foot) 			( KM_PER_MILE * (foot) / FOOT_PER_MILES)

//--------------------------------------------------------------------
// angle conversions
#define DEGREES_FOR_RADIANS 	1.74532925199e-02

// convert degrees to radians
#define	RADIANS(degrees)		(degrees*DEGREES_FOR_RADIANS)
// convert radians to degrees
#define	DEGREES(radians)		(radians/DEGREES_FOR_RADIANS)

// convert degrees to first round
#define Degrees2FirstRound(degrees)		{	\
	while ((degrees)<0)                     \
	{                                       \
		(degrees)+=360;                     \
	}                                       \
	while ((degrees)>=360)                  \
	{                                       \
		(degrees)-=360;                     \
	}                                       \
}

#endif			// #ifndef CONVERSION_H
