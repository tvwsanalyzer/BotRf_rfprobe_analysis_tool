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
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "languages.h"

// language manager

#define	NMSG(LangArray)		( sizeof(LangArray) / sizeof(LangArray[(STR_MSG)0]) )

// array of languages
const char *lang_english[LAST_LANG_MSG] = 
{
// help messages
"\n\t\t --==[ %s v%s Available Options... ]==--\n\n",											// available_options,
"       -t txsite(s).qth (max of 4 with -c, max of 30 with -L)\n",                              // t_txsitesqth_max_4,
"       -r rxsite.qth\n",                                                                       // r_rxsiteqth,
"       -c plot LOS coverage of TX(s) with an RX antenna at X feet/meters AGL\n",               // c_plot_los_coverage_of_txs,
"       -L plot path loss map of TX based on an RX at X feet/meters AGL\n",                     // l_plot_path_loss_map_of_tx,
"       -s filename(s) of city/site file(s) to import (5 max)\n",                               // s_filenames_of_city,
"       -b filename(s) of cartographic boundary file(s) to import (5 max)\n",                   // b_filenames_of_cartographic_boundary,
"       -p filename of terrain profile graph to plot\n",                                        // p_terrain_profile_graph,
"       -e filename of terrain elevation graph to plot\n",                                      // e_terrain_elevation_graph,
"       -h filename of terrain height graph to plot\n",                                         // h_terrain_height_graph,
"       -H filename of normalized terrain height graph to plot\n",                              // h_normalized_terrain_height_graph,
"       -l filename of path loss graph to plot\n",                                              // l_path_loss_graph,
"       -o filename of topographic map to generate (.ppm)\n",                                   // o_topographic_map_to_generate_ppm,
"       -u filename of user-defined terrain file to import\n",                                  // u_userdefined_terrain_file,
"       -d sdf file directory path (overrides path in ~/.splat_path file)\n",                   // d_sdf_file_directory_path,
"       -m earth radius multiplier\n",                                                          // m_earth_radius_multiplier,
"       -n do not plot LOS paths in .ppm maps\n",                                               // n_do_not_plot_los_paths_in_ppm_maps,
"       -N do not produce unnecessary site or obstruction reports\n",	                        // n_do_not_produce_unnecessary_site,	
"       -f frequency for Fresnel zone calculation (MHz)\n",                                     // f_frequency_for_fresnel_zone,
"       -R modify default range for -c or -L (miles/km)\n",                                     // r_modify_default_range_for_c_or_l,
"      -sc display smooth rather than quantized contour levels\n",                              // sc_display_smooth_contour_levels,
"      -db threshold beyond which contours will not be displayed\n",                            // db_threshold_will_not_be_displayed,
"      -nf do not plot Fresnel zones in height plots\n",                                        // nf_do_not_plot_fresnel_zones,
"      -fz Fresnel zone clearance percentage (default = 60)\n",                                 // fz_fresnel_zone_clearance_percentage,
"      -gc ground clutter height (feet/meters)\n",                                              // gc_ground_clutter_height_feet_meters,
"     -ngs display greyscale topography as white in .ppm files\n", 	                            // ngs_display_greyscale_topography,
"     -erp override ERP in .lrp file (Watts)\n",                                                // erp_override_erp_in_lrp_file_watts,
"     -ano name of alphanumeric output file\n",                                                 // ano_name_of_file,
"     -ani name of alphanumeric input file\n",                                                  // ani_name_of_input_file,
"     -udt name of user defined terrain input file\n",                                          // udt_name_of_user_defined_terrain,
"     -kml generate Google Earth (.kml) compatible output\n",                                   // kml_generate_google_earth_kml,
"     -geo generate an Xastir .geo georeference file (with .ppm output)\n",                     // geo_generate_an_xastir_geo,
"     -dbm plot signal power level contours rather than field strength\n",                      // dbm_plot_signal_power_level,
"     -log copy command line string to this output file\n",                                     // log_copy_command_line_string,
"   -gpsav preserve gnuplot temporary working files after execution\n",                         // gpsav_preserve_gnuplot_files,
"  -metric employ metric rather than imperial units for all user I/O\n",                        // metric_employ_metric_units,
"  -olditm invoke Longley-Rice rather than the default ITWOM model\n\n",                        // olditm_longleyrice,
"If that flew by too fast, consider piping the output through 'less':\n",                       // if_that_flew_by_too_fast,
"\n\%s | less\n\n",                                                                             // less,
"\n\%s-hd | less\n\n",                                                                          // hd_less,
"Type 'man %s', or see the documentation for more details.\n\n",                                // type_man_or_see_documentation,
"This compilation of %s supports analysis over a region of %d square\n",                        // compilation_analysis_over_square,
"degree",                                                                                       // degree,
"degrees",                                                                                      // degrees,
" of terrain. Default signal levels model: ITWOM Version %.1f.\n\n",                            // of_terrain_default_signal_levels,
// haat()
"--- haat terrain[%f] c[%i]\n",																	// minus_haat_terrain,
"+++ haat terrain[%f] c[%i] sum[%f]\n",                                                         // plus_haat_terrain_sum,
"haat sum[%f] c[%i]\n",                                                                         // haat_sum_c,
"haat avg_terrain[%f] alt[%f] elevation[%f] haat[%f]\n",                                        // haat_avg_terrain_alt_elevation_haat,
// LoadMapSDF_SDF(), LoadMapSDF_BZ()
"Done!\n",																						// sdf_done
"Loading \"%s\" into page %d... ",                                            					// loading_sdf_into page
// LoadCities()
"\n*** ERROR: \"%s\": not found!",																// error_file_not_found,
"\nReading \"%s\"... ",                                                                         // reading_file,
// LoadMapSDF()
"--- LoadMapSRTM_hgt name[%s]\n",																// loadmapsrtm_hgt_name,	
"--- LoadMapSRTM_hgt return:[%i]\n",															// loadmapsrtm_hgt_return,
"Region  \"%s\" assumed as sea-level into page %d...",											// region_assumed_as_sea_level,	
// PlotLOSMap()
"computing line-of-sight coverage of \"%s\" with an rx antenna\nat %.2f %s agl",				// Computing_lineofsight_coverage,
" and %.2f %s of ground clutter",																// and_ground_clutter,	
" 0%c to  25%c ",																				// perc_00_to_25,
"25%c to  50%c ",																				// perc_25_to_50,
"50%c to  75%c ",																				// perc_50_to_75,
"75%c to 100%c ",																				// perc_75_to_100,
// WritePPM()
"Writing \"%s\" (%ux%u pixmap image)... "														// writing_pixmap_image,


};

const char *lang_spanish[LAST_LANG_MSG];
const char *lang_french[LAST_LANG_MSG];
const char *lang_german[LAST_LANG_MSG];
const char *lang_italian[LAST_LANG_MSG];

const int nMsg[LAST_LANGUAGE] =
{
	NMSG( lang_english ), 
	NMSG( lang_spanish ), 
	NMSG( lang_french  ), 
	NMSG( lang_german  ),
	NMSG( lang_italian )
};

LANGUAGE_ID lang_code = ENGLISH;
STR_MSG max_lang_msg = (STR_MSG)(NMSG( lang_english ));

LANGUAGE_ID SetLang(LANGUAGE_ID id)
{
	if(id>=LAST_LANGUAGE)
	{
		id = ENGLISH;
	}
	lang_code = id;
	max_lang_msg = (STR_MSG)(nMsg[ id ]);
	return(id);
}

const char *strLang(STR_MSG str_id)
{
	const char *msg;
	LANGUAGE_ID lang_id = lang_code;
	STR_MSG msg_id = str_id;
	if(msg_id>max_lang_msg)
	{
		lang_id = ENGLISH;
	}
	switch(lang_id)
	{
		case ENGLISH: 	msg=lang_english[msg_id];		return(msg);
		case SPANISH:   msg=lang_spanish[msg_id];		return(msg);
		case FRENCH:    msg=lang_french[msg_id];		return(msg); 
		case GERMAN:    msg=lang_german[msg_id];		return(msg); 
		case ITALIAN:   msg=lang_italian[msg_id];		return(msg);
	}
	// for other cases
	return(lang_english[msg_id]);
}

#define MAX_CHAR_BUFF	2048
char buffStr[MAX_CHAR_BUFF];

// http://www.cplusplus.com/reference/cstdio/vsnprintf/
// print a language message
void prtLang(FILE *pf, const char *format, ...)
{
	va_list args;
	// const char *format;
	
	// format = str(str_id);
	va_start (args, format );
	vsnprintf(buffStr, MAX_CHAR_BUFF, format, args);
	va_end (args);
	fprintf(pf, "%s", buffStr);
}



// end languages.c


