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
#ifndef LANGUAGES_H
#define LANGUAGES_H

#include <stdarg.h>

// http://www.barrgroup.com/Embedded-Systems/How-To/Firmware-Internationalization

// ID languages
typedef enum 
{ 
	ENGLISH, 
	SPANISH, 
	FRENCH, 
	GERMAN,
	ITALIAN,
	LAST_LANGUAGE 
}	LANGUAGE_ID;

// string id
typedef enum
{
// help messages
available_options,
t_txsitesqth_max_4,
r_rxsiteqth,
c_plot_los_coverage_of_txs,
l_plot_path_loss_map_of_tx,
s_filenames_of_city,
b_filenames_of_cartographic_boundary,
p_terrain_profile_graph,
e_terrain_elevation_graph,
h_terrain_height_graph,
h_normalized_terrain_height_graph,
l_path_loss_graph,
o_topographic_map_to_generate_ppm,
u_userdefined_terrain_file,
d_sdf_file_directory_path,
m_earth_radius_multiplier,
n_do_not_plot_los_paths_in_ppm_maps,
n_do_not_produce_unnecessary_site,	
f_frequency_for_fresnel_zone,
r_modify_default_range_for_c_or_l,
sc_display_smooth_contour_levels,
db_threshold_will_not_be_displayed,
nf_do_not_plot_fresnel_zones,
fz_fresnel_zone_clearance_percentage,
gc_ground_clutter_height_feet_meters,
ngs_display_greyscale_topography,
erp_override_erp_in_lrp_file_watts,
ano_name_of_file,
ani_name_of_input_file,
udt_name_of_user_defined_terrain,
kml_generate_google_earth_kml,
geo_generate_an_xastir_geo,
dbm_plot_signal_power_level,
log_copy_command_line_string,
gpsav_preserve_gnuplot_files,
metric_employ_metric_units,
olditm_longleyrice,
if_that_flew_by_too_fast,
less,
hd_less,
type_man_or_see_documentation,
compilation_analysis_over_square,
degree,
degrees,
of_terrain_default_signal_levels,

// haat()
minus_haat_terrain,
plus_haat_terrain_sum,
haat_sum_c,
haat_avg_terrain_alt_elevation_haat,

// LoadMapSDF_SDF(), LoadMapSDF_BZ()
sdf_done,
loading_sdf_into_page,

// LoadCities()
error_file_not_found,
reading_file,

// LoadMapSDF()
loadmapsrtm_hgt_name,
loadmapsrtm_hgt_return,
region_assumed_as_sea_level,

// PlotLOSMap()
Computing_lineofsight_coverage,
and_ground_clutter,
perc_00_to_25,
perc_25_to_50,
perc_50_to_75,
perc_75_to_100,

// WritePPM()
writing_pixmap_image,

LAST_LANG_MSG
} STR_MSG;


const char *strLang(STR_MSG str_id);

// print a language message
void prtLang(FILE *pf, const char *format, ...);

// print a language message with id
#define	pmsg( stream, id, ...) 	prtLang( stream, strLang( id ), ##__VA_ARGS__)

#endif
