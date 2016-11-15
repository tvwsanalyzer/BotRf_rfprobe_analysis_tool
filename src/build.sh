#!/bin/bash
#
# Simple shell script for building the tool RFPROBE! and associated utilities.
# This is a modified version of script written by John A. Magliacane, KD2BD May 2002 -- Last update: February 2014
#

cpu=$(uname -m)
model=""

if [ "$cpu" = "x86_64" ]; then
        cpu="x86-64"
	model="-mcmodel=medium"

	if [[ $(uname -a) =~ Darwin ]]; then
		model="";   ## for Mac OSX  ## J.V.
	fi

elif [ "$cpu" = "sun4u" ]; then
	cpu=""
	model=""
fi

build_rfprobe()
{
	if [ -r std-parms.h ]; then
		cp std-parms.h rfprobe.h
	else
		echo "/* Parameters for 3 arc-second standard resolution mode of operation */" > std-parms.h

		echo "#define MAXPAGES 9" >> std-parms.h
		echo "#define HD_MODE 0" >> std-parms.h
		cp std-parms.h rfprobe.h
	fi

	echo -n "Compiling rfprobe!... "
	# ---------------------------------------------------
	# compile rfprobe
	# ---------------------------------------------------
	# The new rfprobe version uses sqlite3 library
	# With gcc use this compile options
	# gcc -o $1 $1.c -lm -lsqlite3 -std=gnu99
	#
	# -------------- original compile version
	# g++ -O2 -s -fomit-frame-pointer -ffast-math -pipe -march=$cpu $model itwom3.0.cpp rfprobe.cpp -lm -lbz2 -o rfprobe
	# &> errbuild.txt      stderr and stdout to file
	# -------------- actual compile version
	g++ -Wall -O2 -s -fomit-frame-pointer -ffast-math -pipe -march=$cpu $model itwom3.0.cpp rfprobe.cpp languages.cpp fileutil.cpp sqlutil.cpp dbmaps.cpp maps.cpp -lm -lsqlite3 -lbz2 -o rfprobe &> errbuild.txt

	if [ -x rfprobe ]; then
		echo "Done!"
	else
		echo "Compilation failed!"
	fi

	if [ -r hd-parms.h ]; then
		cp hd-parms.h rfprobe.h
		echo -n "Compiling RFPROBE! HD... "
		g++ -O2 -s -fomit-frame-pointer -ffast-math -pipe -march=$cpu $model itwom3.0.cpp rfprobe.cpp languages.cpp fileutil.cpp sqlutil.cpp dbmaps.cpp maps.cpp -lm -lbz2 -o rfprobe-hd

		if [ -x rfprobe-hd ]; then
			echo "Done!"
		else
			echo "Compilation failed!"
		fi
	fi
}

build_utils()
{
	cd utils
	./build all
	cd ..
}

if [ "$#" = "0" ]; then
	echo "Usage: build  { rfprobe, utils, all }"
else

	if [ "$1" = "rfprobe" ]; then
		build_rfprobe
	fi

	if [ "$1" = "utils" ]; then
		build_utils
	fi

	if [ "$1" = "all" ]; then
		build_rfprobe
		build_utils
	fi

	if [ "$1" != "rfprobe" ] && [ "$1" != "utils" ] && [ "$1" != "all" ]; then
		echo "Usage: build { rfprobe, utils, all }"
	fi
fi

