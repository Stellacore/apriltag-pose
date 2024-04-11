/* Copyright (C) 2024, Stellacore Corporation.  All rights reserved.

	This file is an extension to the baseline AprilTags Library code
	developed by The Regents of The University of Michigan 2013-2016.
	The contents of this are provided under the same license as the
	original work and subject to same terms as the original work as
	follows:


This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/


// application specific
#include "demo_tagfamily.h"
#include "demo_options.h"
#include "demo_configure_tag_detector.h"
#include "demo_image.h"

// library
#include "apriltag.h"
#include "common/zarray.h"

// system
#include <stdio.h>


//! Perform tag detection and image pose estimation on a single image
bool
process_one_image
	( char const * const imgpath
	, apriltag_detector_t * const tagfinder
	, getopt_t * const getopt  // mostly for reporting
	)
{
	bool okay = false;

	int quiet = getopt_get_bool(getopt, "quiet");
	if (! quiet)
	{
		printf("Info: processing image '%s'\n", imgpath);
	}

	// load input image into memory
	image_u8_t * const tagimg = create_image_from_path(imgpath);
	if (tagimg)
	{
		zarray_t * detections = apriltag_detector_detect(tagfinder, tagimg);

		// process ...
		// TODO

		apriltag_detections_destroy(detections);
	}
	else
	{
		printf
			("\nError: Unable to process image! (process_one_image:)"
				"\n Explanation...." // TODO
				"\n Image from: '%s'"
				"\n"
			, imgpath
			);
	}
	image_u8_destroy(tagimg);


	return okay;
}

//! Perform tag detection and image orientation all all specified input images
bool
process_all_images
	( getopt_t * const getopt
	, apriltag_detector_t * const tagfinder
	)
{
	bool allgood = true; // unless proven otherwise

	// process each image in turn
	zarray_t const * const inputs = getopt_get_extra_args(getopt);
	for (int imgNdx = 0; imgNdx < zarray_size(inputs); ++imgNdx)
	{
		// retrieve input path for this iteration
		char const * imgpath = NULL;
		zarray_get(inputs, imgNdx, &imgpath);

		// perform core processing operations
		bool const okay = process_one_image(imgpath, tagfinder, getopt);

		// exit early if encountring a failure
		allgood &= okay;
		if (! allgood)
		{
			break;
		}
	}

	// return status of processing all images
	return allgood;
}

//! Perform tag detection and image orientation all all specified input images
bool
run_inside_tag_environment
	( getopt_t * const getopt
	)
{
	bool okay = false;

	// allocate tag family space
	char const * famname = getopt_get_string(getopt, "family");
	// get configuration information for requested tag family
	apriltag_family_t * tagfam = create_tagfamily(famname);

	if (tagfam)
	{
		// create and populate tag detector instance
		apriltag_detector_t * tagfinder = apriltag_detector_create();
		if (configure_tag_detector(tagfinder, getopt, tagfam))
		{
			// perform core processing (tag detection and pose estimation)
			okay = process_all_images(getopt, tagfinder);
		}

		// cleanup tag detector
		apriltag_detector_destroy(tagfinder);
	}

	// free tag family space
	destroy_tagfamily(tagfam, famname);

	return okay;
}

//! Smallest of the the two arguments (first if same)
inline
size_t
min_of
	( int const v1
	, int const v2
	)
{
	int min = v1;
	if (v2 < v1)
	{
		min = v2;
	}
	return (size_t)min;
}


int
main
	( int argc
	, char *argv[]
	)
{
	int stat = 1;
printf("Hello from : %s, argc = %d\n", argv[0], argc);

	// allocate options space
	getopt_t * getopt = apriltag_options();

	// parse command line options and check invocation
	if ( (! getopt_parse(getopt, argc, argv, 1))
	   || getopt_get_bool(getopt, "help")
	   )
	{
		printf("\nUsage: %s [options] <input files>\n", argv[0]);
		getopt_do_usage(getopt);
	}
	else
	{
		// iterate pointlessly if requested
		// (e.g. to facilitate timing, memory leak checking, etc)
		size_t const maxiters = min_of(1, getopt_get_int(getopt, "iters"));
		for (size_t nn = 0 ; nn < maxiters ; ++nn)
		{
			if (! run_inside_tag_environment(getopt))
			{
				printf("\nFATAL: unable to process all images! (main:)\n");
				break;
			}
		}
	}

	// free options space
	getopt_destroy(getopt);

	return stat;
}
