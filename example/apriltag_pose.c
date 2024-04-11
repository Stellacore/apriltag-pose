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
#include "demo_tag_env.h"

// library
#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/zarray.h"

// system
#include <stdio.h>


//
// Utilities
//

//! Largest of the two arguments (first if same)
inline
size_t
max_of
	( int const v1
	, int const v2
	)
{
	int max = v1;
	if (v1 < v2)
	{
		max = v2;
	}
	return (size_t)max;
}


//! Print info message to stdout (if getopt quiet is not enabled)
void
info
	( getopt_t * const getopt
	, char * const fmt
	, ...
	)
{
	int quiet = getopt_get_bool(getopt, "quiet");
	if (! quiet)
	{
		fprintf(stdout, "Info: ");

		va_list argptr;
		va_start(argptr, fmt);
		vfprintf(stdout, fmt, argptr);
		va_end(argptr);
	}
}

//
// Processing functions
//

//! Perform tag detection and image pose estimation on a single image
bool
process_one_image
	( char const * const imgpath
	, apriltag_detector_t * const tagfinder
	, getopt_t * const getopt  // mostly for reporting
	)
{
	bool okay = false;

	info(getopt, "Processing image '%s'\n", imgpath);

	// load input image into memory
	image_u8_t * const tagimg = create_image_from_path(imgpath);
	if (tagimg)
	{
		// allocate detection data structures
		zarray_t * detections = apriltag_detector_detect(tagfinder, tagimg);

		// display information about each
		size_t const numFound = zarray_size(detections);
		info(getopt, "Number detections '%zu'\n", numFound);
		for (size_t findNdx = 0 ; findNdx < numFound ; ++findNdx)
		{
			// display detection info
			apriltag_detection_t * detection;
			zarray_get(detections, findNdx, &detection);
			info(getopt
				, "detection"
					" %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n"
				, findNdx
				, detection->family->nbits
				, detection->family->h
				, detection->id
				, detection->hamming
				, detection->decision_margin
				);

			// consfigure space resection required geometry info
			// (detected measurements, tag size, camera geometry)
			apriltag_detection_info_t taginfo =
				{ .det = detection
				, .tagsize = .114
				// guess for image (841w,625h)
				, .fx = 500.
				, .fy = 500.
				, .cx = 420.5
				, .cy = 312.5
				};

			// perform pose estimation
			apriltag_pose_t poseTagWrtCam;
			double const err = estimate_tag_pose(&taginfo, &poseTagWrtCam);

			// report results
			info(getopt, "pose estimation error = %12.9lf\n", err);
		//	matd_print(poseTagWrtCam.R, "%9.6lf");
		//	matd_print(poseTagWrtCam.t, "%9.6lf");

			// ?? Doesn't seem to be a way to detect success/fail in pose est.
			okay = true;
		}

		// free detection structures
		apriltag_detections_destroy(detections);
	}
	else
	{
		printf
			("\nError: Unable to process image! (process_one_image:)"
			//	"\n Explanation...." // TODO
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

//
// Main demo application
//

int
main
	( int argc
	, char *argv[]
	)
{
	int stat = 1;
printf("Hello from : %s, argc = %d\n", argv[0], argc);

	// construct tag finding environment (memory allocation)
	tag_env_t * tagenv = tag_env_new("tag36h11");
	if (tag_env_is_valid(tagenv)) // this should always be true here
	{
		// parse command line options and check invocation
		getopt_t * getopt = tagenv->the_getopt;
		if ( (! getopt_parse(getopt, argc, argv, 1))
		   || getopt_get_bool(getopt, "help")
		   )
		{
			printf("\nUsage: %s [options] <input files>\n", argv[0]);
			getopt_do_usage(getopt);
		}
		else
		{
			// if requested, iterate over all work
			// (e.g. to facilitate timing, memory leak checking, etc)
			size_t const maxiters = max_of(1, getopt_get_int(getopt, "iters"));
			for (size_t nn = 0 ; nn < maxiters ; ++nn)
			{
				// perform core processing (tag detection and pose estimation)
				apriltag_detector_t * tagfinder = tagenv->the_tagfinder;
				getopt_t * const getopt = tagenv->the_getopt;
				bool const okay = process_all_images(getopt, tagfinder);
				if (! okay)
				{
					printf("\nFATAL: unable to process all images! (main:)\n");
					break;
				}
			}
		}
	}
	else
	{
		fprintf(stderr, "FATAL: catastrophic code error (main:)\n");
	}

	// cleanup environment (memory release)
	tag_env_delete(&tagenv);

	return stat;
}
