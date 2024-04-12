/* Copyright (C) 2024, Stellacore Corporation.  All rights reserved.

	This file is an extension to the baseline AprilTags Library code:
	which is copyright The Regents of The University of Michigan 2013-2016.

	The original AprilTags software was developed in the APRIL Robotics
	Lab under the direction of Edwin Olson, ebolson@umich.edu. This
	software may be available under alternative licensing terms; contact
	the address above.

	Ref: AprilTags:
	  https://april.eecs.umich.edu/software/apriltag
	  https://github.com/AprilRobotics/apriltag/blob/master/LICENSE.md

	This extension file and its contents are provided under the same
	"BSD 2-Clause License" terms as the original work:

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

//! 3x3 rotation matrix data
struct rmat3
{
	double the_data[9];
};
typedef struct rmat3 rmat3_t;

//! 3d translation vector data
struct tvec3
{
	double the_data[3];
};
typedef struct tvec3 tvec3_t;

// #define RMAT_EL(rmat, row, col) (rmat).the_data[((row)*3u + (col))]
// #define TVEC_EL(tvec, row) (tvec).the_data[(row)]

//! Fill rmat3_t data contents with values from content of matd
void
fill_rmat3_from_matd
	( rmat3_t * const pt_rmat
	, matd_t const * const pt_matd
	)
{
	if ((3u == pt_matd->nrows) && (3u == pt_matd->ncols))
	{
		for (size_t ndx=0 ; ndx < 9u ; ++ndx)
		{
			pt_rmat->the_data[ndx] = pt_matd->data[ndx];
		}
	}
	else
	{
		fprintf(stderr, "Bad dimensions for (fill_rmat3_from_matd):"
				"\npt_rmat size: (%u x %u)"
				"\n   matd size: (%u x %u)"
				"\nFix calling code!\n"
			, 3u, 3u
			, pt_matd->nrows, pt_matd->ncols
			);
		fflush(stderr);
		exit(-1);
	}
}

//! Fill tvec3_t data contents with values from content of matd
void
fill_tvec3_from_matd
	( tvec3_t * const pt_tvec
	, matd_t const * const pt_matd
	)
{
	unsigned int const matd_size = (pt_matd->nrows * pt_matd->ncols);
	if (3u == matd_size)
	{
		pt_tvec->the_data[0] = pt_matd->data[0];
		pt_tvec->the_data[1] = pt_matd->data[1];
		pt_tvec->the_data[2] = pt_matd->data[2];
	}
	else
	{
		fprintf(stderr, "Bad dimensions for (fill_tvec3_from_matd):"
				"\npt_tvec size: (%u)"
				"\n   matd size: (%u)"
				"\nFix calling code!\n"
			, 3u
			, matd_size
			);
		fflush(stderr);
		exit(-1);
	}
}


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

//! TBD convention for pose
struct stack_pose3d
{
	tvec3_t the_tvec; // TODO ?
	rmat3_t the_rmat; // TODO ?
	double the_err;  // rmse image residual (back projection) error?
};
typedef struct stack_pose3d stack_pose3d_t;


//! Perform pose estimation and put results into pt_sr
void
do_pose_estimation
	( stack_pose3d_t * const pt_pose3
	, apriltag_detection_info_t * pt_taginfo
	)
{
	// Use apriltag structure to hold pose estimation results
	tvec3_t * pt_tvec = &(pt_pose3->the_tvec);
	rmat3_t * pt_rmat = &(pt_pose3->the_rmat);
	double * pt_err = &(pt_pose3->the_err);

	// NOTE: estimate_tag_pose() allocates space that is
	//       attached to consumer code data pointers
	//       inside of 'apriltag_post_t' structure.
	// therefore...

	// perform pose estimation
	apriltag_pose_t poseTagWrtCam; // Will require deallocation
	*pt_err = estimate_tag_pose(pt_taginfo, &poseTagWrtCam);

	// ... it is necessary for consumer to free the library-
	//       allocated data values after using (or copying)
	//       the data values into application management space.
	fill_tvec3_from_matd(pt_tvec, poseTagWrtCam.t);
	fill_rmat3_from_matd(pt_rmat, poseTagWrtCam.R);

	// ...   destroy data space allocated from inside
	//       the estimate_tag_pose() call.
	matd_destroy(poseTagWrtCam.R);
	matd_destroy(poseTagWrtCam.t);
}


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
		// space resection result data
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

			stack_pose3d_t pose;
			do_pose_estimation(&pose, &taginfo);
			info(getopt
				, "pose estimation error = %12.9lf\n", pose.the_err);

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
