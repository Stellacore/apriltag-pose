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
#include "demo_family.h"
#include "demo_options.h"
#include "demo_configure_tag_detector.h"

// library
#include "apriltag.h"
#include "common/image_u8.h"
#include "common/pjpeg.h"
#include "common/zarray.h"

// system
#include <stdio.h>



// Load image from path: (pnm, pgm, jpg supported)
image_u8_t const *
image_from_path
	( char const * const path
	)
{
	image_u8_t *im = NULL;

	if (  str_ends_with(path, "pnm") || str_ends_with(path, "PNM")
	   || str_ends_with(path, "pgm") || str_ends_with(path, "PGM")
	   )
	{
		im = image_u8_create_from_pnm(path);
	}
	else
	if (str_ends_with(path, "jpg") || str_ends_with(path, "JPG"))
	{
		int err = 0;
		pjpeg_t * const pjpeg = pjpeg_create_from_file(path, 0, &err);
		if (pjpeg == NULL)
		{
			printf("pjpeg failed to load: %s, error %d\n", path, err);
			return im; // TODO
		}
		else
		{
			im = pjpeg_to_u8_baseline(pjpeg);
		}
		pjpeg_destroy(pjpeg);
	}

	return im;
}



int
main
	( int argc
	, char *argv[]
	)
{
	int stat = 1;
printf("Hello from : %s, argc = %d\n", argv[0], argc);

	// parse command line options and check invocation
	getopt_t * getopt = apriltag_options();
	if ( (! getopt_parse(getopt, argc, argv, 1))
	   || getopt_get_bool(getopt, "help")
	   )
	{
		printf("Usage: %s [options] <input files>\n", argv[0]);
		getopt_do_usage(getopt);
		exit(0);
	}

	// configure this main app
//	int quiet = getopt_get_bool(getopt, "quiet");

// TODO ignore this?
//	int maxiters = getopt_get_int(getopt, "iters");


	// get configuration information for requested tag family
	char const * famname = getopt_get_string(getopt, "family");
	apriltag_family_t * tagfam = tagfamily_for_name(famname);
	if (tagfam)
	{
		apriltag_detector_t * tagdtor = apriltag_detector_create();
		if (configure_tag_detector(tagdtor, getopt, tagfam))
		{

			// process each image in turn
			zarray_t const * const inputs = getopt_get_extra_args(getopt);
			for (int input = 0; input < zarray_size(inputs); ++input)
			{
				// load input image
				char const * imgpath = NULL;
				zarray_get(inputs, input, &imgpath);

				image_u8_t const * const tagimg = image_from_path(imgpath);
				if (! tagimg)
				{
					printf("\nUnable to load image from path '%s'\n"
						, imgpath);
					continue;
				}


			}

			stat = 0;
		}

	}

	return stat;
}
