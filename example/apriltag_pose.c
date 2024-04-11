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
	}
	else
	{
		// configure this main app
	//	int quiet = getopt_get_bool(getopt, "quiet");

	// TODO ignore this?
	//	int maxiters = getopt_get_int(getopt, "iters");


		// get configuration information for requested tag family
		char const * famname = getopt_get_string(getopt, "family");
		apriltag_family_t * tagfam = create_tagfamily(famname);
		if (tagfam)
		{
			// create and populate tag detector instance
			apriltag_detector_t * tagfinder = apriltag_detector_create();
			if (configure_tag_detector(tagfinder, getopt, tagfam))
			{

				// process each image in turn
				zarray_t const * const inputs = getopt_get_extra_args(getopt);
				for (int imgNdx = 0; imgNdx < zarray_size(inputs); ++imgNdx)
				{
					// retrieve input path for this iteration
					char const * imgpath = NULL;
					zarray_get(inputs, imgNdx, &imgpath);

					// load input image into memory
					image_u8_t * const tagimg = create_image_from_path(imgpath);
					if (! tagimg)
					{
						printf("\nUnable to load image from path '%s'\n"
							, imgpath);
						continue;
					}
					image_u8_destroy(tagimg);


				}

				stat = 0;
			}
			// cleanup tag detector
			apriltag_detector_destroy(tagfinder);

		}
		destroy_tagfamily(tagfam, famname);
	}

	getopt_destroy(getopt);

	return stat;
}
