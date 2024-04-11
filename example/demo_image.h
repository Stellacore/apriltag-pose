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

#ifndef _demo_image_INCL_
#define _demo_image_INCL_


#include "common/image_u8.h"
#include "common/pjpeg.h"


// Load image from path: (pnm, pgm, jpg supported)
image_u8_t *
create_image_from_path
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

	if (! im)
	{
		printf
			("\nError: Unable to load image! (create_image_from_path:)"
				"\n  Supported formats are 8-bit{ PNM, PGM, JPG }"
				"\n  (Note: this does not include PPM, nor 16-bit images)"
				"\n  Load path is '%s'"
				"\n"
			, path
			);
	}

	return im;
}

#endif // _demo_image_INCL_

