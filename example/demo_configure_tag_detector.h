/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

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


#ifndef _demo_configure_tag_detectorINCL_
#define _demo_configure_tag_detectorINCL_


#include "apriltag.h"

#include <errno.h>


bool
configure_tag_detector
	( apriltag_detector_t * const tagdtor
	, getopt_t * const getopt
	, apriltag_family_t * const tagfam
	)
{
	bool okay = false;

	// configure details of specified tag family
	apriltag_detector_add_family_bits
		( tagdtor, tagfam, getopt_get_int(getopt, "hamming"));
	if (errno)
	{
		switch(errno)
		{
			case EINVAL:
				printf("\"hamming\" parameter is out-of-range.\n");
				break;
			case ENOMEM:
				printf("Unable to add family to detector due to"
						" insufficient memory to allocate the tag-family"
						" decoder. Try reducing \"hamming\" from %d or"
						" choose an alternative tag family.\n"
					, getopt_get_int(getopt
					, "hamming")
					);
				break;
		}
	}
	else
	{
		tagdtor->quad_decimate = getopt_get_double(getopt, "decimate");
		tagdtor->quad_sigma = getopt_get_double(getopt, "blur");
		tagdtor->nthreads = getopt_get_int(getopt, "threads");
		tagdtor->debug = getopt_get_bool(getopt, "debug");
		tagdtor->refine_edges = getopt_get_bool(getopt, "refine-edges");
		okay = true;
	}
	return okay;
}

#endif // _demo_configure_tag_detectorINCL_

