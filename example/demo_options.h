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

#ifndef _demo_options_INCL_
#define _demo_options_INCL_


#include "common/getopt.h"


//! Populate a consumer getopt_t struct with AprilTag command line options.
void
populate_apriltag_options
    ( getopt_t * getopt
	)
{
    getopt_add_bool(getopt
		, 'h', "help"
		, 0
		, "Show this help"
		);
    getopt_add_bool(getopt
		, 'd', "debug"
		, 0
		, "Enable debugging output (slow)"
		);
    getopt_add_bool(getopt
		, 'q', "quiet"
		, 0
		, "Reduce output"
		);
    getopt_add_string(getopt
		, 'f', "family"
		, "tag36h11"
		, "Tag family to use"
		);
    getopt_add_int(getopt
		, 'i', "iters"
		, "1"
		, "Repeat processing on input set this many times"
		);
    getopt_add_int(getopt
		, 't', "threads"
		, "1"
		, "Use this many CPU threads"
		);
    getopt_add_int(getopt
		, 'a', "hamming"
		, "1"
		, "Detect tags with up to this many bit errors."
		);
    getopt_add_double(getopt
		, 'x', "decimate"
		, "2.0"
		, "Decimate input image by this factor"
		);
    getopt_add_double(getopt
		, 'b', "blur"
		, "0.0"
		, "Apply low-pass blur to input; negative sharpens"
		);
    getopt_add_bool(getopt
		, '0', "refine-edges"
		, 1
		, "Spend more time trying to align edges of tags"
		);
}

#endif // _demo_options_INCL_

