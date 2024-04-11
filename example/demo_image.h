
#ifndef _demo_image_from_path_INCL_
#define _demo_image_from_path_INCL_
#endif // _demo_image_from_path_INCL_


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

	return im;
}


