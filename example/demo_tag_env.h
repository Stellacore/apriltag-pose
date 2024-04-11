
#ifndef _demo_tagenv_INCL_
#define _demo_tagenv_INCL_


#include "demo_configure_tag_detector.h"
#include "demo_options.h"

#include "apriltag.h"
#include "common/getopt.h"
#include "common/zarray.h"


/*! \brief Example/demo environment utilized for tag detection.
 *
 * Usage
 * \verbatim
 * struct tag_env * tagenv = tag_env_new();
 * 	// ...
 * tag_env_delete(&tagenv);
 * \endverbatim
*/
struct tag_env
{
	getopt_t * the_getopt; // getopt_create();

	char const * the_tagfam_name; // static space
	apriltag_family_t * the_tagfamily; // create_tagfamily(famname);

	apriltag_detector_t * the_tagfinder; // apriltag_detector_create();

//	image_u8_t * the_tagimg; // create_image_from_path(imgpath);
//	zarray_t * the_detections; // apriltag_detector_detect(tagfinder, tagimg);
};
typedef struct tag_env tag_env_t;


// True if all tagenv members are not null (hopefully this means valid;-)
bool
tag_env_is_valid
	( tag_env_t const * const tagenv
	)
{
	return
		(  tagenv->the_getopt
		&& tagenv->the_tagfam_name
		&& tagenv->the_tagfamily
		&& tagenv->the_tagfinder
		);
}

// Allocate a new tag_env instance
tag_env_t *
tag_env_new
	( char const * const tagfam_name // used as default if no command line arg
	)
{
	tag_env_t * tagenv = malloc(sizeof(tag_env_t));
	if (tagenv)
	{
		tagenv->the_getopt = getopt_create();
		if (tagenv->the_getopt)
		{
			populate_apriltag_options(tagenv->the_getopt);
		}
		tagenv->the_tagfam_name = tagfam_name;
		if (tagenv->the_tagfam_name)
		{
			tagenv->the_tagfamily = NULL;
			tagenv->the_tagfamily = create_tagfamily(tagenv->the_tagfam_name);
		}
		tagenv->the_tagfinder = apriltag_detector_create();
		bool okay = configure_tag_detector
			( tagenv->the_tagfinder
			, tagenv->the_getopt
			, tagenv->the_tagfamily
			);
		if (! okay)
		{
			apriltag_detector_destroy(tagenv->the_tagfinder);
			tagenv->the_tagfinder = NULL;
		}

//		tagenv->the_tagimg = NULL;
//		tagenv->the_detections = NULL;
	}
	return tagenv;
}

// Destory (smartly) the specified tag_env and nullify pt_tagenv after
void
tag_env_delete
	( tag_env_t * * const pt_tagenv
	)
{
	tag_env_t * tagenv = * pt_tagenv;
	if (tagenv)
	{
		if (tagenv->the_getopt)
		{
			getopt_destroy(tagenv->the_getopt);
			tagenv->the_getopt = NULL;
		}

		if (tagenv->the_tagfam_name)
		{
			if (tagenv->the_tagfamily)
			{
				destroy_tagfamily
					(tagenv->the_tagfamily, tagenv->the_tagfam_name);
			}
			tagenv->the_tagfam_name = NULL;
		}

		if (tagenv->the_tagfinder)
		{
			apriltag_detector_destroy(tagenv->the_tagfinder);
			tagenv->the_tagfinder = NULL;
		}
	}
	tagenv = NULL;
}


#endif // _demo_tagenv_INCL_
