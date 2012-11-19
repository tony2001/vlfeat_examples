#include <vl/generic.h>
#include <vl/stringop.h>
#include <vl/pgm.h>
#include <vl/sift.h>
#include <vl/getopt_long.h>
#include <vl/mathop.h>
#include <vl/kdtree.h>
#include <unistd.h>

#include "utils.h"

#define VL_OCTAVES_NUM -1
#define VL_LEVELS_PER_OCTAVE 3
#define VL_OCTAVE_MIN -1

#define VL_EDGE_THRESHOLD 7
#define VL_PEAK_THRESHOLD 10

#define VL_FOREST_MAX_COMPARISONS 10
#define VL_MIN_POINT_RATIO 0.3

#define VL_DESC_SIZE 128

inline static int sift_detect_descriptors(FILE *fp, float **pdescr, VlSiftKeypoint **keypts, int *keypts_num, int octaves_num, int levels_per_octave, int octave_min, int edge_threshold, int peak_threshold) /* {{{ */
{
	VlSiftFilt *filt;
	VlPgmImage pim;
	VlSiftKeypoint const *keys;
	vl_bool err;
	vl_uint8 *data;
	vl_sift_pix *fdata;
	float *descr = NULL;
	int nframes = 0, reserved = 0, nkeys = 0;
	int first = 1;
	int i, q;
	VlSiftKeypoint *saved_keys = NULL;
	int saved_keys_num = 0;

	err = vl_pgm_extract_head (fp, &pim);
	if (err) {
		return 1;
	}

	data  = malloc(vl_pgm_get_npixels (&pim) * vl_pgm_get_bpp (&pim) * sizeof(vl_uint8));
	err  = vl_pgm_extract_data (fp, &pim, data);
	if (err) {
		free(data);
		return 1;
	}

	fdata = malloc(vl_pgm_get_npixels (&pim) * vl_pgm_get_bpp (&pim) * sizeof(vl_sift_pix));
	for (i = 0; i < (unsigned) (pim.width * pim.height); ++i) {
		fdata [i] = data [i];
	}

	filt = vl_sift_new (pim.width, pim.height, octaves_num, levels_per_octave, octave_min);
	vl_sift_set_edge_thresh (filt, edge_threshold);
	vl_sift_set_peak_thresh (filt, peak_threshold);

	while (1) {
		int new_nkeys;

		/* calculate the GSS for the next octave .................... */
		if (first) {
			first = 0;
			err = vl_sift_process_first_octave (filt, fdata);
		} else {
			err = vl_sift_process_next_octave (filt);
		}

		if (err) {
			err = VL_ERR_OK;
			break;
		}

		vl_sift_detect (filt);

		keys  = vl_sift_get_keypoints (filt);
		new_nkeys = vl_sift_get_nkeypoints (filt);

		nkeys += new_nkeys;

		/* for each keypoint */
		for (i = 0; i < new_nkeys; ++i) {
			double angles [4];
			int nangles;
			VlSiftKeypoint const *k;

			/* obtain keypoint orientations */
			k = keys + i;
			nangles = vl_sift_calc_keypoint_orientations(filt, angles, k);

			/* For each orientation */
			for (q = 0; q < nangles; ++q) {
				vl_sift_pix buf[VL_DESC_SIZE]; /* this can be only changed along with patching VLFeat lib */

				/* compute descriptor */
				vl_sift_calc_keypoint_descriptor (filt, buf, k, angles [q]);

				/* make enough room for all these keypoints and more */
				if (reserved < nframes + 1) {
					reserved += 2 * nkeys;
					descr  = realloc (descr, VL_DESC_SIZE * sizeof(float) * reserved);
				}

				if (keypts) {
					saved_keys = realloc(saved_keys, sizeof(VlSiftKeypoint) * (saved_keys_num + 1));
					saved_keys[saved_keys_num] = *k;
				}
				if (keypts || keypts_num) {
					saved_keys_num++;
				}

				memcpy(((float *)descr) + VL_DESC_SIZE * nframes, buf, VL_DESC_SIZE * sizeof(vl_sift_pix));
				++ nframes;
			}
		}
	}

	vl_sift_delete(filt);
	*pdescr = descr;
	if (keypts) {
		*keypts = saved_keys;
	}
	if (keypts_num) {
		*keypts_num = saved_keys_num;
	}
	return 0;
}
/* }}} */

int main(int argc, char **argv)
{
	FILE **files;
	char **filenames;
	char timer_buf[32];
	int files_num;
	int i, opt, repeat_num = 1, report_timings = 0;

	int octaves_num = VL_OCTAVES_NUM;
	int levels_per_octave = VL_LEVELS_PER_OCTAVE;
	int octave_min = VL_OCTAVE_MIN;
	int edge_threshold = VL_EDGE_THRESHOLD;
	int peak_threshold = VL_PEAK_THRESHOLD;

	struct timeval tv;
	VlKDForest **forests;
	int *keypts_cnt;
	int *matches;
	float *target_descr;
	int target_keypts_num;

	if (argc < 3) {
		goto print_help_and_exit;
	}

	/* parse options */
	while ((opt = getopt(argc, argv, "hn:tO:L:M:E:P:")) != -1) { /* {{{ */
		switch (opt) {
			case 'h':
				goto print_help_and_exit;
			case 'n':
				repeat_num = atoi(optarg);
				if (repeat_num < 1) {
					utils_error(0, "got meaningless value '%d' for number of loops", repeat_num);
					goto print_help_and_exit;
				}
				break;
			case 't':
				report_timings = 1;
				break;
			case 'O':
				octaves_num = atoi(optarg);
				break;
			case 'L':
				levels_per_octave = atoi(optarg);
				if (levels_per_octave < 1) {
					utils_error(0, "got meaningless value '%d' for number of levels per octave", levels_per_octave);
					goto print_help_and_exit;
				}
				break;
			case 'M':
				octave_min = atoi(optarg);
				break;
			case 'E':
				edge_threshold = atoi(optarg);
				if (edge_threshold < 1) {
					utils_error(0, "got meaningless value '%d' for edge threshold", edge_threshold);
					goto print_help_and_exit;
				}
				break;
			case 'P':
				peak_threshold = atoi(optarg);
				if (peak_threshold < 1) {
					utils_error(0, "got meaningless value '%d' for peak threshold", peak_threshold);
					goto print_help_and_exit;
				}
				break;
		}
	}
	/* }}} */

	/* scan dir */
	if (!utils_scandir(argv[argc - 2], &files, &filenames, &files_num)) {
		return 1;
	}

	if (report_timings) {
		utils_start_timer(&tv);
	}

	forests = calloc(files_num, sizeof(VlKDForest *));
	keypts_cnt = calloc(files_num, sizeof(int));
	matches = calloc(files_num, sizeof(int));

	/* detect keypoints in the images */
	for (i = 0; i < files_num; i++) {
		float *descr;
		int keypts_num, res;
		VlKDForest *forest;
		
		res = sift_detect_descriptors(files[i], &descr, NULL, &keypts_num, octaves_num, levels_per_octave, octave_min, edge_threshold, peak_threshold);
		if (res) {
			utils_error(0, "failed to obtain SIFT descriptors from file %s", files[i]);
			continue;
		}

		keypts_cnt[i] = keypts_num;

		/* we build forest using each image in order to search in those forests */
		forest = vl_kdforest_new(VL_TYPE_FLOAT, 128, 1);
		vl_kdforest_set_thresholding_method(forest, VL_KDTREE_MEAN);
		vl_kdforest_build(forest, keypts_num, descr);
		forests[i] = forest;

		/* printf("%d - %s (%d keypts)\n", i, filenames[i], keypts_num); */
	}

	{
		FILE *fp;

		fp = fopen(argv[argc - 1], "r");
		if (!fp) {
			utils_error(errno, "failed to open file '%s'", argv[argc - 1]);
			exit(EXIT_FAILURE);
		}

		if (sift_detect_descriptors(fp, &target_descr, NULL, &target_keypts_num, octaves_num, levels_per_octave, octave_min, edge_threshold, peak_threshold)) {
			utils_error(0, "failed to obtain SIFT descriptors from file %s", argv[argc - 1]);
			exit(EXIT_FAILURE);
		}
		fclose(fp);

		if (!target_keypts_num) {
			utils_error(0, "no SIFT keypoints found in file %s", argv[argc - 1]);
			exit(EXIT_FAILURE);
		}
	}

	if (report_timings) {
		utils_stop_timer(&tv, timer_buf);
		printf("processed %d files in %s sec\n", files_num + 1, timer_buf);
	}

	if (report_timings) {
		utils_start_timer(&tv);
	}

	for (i = 0; i < repeat_num; i++) {
		int k;

		for (k = 0; k < files_num; k++) {
			int j;
			int matching_pts = 0;

			if (!forests[k]) {
				continue;
			}

			for (j = 0; j < target_keypts_num; j++) {
				VlKDForestNeighbor neighbors[2];
				double ratio;

				vl_kdforest_query (forests[k], neighbors, 2, target_descr + VL_DESC_SIZE*j);
				ratio = neighbors[0].distance / neighbors[1].distance;
				if (ratio < VL_MIN_POINT_RATIO) {
					matching_pts++;
				}
			}
			matches[k] = matching_pts;
		}
	}

	if (report_timings) {
		utils_stop_timer(&tv, timer_buf);
		printf("finished %d search loops in %s sec\n", repeat_num, timer_buf);
	}

	for (i = 0; i < files_num; i++) {
		printf("%s:	%0.2f%% (%d/%d)\n", filenames[i], 100 * (double)matches[i] / (double)target_keypts_num, matches[i], target_keypts_num);
	}
	return 0;

print_help_and_exit:
	printf("Usage: %s [OPTIONS] [DIRECTORY WITH IMAGES] [IMAGE TO LOOK FOR] \n\
\n\
Options:\n\
	-n <uint>  number of times to repeat the search (i.e. benchmark mode)\n\
	-t         print timings\n\
	-h         this help\n\
\n\
	-O <int>   number of octaves, -1 means autodetect (default value: -1)\n\
	-L <uint>  number of levels per octave (default value: 3)\n\
	-M <int>   first octave index (default value: -1)\n\
	-E <uint>  edge threshold (default value: 7)\n\
	-P <uint>  peak threshold (default value: 10)\n\
", argv[0]);
	exit(EXIT_FAILURE);
	return 1;
}
