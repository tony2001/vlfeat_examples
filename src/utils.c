#include "utils.h"

void utils_error_ex(const char *file, int line, int err, const char *format, ...) /* {{{ */
{
	va_list args;
	char tmp_format[V_ERRBUF_SIZE];
	char errormsg[V_ERRBUF_SIZE];

	snprintf(tmp_format, V_ERRBUF_SIZE, "[%s:%d] %s%s%s", file, line, err ? strerror(err) : "", err ? " - " : "" , format);
	va_start(args, format);
	vsnprintf(errormsg, V_ERRBUF_SIZE, tmp_format, args);
	va_end(args);

	fprintf(stderr, "%s\n", errormsg);
	fflush(stderr);
}
/* }}} */

#define V_ALLOC_STEP 16

int utils_scandir(char *dirpath, FILE ***files, char ***filenames, int *files_num) /* {{{ */
{
	DIR *dir;
	struct dirent *dir_entry;
	int fp_alloc = 0;
	int dirpath_len;

	dir = opendir(dirpath);
	if (!dir) {
		utils_error(errno, "failed to open dir '%s'", dirpath);
		return V_FAILURE;
	}

	/* prettify dirname */
	dirpath_len = strlen(dirpath);
	if (dirpath_len > 0 && dirpath[dirpath_len - 1] == '/') {
		dirpath[dirpath_len - 1] = '\0';
	}

	*files = NULL;
	*filenames = NULL;
	*files_num = 0;

	while ((dir_entry = readdir(dir)) != NULL) {
		char full_filename[2048];
		FILE *fp;

		if (dir_entry->d_type != DT_REG) {
			continue;
		}

		snprintf(full_filename, sizeof(full_filename), "%s/%s", dirpath, dir_entry->d_name);

		fp = fopen(full_filename, "r");
		if (!fp) {
			utils_error(errno, "failed to open file '%s'", full_filename);
			continue;
		}

		if (*files_num <= fp_alloc) {
			fp_alloc += V_ALLOC_STEP;
			*files = realloc(*files, sizeof(FILE *) * fp_alloc);
			*filenames = realloc(*filenames, sizeof(char *) * fp_alloc);
		}

		(*files)[*files_num] = fp;
		(*filenames)[*files_num] = strdup(full_filename);
		(*files_num)++;
	}

	if (*files_num != fp_alloc - 1) {
		*files = realloc(*files, sizeof(FILE *) * (*files_num));
		*filenames = realloc(*filenames, sizeof(char *) * (*files_num));
	}
	closedir(dir);
	return V_SUCCESS;
}
/* }}} */

void utils_start_timer(struct timeval *tv) /* {{{ */
{
	gettimeofday(tv, NULL);
}
/* }}} */

void utils_stop_timer(struct timeval *tv, char *buf)
{
	struct timeval end_tv, timer_value;

	gettimeofday(&end_tv, NULL);
	timersub(&end_tv, tv, &timer_value);
	sprintf(buf, "%ld.%ld", timer_value.tv_sec, timer_value.tv_usec);
}

