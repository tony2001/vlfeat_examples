
#include <sys/types.h>
#include <sys/time.h>
#include <dirent.h>
#include <stdio.h>
#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#define V_SUCCESS 1
#define V_FAILURE 0
#define V_ERRBUF_SIZE 4096

#define utils_error(err, ...) \
	utils_error_ex(__FILE__, __LINE__, err, __VA_ARGS__)

void utils_error_ex(const char *file, int line, int err, const char *format, ...);
int utils_scandir(char *dirpath, FILE ***files, char ***filenames, int *files_num);
void utils_start_timer(struct timeval *tv); 
void utils_stop_timer(struct timeval *tv, char *buf);
