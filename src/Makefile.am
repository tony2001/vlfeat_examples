AUTOMAKE_OPTIONS = foreign

COMMON_SRC = utils.c

bin_PROGRAMS = sift_example 
#dsift_example
if HAVE_LIBSPINDLE
  bin_PROGRAMS += sift_example_mt dsift_example_mt 
endif

sift_example_SOURCES = sift_example.c $(COMMON_SRC)
#dsift_example_SOURCES = dsift_example.c $(COMMON_SRC)

if HAVE_LIBSPINDLE
  sift_example_mt_SOURCES = sift_example_mt.c
  dsift_example_mt_SOURCES = dsift_example_mt.c
endif
