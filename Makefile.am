ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libroad.la

libroad_la_SOURCES  = polyline_road.cpp
libroad_la_CPPFLAGS = $(TVMET_CFLAGS)
libroad_la_LDFLAGS  = -static
libroad_la_LIBADD   = $(BOOST_LDFLAGS) $(LDFLAGS)

noinst_PROGRAMS = road-test

road_test_SOURCES  = road-test.cpp
road_test_CPPFLAGS = $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(CXXFLAGS)
road_test_LDFLAGS  = $(BOOST_LDFLAGS) $(LDFLAGS)
road_test_LDADD    = libroad.la

