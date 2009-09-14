ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libroad.la

libroad_la_SOURCES  = polyline_road.cpp \
		      partition01.hpp
libroad_la_CPPFLAGS = $(TVMET_CFLAGS) $(BOOST_CPPFLAGS)
libroad_la_LDFLAGS  = -static
libroad_la_LIBADD   = $(BOOST_LDFLAGS) $(BOOST_SERIALIZATION_LIB) $(LDFLAGS)

noinst_PROGRAMS = road-test interval-test

road_test_SOURCES  = road-test.cpp
road_test_CPPFLAGS = $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(CXXFLAGS)
road_test_LDFLAGS  = $(LDFLAGS)
road_test_LDADD    = libroad.la

interval_test_SOURCES  = interval-test.cpp
interval_test_CPPFLAGS = $(BOOST_CPPFLAGS) $(CXXFLAGS)
interval_test_LDFLAGS  = $(LDFLAGS)
interval_test_LDADD    = libroad.la

