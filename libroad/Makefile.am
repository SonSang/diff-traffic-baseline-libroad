lib_LTLIBRARIES = libroad.la

libroad_la_SOURCES  = polyline_road.cpp
pkginclude_HEADERS  = partition01.hpp \
		      road_rep.hpp \
		      polyline_road.hpp
libroad_la_CPPFLAGS = $(TVMET_CFLAGS) $(BOOST_CPPFLAGS)
libroad_la_LDFLAGS  = -static
libroad_la_LIBADD   = $(BOOST_LDFLAGS) $(BOOST_SERIALIZATION_LIB) $(LDFLAGS)

