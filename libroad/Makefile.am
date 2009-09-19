lib_LTLIBRARIES = libroad.la

libroad_la_SOURCES  = polyline_road.cpp \
		      sumo_network.cpp \
		      hwm_network.cpp
pkginclude_HEADERS  = partition01.hpp \
		      road_rep.hpp \
		      polyline_road.hpp \
		      sumo_network.hpp \
		      hwm_network.hpp \
		      xml_util.hpp
libroad_la_CPPFLAGS = $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS)
libroad_la_LDFLAGS  = -static
libroad_la_LIBADD   = $(BOOST_LDFLAGS) $(GLIBMM_LIBS) $(LIBXMLPP_LIBS) $(LDFLAGS)

