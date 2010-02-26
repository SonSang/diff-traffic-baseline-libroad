lib_LTLIBRARIES = libroad.la

libroad_la_SOURCES  = polyline_road.cpp \
		      arc_road.cpp \
		      sumo_network.cpp \
		      sumo_xml_read.cpp \
		      osm_network.cpp \
		      osm_xml_read.cpp \
		      hwm_network.cpp \
		      hwm_road.cpp \
		      hwm_lane.cpp \
		      hwm_intersection.cpp \
		      hwm_xml_read.cpp \
		      hwm_xml_write.cpp \
		      libroad_common.cpp
pkginclude_HEADERS  = partition01.hpp \
		      road_rep.hpp \
		      polyline_road.hpp \
		      arc_road.hpp \
		      sumo_network.hpp \
		      osm_network.hpp \
		      hwm_network.hpp \
		      xml_util.hpp \
		      libroad_common.hpp \
		      svg_helper.hpp \
		      hwm_draw.hpp

libroad_la_CPPFLAGS = $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS)
libroad_la_LDFLAGS  = -static $(BOOST_IOSTREAMS_LDFLAGS)
libroad_la_LIBADD   = $(GLIBMM_LIBS) $(LIBXMLPP_LIBS)  $(BOOST_IOSTREAMS_LIBS) $(LDFLAGS)

if DO_VISUAL
lib_LTLIBRARIES += libroad_visual.la

libroad_visual_la_SOURCES  = hwm_draw.cpp \
		             osm_draw.cpp

libroad_visual_la_CPPFLAGS = $(TVMET_CFLAGS) $(BOOST_CPPFLAGS) $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS)
libroad_visual_la_LDFLAGS  = -static -lGLU -lGL $(BOOST_IOSTREAMS_LDFLAGS)
libroad_visual_la_LIBADD   = $(GLIBMM_LIBS) $(LIBXMLPP_LIBS) $(BOOST_IOSTREAMS_LIBS) $(LDFLAGS)
endif