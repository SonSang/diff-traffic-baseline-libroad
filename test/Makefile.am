noinst_PROGRAMS = road-test interval-test sumo-test

road_test_SOURCES  = road-test.cpp
road_test_CPPFLAGS = $(LIBXML2_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
road_test_LDFLAGS  = $(LDFLAGS)
road_test_LDADD    = $(top_builddir)/libroad/libroad.la

interval_test_SOURCES  = interval-test.cpp
interval_test_CPPFLAGS = $(LIBXML2_CFLAGS) $(BOOST_CPPFLAGS) $(CXXFLAGS) -I$(top_srcdir)
interval_test_LDFLAGS  = $(LDFLAGS)
interval_test_LDADD    = $(top_builddir)/libroad/libroad.la

sumo_test_SOURCES  = sumo-test.cpp
sumo_test_CPPFLAGS = $(LIBXML2_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
sumo_test_LDFLAGS  = $(LDFLAGS)
sumo_test_LDADD    = $(top_builddir)/libroad/libroad.la
