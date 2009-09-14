noinst_PROGRAMS = road-test interval-test

road_test_SOURCES  = road-test.cpp
road_test_CPPFLAGS = $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
road_test_LDFLAGS  = $(LDFLAGS)
road_test_LDADD    = $(top_builddir)/libroad/libroad.la

interval_test_SOURCES  = interval-test.cpp
interval_test_CPPFLAGS = $(BOOST_CPPFLAGS) $(CXXFLAGS) -I$(top_srcdir)
interval_test_LDFLAGS  = $(LDFLAGS)
interval_test_LDADD    = $(top_builddir)/libroad/libroad.la

