noinst_PROGRAMS = road-test interval-test sumo-test hwm-test

road_test_SOURCES  = road-test.cpp
road_test_CPPFLAGS = $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
road_test_LDFLAGS  = $(LDFLAGS)
road_test_LDADD    = $(top_builddir)/libroad/libroad.la

interval_test_SOURCES  = interval-test.cpp
interval_test_CPPFLAGS = $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(BOOST_CPPFLAGS) $(CXXFLAGS) -I$(top_srcdir)
interval_test_LDFLAGS  = $(LDFLAGS)
interval_test_LDADD    = $(top_builddir)/libroad/libroad.la

sumo_test_SOURCES  = sumo-test.cpp
sumo_test_CPPFLAGS = $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
sumo_test_LDFLAGS  = $(LDFLAGS)
sumo_test_LDADD    = $(top_builddir)/libroad/libroad.la

hwm_test_SOURCES  = hwm-test.cpp
hwm_test_CPPFLAGS = $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
hwm_test_LDFLAGS  = $(LDFLAGS)
hwm_test_LDADD    = $(top_builddir)/libroad/libroad.la
