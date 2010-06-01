noinst_PROGRAMS =
if DO_IMAGE
if DO_VISUAL
noinst_PROGRAMS += arc-chop

arc_chop_SOURCES  = arc-chop.cpp
arc_chop_CPPFLAGS = $(GLIBMM_CFLAGS) $(LIBXMLPP_CFLAGS) $(CAIRO_CFLAGS) $(BOOST_CPPFLAGS) $(TVMET_CFLAGS) $(CXXFLAGS) -I$(top_srcdir)
arc_chop_LDFLAGS  = $(FLTK_LDFLAGS) $(FLTK_GL_LDFLAGS) $(GLEW_LIBS) $(LDFLAGS)
arc_chop_LDADD    = $(top_builddir)/libroad/libroad.la $(top_builddir)/libroad/libroad_visual.la $(top_builddir)/libroad/libroad_image.la
endif
endif