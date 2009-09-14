AC_PREREQ([2.59])
AC_INIT([libroad], [0.01], [jasonsewall@gmail.com])
AC_CONFIG_AUX_DIR([config])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_SRCDIR([road_rep.hpp])
AC_CONFIG_HEADERS([config.h])

AC_LANG([C++])

GIT_VERSION=`./GIT-VERSION-GEN`
AC_SUBST(GIT_VERSION)
AH_TEMPLATE([GIT_VERSION],
	    [Git revision of this build])

AC_DEFINE_UNQUOTED(GIT_VERSION, "$GIT_VERSION")

# Checks for programs.
AC_PROG_CXX
AC_PROG_LIBTOOL

AC_CHECK_PROG(have_tvmet_conf, tvmet-config, [yes], [no])
AS_IF([test x"$have_tvmet_conf" != xno],
	    [TVMET_CFLAGS="`tvmet-config --cxxflags` `tvmet-config --includes `"
	    TVMET_LIBS="`tvmet-config --libs`"
	    AC_SUBST([TVMET_CFLAGS])
	    AC_SUBST([TVMET_LIBS])],
       [AC_MSG_ERROR([Could not find tvmet-config. Please install tvmet!])])

# Checks for libraries.
AX_BOOST_BASE([1.39])
AX_BOOST_SERIALIZATION

# Checks for header files.

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_SIZE_T

# Checks for library functions.

AC_CONFIG_FILES([Makefile])
AC_OUTPUT

echo "
  $PACKAGE_NAME version $PACKAGE_VERSION-$GIT_VERSION
  Prefix...................: $prefix
  TVMET_CFLAGS.............: $TVMET_CFLAGS
  TVMET_LIBS...............: $TVMET_LIBS
  BOOST_CPPFLAGS...........: $BOOST_CPPFLAGS
  BOOST_LDFLAGS............: $BOOST_LDFLAGS
  BOOST_SERIALIZATION_LIB..: $BOOST_SERIALIZATION_LIB
  C++ Compiler.............: $CXX $CXXFLAGS $CPPFLAGS
  Linker...................: $LD $LDFLAGS $LIBS"
