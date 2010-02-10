#ifdef HAVE_CONFIG_H
#include "config.h"

const char *libroad_package_string()
{
    static const char package_string[] = PACKAGE_NAME " " PACKAGE_VERSION "-" GIT_VERSION " built on " HOSTNAME " at " BUILD_DATE;
    return package_string;
}

#else

const char *libroad_package_string()
{
    static const char package_string[] = "No config.h available";
    return package_string;
}

#endif


