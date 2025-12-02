#ifndef HW_XBOX360_STORAGE_H
#define HW_XBOX360_STORAGE_H

#include "hw/xbox360/xbox360.h"

/* Public functions */
void xenon_storage_init(XenonState *s, const char *hdd_path,
                       const char *dvd_path, Error **errp);
void xenon_storage_cleanup(XenonState *s);

#endif
