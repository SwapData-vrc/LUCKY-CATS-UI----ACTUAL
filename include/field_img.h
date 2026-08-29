#pragma once

#include "liblvgl/lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

// The real VEX field, 232x232, baked from the Game Manual render. Screen
// coordinates: the image spans the full 144 in field, so 232 px / 144 in.
extern const lv_img_dsc_t field_img;

#ifdef __cplusplus
}
#endif
