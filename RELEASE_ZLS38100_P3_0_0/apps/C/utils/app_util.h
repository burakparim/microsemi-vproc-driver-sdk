#ifndef __APP_UTIL_H_
#define __APP_UTIL_H__

hbi_status_t vproc_load_image(hbi_handle_t handle, uint8_t *pArg);
#ifdef MICROSEMI_DEBUG_TEST
hbi_status_t save_cfg_to_flash(hbi_handle_t handle, uint8_t image_number);
#endif
#endif


