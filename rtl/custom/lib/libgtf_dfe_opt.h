/*
 * GTF DFE Optimization - Userspace Library Header
 */

#ifndef LIBGTF_DFE_OPT_H
#define LIBGTF_DFE_OPT_H

#include <stdint.h>
#include "../driver/mqnic_gtf_dfe.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gtf_dfe_handle gtf_dfe_handle_t;

/* Open/close */
gtf_dfe_handle_t *gtf_dfe_open(const char *device);
void gtf_dfe_close(gtf_dfe_handle_t *h);

/* Calibration */
int gtf_dfe_calibrate(gtf_dfe_handle_t *h);

/* Eye scan */
int gtf_dfe_eye_scan(gtf_dfe_handle_t *h);
int gtf_dfe_quick_scan(gtf_dfe_handle_t *h);

/* Tap management */
int gtf_dfe_get_taps(gtf_dfe_handle_t *h, struct gtf_dfe_taps *taps);
int gtf_dfe_set_taps(gtf_dfe_handle_t *h, const struct gtf_dfe_taps *taps);
int gtf_dfe_save_profile(gtf_dfe_handle_t *h, const char *filename);
int gtf_dfe_load_profile(gtf_dfe_handle_t *h, const char *filename);

/* Mode control */
int gtf_dfe_set_mode(gtf_dfe_handle_t *h, enum gtf_dfe_mode mode);

/* Status */
int gtf_dfe_get_status(gtf_dfe_handle_t *h, struct gtf_dfe_status *status);

/* Direct DRP access */
int gtf_dfe_drp_read(gtf_dfe_handle_t *h, uint16_t addr, uint16_t *data);
int gtf_dfe_drp_write(gtf_dfe_handle_t *h, uint16_t addr, uint16_t data);

#ifdef __cplusplus
}
#endif

#endif /* LIBGTF_DFE_OPT_H */
