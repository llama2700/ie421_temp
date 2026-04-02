/*
 * GTF DFE Optimization - Userspace Library
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "libgtf_dfe_opt.h"

struct gtf_dfe_handle {
	int fd;
	char device[256];
};

gtf_dfe_handle_t *gtf_dfe_open(const char *device)
{
	gtf_dfe_handle_t *h;

	h = calloc(1, sizeof(*h));
	if (!h)
		return NULL;

	h->fd = open(device, O_RDWR);
	if (h->fd < 0) {
		free(h);
		return NULL;
	}

	snprintf(h->device, sizeof(h->device), "%s", device);
	return h;
}

void gtf_dfe_close(gtf_dfe_handle_t *h)
{
	if (h) {
		if (h->fd >= 0)
			close(h->fd);
		free(h);
	}
}

int gtf_dfe_calibrate(gtf_dfe_handle_t *h)
{
	return ioctl(h->fd, GTF_DFE_CALIBRATE) ? -errno : 0;
}

int gtf_dfe_eye_scan(gtf_dfe_handle_t *h)
{
	return ioctl(h->fd, GTF_DFE_EYE_SCAN) ? -errno : 0;
}

int gtf_dfe_quick_scan(gtf_dfe_handle_t *h)
{
	return ioctl(h->fd, GTF_DFE_QUICK_SCAN) ? -errno : 0;
}

int gtf_dfe_get_taps(gtf_dfe_handle_t *h, struct gtf_dfe_taps *taps)
{
	return ioctl(h->fd, GTF_DFE_GET_TAPS, taps) ? -errno : 0;
}

int gtf_dfe_set_taps(gtf_dfe_handle_t *h, const struct gtf_dfe_taps *taps)
{
	return ioctl(h->fd, GTF_DFE_SET_TAPS, taps) ? -errno : 0;
}

int gtf_dfe_set_mode(gtf_dfe_handle_t *h, enum gtf_dfe_mode mode)
{
	int m = (int)mode;
	return ioctl(h->fd, GTF_DFE_SET_MODE, &m) ? -errno : 0;
}

int gtf_dfe_get_status(gtf_dfe_handle_t *h, struct gtf_dfe_status *status)
{
	return ioctl(h->fd, GTF_DFE_GET_STATUS, status) ? -errno : 0;
}

int gtf_dfe_drp_read(gtf_dfe_handle_t *h, uint16_t addr, uint16_t *data)
{
	struct gtf_dfe_drp_xfer xfer = { .addr = addr, .data = 0 };
	int ret;

	ret = ioctl(h->fd, GTF_DFE_DRP_READ, &xfer);
	if (ret)
		return -errno;

	*data = xfer.data;
	return 0;
}

int gtf_dfe_drp_write(gtf_dfe_handle_t *h, uint16_t addr, uint16_t data)
{
	struct gtf_dfe_drp_xfer xfer = { .addr = addr, .data = data };
	return ioctl(h->fd, GTF_DFE_DRP_WRITE, &xfer) ? -errno : 0;
}

int gtf_dfe_save_profile(gtf_dfe_handle_t *h, const char *filename)
{
	struct gtf_dfe_taps taps;
	struct gtf_dfe_status st;
	FILE *f;
	int i, ret;

	ret = gtf_dfe_get_taps(h, &taps);
	if (ret)
		return ret;

	ret = gtf_dfe_get_status(h, &st);
	if (ret)
		return ret;

	f = fopen(filename, "w");
	if (!f)
		return -errno;

	fprintf(f, "# GTF DFE Tap Profile\n");
	fprintf(f, "# Device: %s\n", h->device);
	fprintf(f, "mode=%s\n", st.mode == GTF_DFE_MODE_LPM ? "lpm" :
				st.mode == GTF_DFE_MODE_DFE ? "dfe" : "auto");
	fprintf(f, "tap_count=%u\n", taps.count);
	for (i = 0; i < GTF_DFE_NUM_TAPS; i++)
		fprintf(f, "tap%d=%u\n", i, taps.values[i]);
	fprintf(f, "active_mask=0x%04X\n", taps.active_mask);
	fprintf(f, "buf_bypass=%u\n", st.buf_bypass);
	fprintf(f, "fcs_check=%u\n", st.fcs_check);

	fclose(f);
	return 0;
}

int gtf_dfe_load_profile(gtf_dfe_handle_t *h, const char *filename)
{
	struct gtf_dfe_taps taps;
	FILE *f;
	char line[256];
	int i;

	memset(&taps, 0, sizeof(taps));

	f = fopen(filename, "r");
	if (!f)
		return -errno;

	while (fgets(line, sizeof(line), f)) {
		if (line[0] == '#' || line[0] == '\n')
			continue;

		if (sscanf(line, "tap_count=%hhu", &taps.count) == 1)
			continue;
		if (sscanf(line, "active_mask=0x%hX", &taps.active_mask) == 1)
			continue;

		for (i = 0; i < GTF_DFE_NUM_TAPS; i++) {
			char key[16];
			unsigned int val;
			snprintf(key, sizeof(key), "tap%d=%%u", i);
			if (sscanf(line, key, &val) == 1) {
				taps.values[i] = (uint8_t)val;
				break;
			}
		}
	}

	fclose(f);

	return gtf_dfe_set_taps(h, &taps);
}
