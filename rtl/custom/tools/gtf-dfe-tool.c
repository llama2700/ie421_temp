/*
 * GTF DFE Optimization - CLI Tool
 *
 * Usage:
 *   gtf-dfe-tool status                    Print current status
 *   gtf-dfe-tool calibrate                 Run DFE calibration
 *   gtf-dfe-tool scan                      Run full eye scan
 *   gtf-dfe-tool quickscan                 Run quick eye scan
 *   gtf-dfe-tool taps                      Print tap values
 *   gtf-dfe-tool mode [lpm|dfe|auto]       Set equalization mode
 *   gtf-dfe-tool profile save <file>       Save tap profile
 *   gtf-dfe-tool profile load <file>       Load tap profile
 *   gtf-dfe-tool drp read <addr>           Read DRP register
 *   gtf-dfe-tool drp write <addr> <data>   Write DRP register
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "../lib/libgtf_dfe_opt.h"

#define DEFAULT_DEVICE "/dev/gtf_dfe0"

static void usage(const char *prog)
{
	fprintf(stderr, "Usage: %s [-d device] <command> [args...]\n\n", prog);
	fprintf(stderr, "Commands:\n");
	fprintf(stderr, "  status                    Print current status\n");
	fprintf(stderr, "  calibrate                 Run DFE calibration\n");
	fprintf(stderr, "  scan                      Run full eye scan\n");
	fprintf(stderr, "  quickscan                 Run quick eye scan\n");
	fprintf(stderr, "  taps                      Print tap values\n");
	fprintf(stderr, "  mode [lpm|dfe|auto]       Set equalization mode\n");
	fprintf(stderr, "  profile save <file>       Save tap profile\n");
	fprintf(stderr, "  profile load <file>       Load tap profile\n");
	fprintf(stderr, "  drp read <addr>           Read DRP register (hex)\n");
	fprintf(stderr, "  drp write <addr> <data>   Write DRP register (hex)\n");
}

static int cmd_status(gtf_dfe_handle_t *h)
{
	struct gtf_dfe_status st;
	int ret;

	ret = gtf_dfe_get_status(h, &st);
	if (ret) {
		fprintf(stderr, "Failed to get status: %s\n", strerror(-ret));
		return 1;
	}

	printf("GTF DFE Optimization Status\n");
	printf("===========================\n");
	printf("Mode:             %s\n",
		st.mode == GTF_DFE_MODE_LPM ? "LPM (CTLE only)" :
		st.mode == GTF_DFE_MODE_DFE ? "DFE (15-tap)" : "Auto");
	printf("Active taps:      %u / %u\n", st.active_taps, GTF_DFE_NUM_TAPS);
	printf("Active mask:      0x%04X\n", st.active_mask);
	printf("Eye height:       %u\n", st.eye_height);
	printf("Eye width:        %u\n", st.eye_width);
	printf("Eye BER:          %u\n", st.eye_ber);
	printf("Buffer bypass:    %s\n", st.buf_bypass ? "aligned" : "disabled");
	printf("FCS check:        %s\n", st.fcs_check ? "enabled" : "disabled");
	printf("TX align hits:    %u\n", st.tx_align_hits);
	printf("TX align misses:  %u\n", st.tx_align_misses);

	if (st.tx_align_hits + st.tx_align_misses > 0) {
		printf("TX align rate:    %.1f%%\n",
			100.0 * st.tx_align_hits / (st.tx_align_hits + st.tx_align_misses));
	}

	printf("Cal busy:         %s\n", (st.status & GTF_DFE_STATUS_CAL_BUSY) ? "yes" : "no");
	printf("CDR lock:         %s\n", (st.status & GTF_DFE_STATUS_CDR_LOCK) ? "yes" : "no");

	return 0;
}

static int cmd_calibrate(gtf_dfe_handle_t *h)
{
	int ret;

	printf("Running DFE calibration...\n");
	ret = gtf_dfe_calibrate(h);
	if (ret) {
		fprintf(stderr, "Calibration failed: %s\n", strerror(-ret));
		return 1;
	}
	printf("Calibration complete.\n");

	/* Print resulting taps */
	struct gtf_dfe_taps taps;
	ret = gtf_dfe_get_taps(h, &taps);
	if (ret == 0) {
		printf("Active taps: %u, mask: 0x%04X\n", taps.count, taps.active_mask);
		for (int i = 0; i < GTF_DFE_NUM_TAPS; i++) {
			if (taps.values[i])
				printf("  TAP%-2d = %u%s\n", i + 1, taps.values[i],
					(taps.active_mask & (1 << i)) ? "" : " (zeroed)");
		}
	}

	return 0;
}

static int cmd_scan(gtf_dfe_handle_t *h, int quick)
{
	struct gtf_dfe_status st;
	int ret;

	printf("Running %s eye scan...\n", quick ? "quick" : "full");
	ret = quick ? gtf_dfe_quick_scan(h) : gtf_dfe_eye_scan(h);
	if (ret) {
		fprintf(stderr, "Eye scan failed: %s\n", strerror(-ret));
		return 1;
	}

	ret = gtf_dfe_get_status(h, &st);
	if (ret == 0) {
		printf("Eye height: %u\n", st.eye_height);
		printf("Eye width:  %u\n", st.eye_width);
		printf("Eye BER:    %u\n", st.eye_ber);
	}

	return 0;
}

static int cmd_taps(gtf_dfe_handle_t *h)
{
	struct gtf_dfe_taps taps;
	int ret, i;

	ret = gtf_dfe_get_taps(h, &taps);
	if (ret) {
		fprintf(stderr, "Failed to read taps: %s\n", strerror(-ret));
		return 1;
	}

	printf("DFE Tap Values (active: %u, mask: 0x%04X)\n", taps.count, taps.active_mask);
	for (i = 0; i < GTF_DFE_NUM_TAPS; i++) {
		printf("  TAP%-2d = %-3u %s\n", i + 1, taps.values[i],
			(taps.active_mask & (1 << i)) ? "[active]" : "");
	}

	return 0;
}

static int cmd_mode(gtf_dfe_handle_t *h, const char *mode_str)
{
	enum gtf_dfe_mode mode;
	int ret;

	if (strcmp(mode_str, "lpm") == 0)
		mode = GTF_DFE_MODE_LPM;
	else if (strcmp(mode_str, "dfe") == 0)
		mode = GTF_DFE_MODE_DFE;
	else if (strcmp(mode_str, "auto") == 0)
		mode = GTF_DFE_MODE_AUTO;
	else {
		fprintf(stderr, "Invalid mode: %s (use lpm, dfe, or auto)\n", mode_str);
		return 1;
	}

	ret = gtf_dfe_set_mode(h, mode);
	if (ret) {
		fprintf(stderr, "Failed to set mode: %s\n", strerror(-ret));
		return 1;
	}

	printf("Mode set to %s\n", mode_str);
	return 0;
}

static int cmd_profile(gtf_dfe_handle_t *h, const char *action, const char *filename)
{
	int ret;

	if (strcmp(action, "save") == 0) {
		ret = gtf_dfe_save_profile(h, filename);
		if (ret) {
			fprintf(stderr, "Failed to save profile: %s\n", strerror(-ret));
			return 1;
		}
		printf("Profile saved to %s\n", filename);
	} else if (strcmp(action, "load") == 0) {
		ret = gtf_dfe_load_profile(h, filename);
		if (ret) {
			fprintf(stderr, "Failed to load profile: %s\n", strerror(-ret));
			return 1;
		}
		printf("Profile loaded from %s\n", filename);
	} else {
		fprintf(stderr, "Invalid profile action: %s (use save or load)\n", action);
		return 1;
	}

	return 0;
}

static int cmd_drp(gtf_dfe_handle_t *h, int argc, char **argv)
{
	uint16_t addr, data;
	int ret;

	if (argc < 1) {
		fprintf(stderr, "drp: need 'read' or 'write'\n");
		return 1;
	}

	if (strcmp(argv[0], "read") == 0) {
		if (argc < 2) {
			fprintf(stderr, "drp read: need address\n");
			return 1;
		}
		addr = (uint16_t)strtoul(argv[1], NULL, 16);
		ret = gtf_dfe_drp_read(h, addr, &data);
		if (ret) {
			fprintf(stderr, "DRP read failed: %s\n", strerror(-ret));
			return 1;
		}
		printf("DRP[0x%04X] = 0x%04X\n", addr, data);
	} else if (strcmp(argv[0], "write") == 0) {
		if (argc < 3) {
			fprintf(stderr, "drp write: need address and data\n");
			return 1;
		}
		addr = (uint16_t)strtoul(argv[1], NULL, 16);
		data = (uint16_t)strtoul(argv[2], NULL, 16);
		ret = gtf_dfe_drp_write(h, addr, data);
		if (ret) {
			fprintf(stderr, "DRP write failed: %s\n", strerror(-ret));
			return 1;
		}
		printf("DRP[0x%04X] <- 0x%04X\n", addr, data);
	} else {
		fprintf(stderr, "drp: unknown action '%s'\n", argv[0]);
		return 1;
	}

	return 0;
}

int main(int argc, char **argv)
{
	const char *device = DEFAULT_DEVICE;
	gtf_dfe_handle_t *h;
	int ret = 1;
	int arg_start = 1;

	if (argc < 2) {
		usage(argv[0]);
		return 1;
	}

	/* Parse -d option */
	if (strcmp(argv[1], "-d") == 0) {
		if (argc < 4) {
			usage(argv[0]);
			return 1;
		}
		device = argv[2];
		arg_start = 3;
	}

	if (arg_start >= argc) {
		usage(argv[0]);
		return 1;
	}

	h = gtf_dfe_open(device);
	if (!h) {
		fprintf(stderr, "Failed to open %s: %s\n", device, strerror(errno));
		return 1;
	}

	const char *cmd = argv[arg_start];

	if (strcmp(cmd, "status") == 0) {
		ret = cmd_status(h);
	} else if (strcmp(cmd, "calibrate") == 0) {
		ret = cmd_calibrate(h);
	} else if (strcmp(cmd, "scan") == 0) {
		ret = cmd_scan(h, 0);
	} else if (strcmp(cmd, "quickscan") == 0) {
		ret = cmd_scan(h, 1);
	} else if (strcmp(cmd, "taps") == 0) {
		ret = cmd_taps(h);
	} else if (strcmp(cmd, "mode") == 0) {
		if (arg_start + 1 >= argc) {
			fprintf(stderr, "mode: need argument (lpm, dfe, auto)\n");
			ret = 1;
		} else {
			ret = cmd_mode(h, argv[arg_start + 1]);
		}
	} else if (strcmp(cmd, "profile") == 0) {
		if (arg_start + 2 >= argc) {
			fprintf(stderr, "profile: need action and filename\n");
			ret = 1;
		} else {
			ret = cmd_profile(h, argv[arg_start + 1], argv[arg_start + 2]);
		}
	} else if (strcmp(cmd, "drp") == 0) {
		ret = cmd_drp(h, argc - arg_start - 1, argv + arg_start + 1);
	} else {
		fprintf(stderr, "Unknown command: %s\n", cmd);
		usage(argv[0]);
		ret = 1;
	}

	gtf_dfe_close(h);
	return ret;
}
