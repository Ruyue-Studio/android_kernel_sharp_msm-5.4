/*
 * Copyright (C) 2021 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <video/mipi_display.h>
#include "../dsi/dsi_display.h"
#include "../dsi/dsi_panel.h"
#include "../msm_drv.h"
#include "../sharp/drm_cmn.h"
#include "drm_mipi_dsi.h"
#include "drm_gmm.h"

/* ------------------------------------------------------------------------- */
/* FUNCTION                                                                  */
/* ------------------------------------------------------------------------- */

static int drm_gmm_get_gamma_table(struct dsi_panel *panel, u32 preset);
static void drm_gmm_calc_gamma_table(void);
static void drm_gmm_convert_short_gamma_table(u32 band);
static void drm_gmm_convert_tone_table(void);
static void drm_gmm_calc_revise(void);
static void drm_gmm_convert_short_write_table(void);
static void drm_gmm_convert_write_table(u32 band);
static void drm_gmm_default_gamma_table(void);

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define DRM_GMM_COLOR_TONE_35           35
#define DRM_GMM_REGISTER_PARAM_24       24
#define DRM_GMM_SHORT_PARAM_12          DRM_GMM_REGISTER_PARAM_24 / 2
#define DRM_GMM_BAND_COUNT_10           10
#define DRM_GMM_ADDR_COUNT_9            9
#define DRM_GMM_TRI_COLOR_3             3
#define DRM_GMM_CALC_COEFFICIENT        100
#define DRM_GMM_MAX_VALUE_0xFFF         0xFFF
#define DRM_GMM_MIN_VALUE_0x0           0x0

/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
static int work_short_array9x12[DRM_GMM_ADDR_COUNT_9][DRM_GMM_SHORT_PARAM_12] = {0};
static int work_tone_array3x35[DRM_GMM_TRI_COLOR_3][DRM_GMM_COLOR_TONE_35] = {0};
static unsigned char gamma_array10x9x24[DRM_GMM_BAND_COUNT_10][DRM_GMM_ADDR_COUNT_9][DRM_GMM_REGISTER_PARAM_24] = {0};
static unsigned char read_array9x24[DRM_GMM_ADDR_COUNT_9][DRM_GMM_REGISTER_PARAM_24] = {0};
static unsigned char write_buf9x25[DRM_GMM_ADDR_COUNT_9][DRM_GMM_REGISTER_PARAM_24 + 1] = {0};
static unsigned char addr_value_page[] =                    {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
static unsigned char addr_enable[] =                        {0xCC, 0x00};
static unsigned char addr_gamma_band[] =                    {0xBF, 0x00};
static unsigned char addr_gamma[DRM_GMM_ADDR_COUNT_9][2] = {{0xB0, 24},
															{0xB1, 24},
															{0xB2, 22},
															{0xB3, 24},
															{0xB4, 24},
															{0xB5, 22},
															{0xB6, 24},
															{0xB7, 24},
															{0xB8, 22},
};

static const int revise_gamma_x100[DRM_GMM_TRI_COLOR_3][DRM_GMM_COLOR_TONE_35] = {
	{0, 0, 0,
	-29059, -29059, -28476, -27937, -27427, -26940, -26468, -26010,
	-25562, -24690, -23843, -23014, -22198, -21393, -19808, -18250,
	-16717, -13725, -10856, -8148, -5648, -3408, -1484, 064,
	1177, 1793, 1893, 1845, 1640, 1270, 101, 000},
	{0, 0, 0,
	-29059, -29059, -28476, -27937, -27427, -26940, -26468, -26010,
	-25562, -24690, -23843, -23014, -22198, -21393, -19808, -18250,
	-16717, -13725, -10856, -8148, -5648, -3408, -1484, 064,
	1177, 1793, 1893, 1845, 1640, 1270, 101, 000},
	{0, 0, 0,
	-29059, -29059, -28476, -27937, -27427, -26940, -26468, -26010,
	-25562, -24690, -23843, -23014, -22198, -21393, -19808, -18250,
	-16717, -13725, -10856, -8148, -5648, -3408, -1484, 064,
	1177, 1793, 1893, 1845, 1640, 1270, 101, 000},
};
static unsigned char gamma_default_array10x9x24[DRM_GMM_BAND_COUNT_10][DRM_GMM_ADDR_COUNT_9][DRM_GMM_REGISTER_PARAM_24] = {
	{{0x00, 0x00, 0x00, 0xD0, 0x01, 0x4E, 0x01, 0xA1, 0x01, 0xA9, 0x01, 0xB2, 0x01, 0xBB, 0x01, 0xC4, 0x01, 0xCC, 0x01, 0xD5, 0x01, 0xDE, 0x01, 0xE7},
	{0x01, 0xF9, 0x02, 0x0B, 0x02, 0x1F, 0x02, 0x35, 0x02, 0x49, 0x02, 0x71, 0x02, 0x98, 0x02, 0xC6, 0x03, 0x2D, 0x03, 0x97, 0x04, 0x03, 0x04, 0x6E},
	{0x04, 0xDE, 0x05, 0x4E, 0x05, 0xC5, 0x06, 0x3F, 0x06, 0xBE, 0x06, 0xFF, 0x07, 0x43, 0x07, 0x87, 0x07, 0xCC, 0x08, 0x52, 0x08, 0x5B, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x89, 0x00, 0xDB, 0x01, 0x12, 0x01, 0x1A, 0x01, 0x22, 0x01, 0x2A, 0x01, 0x32, 0x01, 0x3A, 0x01, 0x43, 0x01, 0x4B, 0x01, 0x53},
	{0x01, 0x64, 0x01, 0x74, 0x01, 0x87, 0x01, 0x9B, 0x01, 0xAD, 0x01, 0xD2, 0x01, 0xF7, 0x02, 0x22, 0x02, 0x82, 0x02, 0xE5, 0x03, 0x4A, 0x03, 0xAE},
	{0x04, 0x18, 0x04, 0x82, 0x04, 0xF3, 0x05, 0x67, 0x05, 0xE1, 0x06, 0x1F, 0x06, 0x61, 0x06, 0xA3, 0x06, 0xE6, 0x07, 0x68, 0x07, 0x70, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0xD4, 0x01, 0x54, 0x01, 0xA9, 0x01, 0xB6, 0x01, 0xC3, 0x01, 0xD1, 0x01, 0xDE, 0x01, 0xEC, 0x01, 0xF9, 0x02, 0x06, 0x02, 0x14},
	{0x02, 0x30, 0x02, 0x4A, 0x02, 0x69, 0x02, 0x8C, 0x02, 0xAB, 0x02, 0xEE, 0x03, 0x2E, 0x03, 0x74, 0x03, 0xFD, 0x04, 0x76, 0x04, 0xE7, 0x05, 0x52},
	{0x05, 0xC0, 0x06, 0x31, 0x06, 0xAB, 0x07, 0x29, 0x07, 0xAE, 0x07, 0xF1, 0x08, 0x38, 0x08, 0x7E, 0x08, 0xC6, 0x09, 0x4F, 0x09, 0x58, 0x00, 0x00},},

	{{0x00, 0x00, 0x00, 0x71, 0x00, 0xB5, 0x00, 0xE3, 0x00, 0xF2, 0x01, 0x01, 0x01, 0x0F, 0x01, 0x1E, 0x01, 0x2E, 0x01, 0x3D, 0x01, 0x4C, 0x01, 0x5C},
	{0x01, 0x7A, 0x01, 0x9A, 0x01, 0xBC, 0x01, 0xDF, 0x02, 0x03, 0x02, 0x4E, 0x02, 0xA1, 0x02, 0xF7, 0x03, 0x9A, 0x04, 0x35, 0x04, 0xCD, 0x05, 0x65},
	{0x05, 0xF8, 0x06, 0x88, 0x07, 0x18, 0x07, 0xA8, 0x08, 0x39, 0x08, 0x81, 0x08, 0xCA, 0x09, 0x13, 0x09, 0x5B, 0x09, 0xE3, 0x09, 0xEC, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x2B, 0x00, 0x45, 0x00, 0x57, 0x00, 0x64, 0x00, 0x72, 0x00, 0x80, 0x00, 0x8E, 0x00, 0x9C, 0x00, 0xAA, 0x00, 0xB8, 0x00, 0xC7},
	{0x00, 0xE3, 0x01, 0x01, 0x01, 0x21, 0x01, 0x41, 0x01, 0x63, 0x01, 0xA9, 0x01, 0xF6, 0x02, 0x46, 0x02, 0xDE, 0x03, 0x6E, 0x03, 0xFD, 0x04, 0x8B},
	{0x05, 0x16, 0x05, 0xA0, 0x06, 0x29, 0x06, 0xB3, 0x07, 0x3E, 0x07, 0x84, 0x07, 0xC9, 0x08, 0x10, 0x08, 0x55, 0x08, 0xD9, 0x08, 0xE1, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x64, 0x00, 0xA0, 0x00, 0xC8, 0x00, 0xE0, 0x00, 0xF8, 0x01, 0x10, 0x01, 0x28, 0x01, 0x41, 0x01, 0x59, 0x01, 0x72, 0x01, 0x8A},
	{0x01, 0xBB, 0x01, 0xED, 0x02, 0x23, 0x02, 0x5B, 0x02, 0x96, 0x03, 0x05, 0x03, 0x71, 0x03, 0xD4, 0x04, 0x7E, 0x05, 0x18, 0x05, 0xB3, 0x06, 0x51},
	{0x06, 0xED, 0x07, 0x84, 0x08, 0x1A, 0x08, 0xB0, 0x09, 0x47, 0x09, 0x93, 0x09, 0xDE, 0x0A, 0x2A, 0x0A, 0x75, 0x0B, 0x02, 0x0B, 0x0C, 0x00, 0x00},},

	{{0x00, 0x00, 0x00, 0x72, 0x00, 0xB7, 0x00, 0xE5, 0x00, 0xF8, 0x01, 0x0B, 0x01, 0x1F, 0x01, 0x32, 0x01, 0x46, 0x01, 0x59, 0x01, 0x6D, 0x01, 0x81},
	{0x01, 0xA9, 0x01, 0xD1, 0x01, 0xFD, 0x02, 0x2B, 0x02, 0x5A, 0x02, 0xC6, 0x03, 0x2D, 0x03, 0x91, 0x04, 0x4B, 0x04, 0xFC, 0x05, 0xA5, 0x06, 0x46},
	{0x06, 0xE3, 0x07, 0x7E, 0x08, 0x19, 0x08, 0xB0, 0x09, 0x47, 0x09, 0x93, 0x09, 0xE1, 0x0A, 0x2D, 0x0A, 0x7B, 0x0B, 0x0D, 0x0B, 0x17, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x2D, 0x00, 0x49, 0x00, 0x5B, 0x00, 0x6D, 0x00, 0x7E, 0x00, 0x91, 0x00, 0xA2, 0x00, 0xB5, 0x00, 0xC6, 0x00, 0xD9, 0x00, 0xEB},
	{0x01, 0x10, 0x01, 0x36, 0x01, 0x5E, 0x01, 0x88, 0x01, 0xB5, 0x02, 0x19, 0x02, 0x78, 0x02, 0xD4, 0x03, 0x80, 0x04, 0x24, 0x04, 0xC2, 0x05, 0x5A},
	{0x05, 0xEF, 0x06, 0x82, 0x07, 0x15, 0x07, 0xA6, 0x08, 0x35, 0x08, 0x7D, 0x08, 0xC6, 0x09, 0x0E, 0x09, 0x58, 0x09, 0xE3, 0x09, 0xEC, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x55, 0x00, 0x88, 0x00, 0xAA, 0x00, 0xCB, 0x00, 0xEB, 0x01, 0x0D, 0x01, 0x2D, 0x01, 0x4F, 0x01, 0x6F, 0x01, 0x91, 0x01, 0xB2},
	{0x01, 0xF5, 0x02, 0x38, 0x02, 0x82, 0x02, 0xCC, 0x03, 0x12, 0x03, 0x9D, 0x04, 0x10, 0x04, 0x75, 0x05, 0x2E, 0x05, 0xE6, 0x06, 0x9B, 0x07, 0x45},
	{0x07, 0xEB, 0x08, 0x8E, 0x09, 0x30, 0x09, 0xD0, 0x0A, 0x6E, 0x0A, 0xBE, 0x0B, 0x0E, 0x0B, 0x5E, 0x0B, 0xAF, 0x0C, 0x46, 0x0C, 0x4F, 0x00, 0x00},},

	{{0x00, 0x00, 0x00, 0x76, 0x00, 0xBD, 0x00, 0xEC, 0x01, 0x03, 0x01, 0x1A, 0x01, 0x30, 0x01, 0x47, 0x01, 0x5F, 0x01, 0x76, 0x01, 0x8C, 0x01, 0xA4,},
	{0x01, 0xD3, 0x02, 0x01, 0x02, 0x35, 0x02, 0x6B, 0x02, 0xA9, 0x03, 0x23, 0x03, 0x95, 0x04, 0x01, 0x04, 0xCB, 0x05, 0x87, 0x06, 0x34, 0x06, 0xDD},
	{0x07, 0x81, 0x08, 0x21, 0x08, 0xBC, 0x09, 0x5A, 0x09, 0xF8, 0x0A, 0x48, 0x0A, 0x98, 0x0A, 0xE5, 0x0B, 0x34, 0x0B, 0xCB, 0x0B, 0xD4, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x32, 0x00, 0x50, 0x00, 0x64, 0x00, 0x79, 0x00, 0x8E, 0x00, 0xA3, 0x00, 0xB8, 0x00, 0xCE, 0x00, 0xE3, 0x00, 0xF8, 0x01, 0x0E},
	{0x01, 0x39, 0x01, 0x63, 0x01, 0x93, 0x01, 0xC6, 0x01, 0xFF, 0x02, 0x6F, 0x02, 0xD8, 0x03, 0x3B, 0x03, 0xF6, 0x04, 0xA4, 0x05, 0x47, 0x05, 0xE5},
	{0x06, 0x80, 0x07, 0x17, 0x07, 0xAA, 0x08, 0x3D, 0x08, 0xD2, 0x09, 0x1E, 0x09, 0x69, 0x09, 0xB3, 0x09, 0xFE, 0x0A, 0x8D, 0x0A, 0x96, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x53, 0x00, 0x86, 0x00, 0xA7, 0x00, 0xCE, 0x00, 0xF4, 0x01, 0x1B, 0x01, 0x41, 0x01, 0x69, 0x01, 0x8F, 0x01, 0xB6, 0x01, 0xDD},
	{0x02, 0x2B, 0x02, 0x79, 0x02, 0xCF, 0x03, 0x22, 0x03, 0x75, 0x04, 0x04, 0x04, 0x79, 0x04, 0xE3, 0x05, 0xB4, 0x06, 0x7D, 0x07, 0x35, 0x07, 0xE8},
	{0x08, 0x96, 0x09, 0x3E, 0x09, 0xE3, 0x0A, 0x89, 0x0B, 0x2E, 0x0B, 0x81, 0x0B, 0xD4, 0x0C, 0x26, 0x0C, 0x78, 0x0D, 0x16, 0x0D, 0x20, 0x00, 0x00},},

	{{0x00, 0x00, 0x00, 0x77, 0x00, 0xBE, 0x00, 0xEE, 0x01, 0x0A, 0x01, 0x24, 0x01, 0x3F, 0x01, 0x59, 0x01, 0x74, 0x01, 0x8F, 0x01, 0xAA, 0x01, 0xC5},
	{0x01, 0xFB, 0x02, 0x32, 0x02, 0x6E, 0x02, 0xB0, 0x02, 0xF6, 0x03, 0x78, 0x03, 0xF0, 0x04, 0x61, 0x05, 0x35, 0x05, 0xF5, 0x06, 0xA9, 0x07, 0x55},
	{0x07, 0xF9, 0x08, 0x9B, 0x09, 0x3B, 0x09, 0xDE, 0x0A, 0x7D, 0x0A, 0xCB, 0x0B, 0x1C, 0x0B, 0x6E, 0x0B, 0xC0, 0x0C, 0x5E, 0x0C, 0x69, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x34, 0x00, 0x54, 0x00, 0x69, 0x00, 0x82, 0x00, 0x9A, 0x00, 0xB3, 0x00, 0xCB, 0x00, 0xE3, 0x00, 0xFC, 0x01, 0x15, 0x01, 0x2E},
	{0x01, 0x60, 0x01, 0x93, 0x01, 0xCB, 0x02, 0x08, 0x02, 0x48, 0x02, 0xC0, 0x03, 0x2D, 0x03, 0x95, 0x04, 0x58, 0x05, 0x0B, 0x05, 0xB3, 0x06, 0x54},
	{0x06, 0xEF, 0x07, 0x86, 0x08, 0x1B, 0x08, 0xB3, 0x09, 0x49, 0x09, 0x93, 0x09, 0xE0, 0x0A, 0x2C, 0x0A, 0x7A, 0x0B, 0x0C, 0x0B, 0x17, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x51, 0x00, 0x82, 0x00, 0xA3, 0x00, 0xCF, 0x00, 0xFB, 0x01, 0x27, 0x01, 0x53, 0x01, 0x7F, 0x01, 0xAB, 0x01, 0xD8, 0x02, 0x04},
	{0x02, 0x5D, 0x02, 0xB7, 0x03, 0x19, 0x03, 0x76, 0x03, 0xCE, 0x04, 0x5D, 0x04, 0xD3, 0x05, 0x44, 0x06, 0x25, 0x06, 0xF3, 0x07, 0xB2, 0x08, 0x69},
	{0x09, 0x18, 0x09, 0xC3, 0x0A, 0x6D, 0x0B, 0x17, 0x0B, 0xBF, 0x0C, 0x11, 0x0C, 0x66, 0x0C, 0xBC, 0x0D, 0x12, 0x0D, 0xB9, 0x0D, 0xC5, 0x00, 0x00},},

	{{0x00, 0x00, 0x00, 0x7C, 0x00, 0xC7, 0x00, 0xF8, 0x01, 0x16, 0x01, 0x33, 0x01, 0x50, 0x01, 0x6F, 0x01, 0x8C, 0x01, 0xAA, 0x01, 0xC7, 0x01, 0xE5},
	{0x02, 0x20, 0x02, 0x5C, 0x02, 0x9E, 0x02, 0xE9, 0x03, 0x34, 0x03, 0xBD, 0x04, 0x3B, 0x04, 0xB2, 0x05, 0x8B, 0x06, 0x4E, 0x07, 0x05, 0x07, 0xB3},
	{0x08, 0x5A, 0x08, 0xFE, 0x09, 0xA2, 0x0A, 0x42, 0x0A, 0xE4, 0x0B, 0x36, 0x0B, 0x8A, 0x0B, 0xDE, 0x0C, 0x34, 0x0C, 0xD7, 0x0C, 0xE2, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x3A, 0x00, 0x5D, 0x00, 0x74, 0x00, 0x8F, 0x00, 0xAA, 0x00, 0xC5, 0x00, 0xE1, 0x00, 0xFB, 0x01, 0x17, 0x01, 0x32, 0x01, 0x4E},
	{0x01, 0x84, 0x01, 0xBC, 0x01, 0xF9, 0x02, 0x3E, 0x02, 0x83, 0x03, 0x00, 0x03, 0x73, 0x03, 0xE0, 0x04, 0xA8, 0x05, 0x5E, 0x06, 0x08, 0x06, 0xAC},
	{0x07, 0x47, 0x07, 0xDE, 0x08, 0x76, 0x09, 0x0D, 0x09, 0xA5, 0x09, 0xF2, 0x0A, 0x3F, 0x0A, 0x8C, 0x0A, 0xDC, 0x0B, 0x76, 0x0B, 0x80, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x5D, 0x00, 0x96, 0x00, 0xBB, 0x00, 0xEB, 0x01, 0x19, 0x01, 0x47, 0x01, 0x76, 0x01, 0xA5, 0x01, 0xD4, 0x02, 0x02, 0x02, 0x31},
	{0x02, 0x8F, 0x02, 0xED, 0x03, 0x55, 0x03, 0xB9, 0x04, 0x12, 0x04, 0xA1, 0x05, 0x1E, 0x05, 0x98, 0x06, 0x82, 0x07, 0x52, 0x08, 0x15, 0x08, 0xCF},
	{0x09, 0x81, 0x0A, 0x30, 0x0A, 0xDB, 0x0B, 0x85, 0x0C, 0x30, 0x0C, 0x87, 0x0C, 0xDF, 0x0D, 0x39, 0x0D, 0x94, 0x0E, 0x3F, 0x0E, 0x4B, 0x00, 0x00},},

	{{0x00, 0x00, 0x00, 0x6B, 0x00, 0xAC, 0x00, 0xD7, 0x00, 0xF9, 0x01, 0x1B, 0x01, 0x3E, 0x01, 0x61, 0x01, 0x83, 0x01, 0xA5, 0x01, 0xC9, 0x01, 0xEB},
	{0x02, 0x30, 0x02, 0x77, 0x02, 0xC4, 0x03, 0x15, 0x03, 0x61, 0x03, 0xF2, 0x04, 0x77, 0x04, 0xF2, 0x05, 0xD1, 0x06, 0x9A, 0x07, 0x55, 0x08, 0x04},
	{0x08, 0xAE, 0x09, 0x57, 0x09, 0xF9, 0x0A, 0x9C, 0x0B, 0x44, 0x0B, 0x99, 0x0B, 0xEF, 0x0C, 0x45, 0x0C, 0x9C, 0x0D, 0x39, 0x0D, 0x44, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x2B, 0x00, 0x45, 0x00, 0x56, 0x00, 0x75, 0x00, 0x95, 0x00, 0xB4, 0x00, 0xD5, 0x00, 0xF5, 0x01, 0x14, 0x01, 0x35, 0x01, 0x54},
	{0x01, 0x94, 0x01, 0xD5, 0x02, 0x1D, 0x02, 0x67, 0x02, 0xAD, 0x03, 0x31, 0x03, 0xAB, 0x04, 0x1B, 0x04, 0xE9, 0x05, 0xA4, 0x06, 0x52, 0x06, 0xF6},
	{0x07, 0x92, 0x08, 0x2D, 0x08, 0xC5, 0x09, 0x5D, 0x09, 0xF8, 0x0A, 0x46, 0x0A, 0x96, 0x0A, 0xE6, 0x0B, 0x37, 0x0B, 0xC8, 0x0B, 0xD2, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x4F, 0x00, 0x7E, 0x00, 0x9E, 0x00, 0xD2, 0x01, 0x06, 0x01, 0x39, 0x01, 0x6E, 0x01, 0xA2, 0x01, 0xD6, 0x02, 0x0B, 0x02, 0x3F},
	{0x02, 0xA8, 0x03, 0x11, 0x03, 0x85, 0x03, 0xEC, 0x04, 0x43, 0x04, 0xD6, 0x05, 0x5A, 0x05, 0xDB, 0x06, 0xCC, 0x07, 0xA4, 0x08, 0x6C, 0x09, 0x27},
	{0x09, 0xDD, 0x0A, 0x8F, 0x0B, 0x3C, 0x0B, 0xE8, 0x0C, 0x9A, 0x0C, 0xF5, 0x0D, 0x51, 0x0D, 0xAB, 0x0E, 0x07, 0x0E, 0xB0, 0x0E, 0xBC, 0x00, 0x00},},

	{{0x00, 0x00, 0x00, 0x5A, 0x00, 0x90, 0x00, 0xB4, 0x00, 0xDC, 0x01, 0x05, 0x01, 0x2D, 0x01, 0x57, 0x01, 0x7F, 0x01, 0xA8, 0x01, 0xD1, 0x01, 0xFA},
	{0x02, 0x4C, 0x02, 0x9E, 0x02, 0xF9, 0x03, 0x54, 0x03, 0xAA, 0x04, 0x47, 0x04, 0xD6, 0x05, 0x5C, 0x06, 0x46, 0x07, 0x1B, 0x07, 0xDC, 0x08, 0x92},
	{0x09, 0x46, 0x09, 0xEF, 0x0A, 0x99, 0x0B, 0x47, 0x0B, 0xF5, 0x0C, 0x49, 0x0C, 0x9B, 0x0C, 0xEF, 0x0D, 0x46, 0x0D, 0xF1, 0x0D, 0xFD, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x1B, 0x00, 0x2B, 0x00, 0x36, 0x00, 0x5B, 0x00, 0x80, 0x00, 0xA6, 0x00, 0xCC, 0x00, 0xF1, 0x01, 0x17, 0x01, 0x3D, 0x01, 0x62},
	{0x01, 0xAE, 0x01, 0xFA, 0x02, 0x4E, 0x02, 0xA2, 0x02, 0xF0, 0x03, 0x7F, 0x04, 0x01, 0x04, 0x7C, 0x05, 0x55, 0x06, 0x1A, 0x06, 0xCD, 0x07, 0x74},
	{0x08, 0x19, 0x08, 0xB6, 0x09, 0x53, 0x09, 0xF0, 0x0A, 0x91, 0x0A, 0xDE, 0x0B, 0x28, 0x0B, 0x74, 0x0B, 0xC1, 0x0C, 0x58, 0x0C, 0x63, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x54, 0x00, 0x87, 0x00, 0xA9, 0x00, 0xE1, 0x01, 0x19, 0x01, 0x50, 0x01, 0x89, 0x01, 0xC1, 0x01, 0xF9, 0x02, 0x32, 0x02, 0x6A},
	{0x02, 0xDB, 0x03, 0x4C, 0x03, 0xC9, 0x04, 0x35, 0x04, 0x8E, 0x05, 0x29, 0x05, 0xBE, 0x06, 0x4E, 0x07, 0x4A, 0x08, 0x2F, 0x08, 0xFE, 0x09, 0xC2},
	{0x0A, 0x81, 0x0B, 0x36, 0x0B, 0xEB, 0x0C, 0xA6, 0x0D, 0x5F, 0x0D, 0xBA, 0x0E, 0x14, 0x0E, 0x6F, 0x0E, 0xCD, 0x0F, 0x87, 0x0F, 0x94, 0x00, 0x00},},

	{{0x00, 0x00, 0x00, 0x16, 0x00, 0x2D, 0x00, 0x43, 0x00, 0x71, 0x00, 0x9C, 0x00, 0xC7, 0x00, 0xF3, 0x01, 0x1E, 0x01, 0x4A, 0x01, 0x76, 0x01, 0xA1},
	{0x01, 0xF8, 0x02, 0x50, 0x02, 0xA7, 0x03, 0x04, 0x03, 0x6E, 0x04, 0x2B, 0x04, 0xD2, 0x05, 0x6C, 0x06, 0x73, 0x07, 0x57, 0x08, 0x21, 0x08, 0xDC},
	{0x09, 0x8B, 0x0A, 0x2E, 0x0A, 0xD4, 0x0B, 0x7B, 0x0C, 0x1E, 0x0C, 0x6F, 0x0C, 0xC6, 0x0D, 0x21, 0x0D, 0x85, 0x0E, 0x62, 0x0E, 0x72, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x00, 0x4F, 0x00, 0x77, 0x00, 0x9F, 0x00, 0xC7, 0x00, 0xEF, 0x01, 0x17},
	{0x01, 0x67, 0x01, 0xB8, 0x02, 0x08, 0x02, 0x5E, 0x02, 0xBF, 0x03, 0x6C, 0x04, 0x05, 0x04, 0x91, 0x05, 0x83, 0x06, 0x57, 0x07, 0x12, 0x07, 0xBB},
	{0x08, 0x5A, 0x08, 0xF0, 0x09, 0x85, 0x0A, 0x1C, 0x0A, 0xAF, 0x0A, 0xF7, 0x0B, 0x44, 0x0B, 0x95, 0x0B, 0xED, 0x0C, 0xA8, 0x0C, 0xB6, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x4E, 0x00, 0x96, 0x00, 0xE0, 0x01, 0x29, 0x01, 0x73},
	{0x02, 0x04, 0x02, 0x97, 0x03, 0x2A, 0x03, 0xBE, 0x04, 0x47, 0x05, 0x0F, 0x05, 0xB6, 0x06, 0x5A, 0x07, 0x75, 0x08, 0x6B, 0x09, 0x45, 0x0A, 0x0E},
	{0x0A, 0xC9, 0x0B, 0x7A, 0x0C, 0x2D, 0x0C, 0xE1, 0x0D, 0x93, 0x0D, 0xEC, 0x0E, 0x4A, 0x0E, 0xAE, 0x0F, 0x1C, 0x0F, 0xFF, 0x0F, 0xFF, 0x00, 0x00},},

	{{0x00, 0x00, 0x00, 0x16, 0x00, 0x2D, 0x00, 0x43, 0x00, 0x71, 0x00, 0x9C, 0x00, 0xC7, 0x00, 0xF3, 0x01, 0x1E, 0x01, 0x4A, 0x01, 0x76, 0x01, 0xA1},
	{0x01, 0xF8, 0x02, 0x50, 0x02, 0xA7, 0x03, 0x04, 0x03, 0x6E, 0x04, 0x2B, 0x04, 0xD2, 0x05, 0x6C, 0x06, 0x73, 0x07, 0x57, 0x08, 0x21, 0x08, 0xDC},
	{0x09, 0x8B, 0x0A, 0x2E, 0x0A, 0xD4, 0x0B, 0x7B, 0x0C, 0x1E, 0x0C, 0x6F, 0x0C, 0xC6, 0x0D, 0x21, 0x0D, 0x85, 0x0E, 0x62, 0x0E, 0x72, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x00, 0x4F, 0x00, 0x77, 0x00, 0x9F, 0x00, 0xC7, 0x00, 0xEF, 0x01, 0x17},
	{0x01, 0x67, 0x01, 0xB8, 0x02, 0x08, 0x02, 0x5E, 0x02, 0xBF, 0x03, 0x6C, 0x04, 0x05, 0x04, 0x91, 0x05, 0x83, 0x06, 0x57, 0x07, 0x12, 0x07, 0xBB},
	{0x08, 0x5A, 0x08, 0xF0, 0x09, 0x85, 0x0A, 0x1C, 0x0A, 0xAF, 0x0A, 0xF7, 0x0B, 0x44, 0x0B, 0x95, 0x0B, 0xED, 0x0C, 0xA8, 0x0C, 0xB6, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x4E, 0x00, 0x96, 0x00, 0xE0, 0x01, 0x29, 0x01, 0x73},
	{0x02, 0x04, 0x02, 0x97, 0x03, 0x2A, 0x03, 0xBE, 0x04, 0x47, 0x05, 0x0F, 0x05, 0xB6, 0x06, 0x5A, 0x07, 0x75, 0x08, 0x6B, 0x09, 0x45, 0x0A, 0x0E},
	{0x0A, 0xC9, 0x0B, 0x7A, 0x0C, 0x2D, 0x0C, 0xE1, 0x0D, 0x93, 0x0D, 0xEC, 0x0E, 0x4A, 0x0E, 0xAE, 0x0F, 0x1C, 0x0F, 0xFF, 0x0F, 0xFF, 0x00, 0x00},},
};

static struct dsi_cmd_desc lp_write_start_cmd[] = {
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		sizeof(addr_value_page), addr_value_page,
		0, NULL}, 0, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		sizeof(addr_enable), addr_enable,
		0, NULL}, 1, 0},
};
static struct dsi_cmd_desc lp_write_end_cmd[] = {
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		sizeof(addr_enable), addr_enable,
		0, NULL}, 0, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		sizeof(addr_value_page), addr_value_page,
		0, NULL}, 1, 0},
};
static struct dsi_cmd_desc lp_gamma_write_cmd[] = {
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		sizeof(addr_gamma_band), addr_gamma_band,
		0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		25, write_buf9x25[0],
		0, NULL}, 0, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		25, write_buf9x25[1],
		0, NULL}, 0, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		23, write_buf9x25[2],
		0, NULL}, 0, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		25, write_buf9x25[3],
		0, NULL}, 0, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		25, write_buf9x25[4],
		0, NULL}, 0, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		23, write_buf9x25[5],
		0, NULL}, 0, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		25, write_buf9x25[6],
		0, NULL}, 0, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		25, write_buf9x25[7],
		0, NULL}, 0, 0},
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		23, write_buf9x25[8],
		0, NULL}, 1, 0},
};

static struct dsi_cmd_desc lp_read_start_cmd[] = {
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		sizeof(addr_value_page), addr_value_page,
		0, NULL}, 1, 0},
};
static struct dsi_cmd_desc lp_read_end_cmd[] = {
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		sizeof(addr_value_page), addr_value_page,
		0, NULL}, 1, 0},
};
static struct dsi_cmd_desc lp_gamma_read_cmd[] = {
	{{0, MIPI_DSI_GENERIC_LONG_WRITE,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		sizeof(addr_gamma_band), addr_gamma_band,
		0, NULL}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		1, addr_gamma[0],
		24, read_array9x24[0]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		1, addr_gamma[1],
		24, read_array9x24[1]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		1, addr_gamma[2],
		22, read_array9x24[2]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		1, addr_gamma[3],
		24, read_array9x24[3]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		1, addr_gamma[4],
		24, read_array9x24[4]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		1, addr_gamma[5],
		22, read_array9x24[5]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		1, addr_gamma[6],
		24, read_array9x24[6]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		1, addr_gamma[7],
		24, read_array9x24[7]}, 1, 0},
	{{0, MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM,
		MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0,
		1, addr_gamma[8],
		22, read_array9x24[8]}, 1, 0},
};
struct drm_gmm_ctx {
	bool read_status;
};
static struct drm_gmm_ctx gmm_ctx = {0};

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_gmm_get_gamma_table(struct dsi_panel *panel, u32 preset)
{
	struct dsi_display *pdisp = NULL;
	int ret = 0;
	int i = 0;

	if (!panel) {
		DSI_DEBUG("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	memset(&gmm_ctx, 0x00, sizeof(gmm_ctx));

	memset(gamma_array10x9x24, 0x00, sizeof(gamma_array10x9x24));
	pdisp = container_of(panel->host, struct dsi_display, host);

	addr_value_page[5] = 0x02;
	ret = drm_cmn_dsi_cmds_transfer(pdisp, lp_read_start_cmd,
		ARRAY_SIZE(lp_read_start_cmd));
	if (ret) {
		DSI_ERR("%s: <RESULT_FAILURE> read_start_cmd ret=%d.\n",
			__func__, ret);
		return ret;
	}

	for (i = 0; i < DRM_GMM_BAND_COUNT_10; i++) {
		memset(read_array9x24, 0x00, sizeof(read_array9x24));
		addr_gamma_band[1] = i + 0x10 * preset;

		ret = drm_cmn_dsi_cmds_transfer(pdisp, lp_gamma_read_cmd,
			ARRAY_SIZE(lp_gamma_read_cmd));
		if (ret) {
			DSI_ERR("%s: <RESULT_FAILURE> failed to read gamma table ret=%d.\n",
				__func__, ret);
			return ret;
		}
		memcpy(&gamma_array10x9x24[i][0][0], read_array9x24, sizeof(read_array9x24));
	}
	addr_value_page[5] = 0x00;
	ret = drm_cmn_dsi_cmds_transfer(pdisp, lp_read_end_cmd,
		ARRAY_SIZE(lp_read_end_cmd));
	if (ret) {
		DSI_ERR("%s: <RESULT_FAILURE> read_end_cmd ret=%d.\n",
			__func__, ret);
		return ret;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_gmm_set_gamma_table(struct dsi_panel *panel, u32 preset)
{
	struct dsi_display *pdisp = NULL;
	int ret = 0;
	int i, j = 0;

	if (!panel) {
		DSI_DEBUG("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pdisp = container_of(panel->host, struct dsi_display, host);

	addr_value_page[5] = 0x02;
	addr_enable[1] = 0x30;
	ret = drm_cmn_dsi_cmds_transfer(pdisp, lp_write_start_cmd,
		ARRAY_SIZE(lp_write_start_cmd));
	if (ret) {
		DSI_ERR("%s: <RESULT_FAILURE> write_start_cmd ret=%d.\n",
			__func__, ret);
		return ret;
	}

	for (i = 0; i < DRM_GMM_BAND_COUNT_10; i++) {
		memset(write_buf9x25, 0x00, sizeof(write_buf9x25));
		for (j = 0; j < DRM_GMM_ADDR_COUNT_9; j++) {
			write_buf9x25[j][0] = addr_gamma[j][0];
			memcpy(&write_buf9x25[j][1], &gamma_array10x9x24[i][j][0], addr_gamma[j][1]);
		}
		addr_gamma_band[1] = i + 0x10 * preset;

		ret = drm_cmn_dsi_cmds_transfer(pdisp, lp_gamma_write_cmd,
			ARRAY_SIZE(lp_gamma_write_cmd));
		if (ret) {
			DSI_ERR("%s: <RESULT_FAILURE> failed to write gamma table ret=%d.\n",
				__func__, ret);
			return ret;
		}
	}

	addr_value_page[5] = 0x00;
	addr_enable[1] = 0x00;
	ret = drm_cmn_dsi_cmds_transfer(pdisp, lp_write_end_cmd,
		ARRAY_SIZE(lp_write_end_cmd));
	if (ret) {
		DSI_ERR("%s: <RESULT_FAILURE> write_end_cmd ret=%d.\n",
			__func__, ret);
		return ret;
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void drm_gmm_calc_gamma_table(void)
{
	int i = 0;

	for (i = 0; i < DRM_GMM_BAND_COUNT_10; i++) {
		drm_gmm_convert_short_gamma_table(i);
		drm_gmm_convert_tone_table();
		drm_gmm_calc_revise();
		drm_gmm_convert_short_write_table();
		drm_gmm_convert_write_table(i);
	}
	return;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void drm_gmm_convert_short_gamma_table(u32 band)
{
	int i, j = 0;

	memset(work_short_array9x12, 0x00, sizeof(work_short_array9x12));

	for (i = 0; i < DRM_GMM_ADDR_COUNT_9; i++) {
		for (j = 0; j < (DRM_GMM_SHORT_PARAM_12); j++) {
			work_short_array9x12[i][j] = (gamma_array10x9x24[band][i][j * 2] << 8) | (gamma_array10x9x24[band][i][j * 2 + 1]);
		}
	}
	return;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void drm_gmm_convert_tone_table(void)
{
	int i = 0;
	int color = 0;
	int idx = 0;

	memset(work_tone_array3x35, 0x00, sizeof(work_tone_array3x35));

	for (i = 0; i < DRM_GMM_TRI_COLOR_3; i++) {
		color = i * 3;
		idx = 0;
		memcpy(&work_tone_array3x35[i][idx], &work_short_array9x12[color][0],
				DRM_GMM_SHORT_PARAM_12 * sizeof(work_short_array9x12[0][0]));
		idx += DRM_GMM_SHORT_PARAM_12;
		memcpy(&work_tone_array3x35[i][idx], &work_short_array9x12[color + 1][0],
				DRM_GMM_SHORT_PARAM_12 * sizeof(work_short_array9x12[0][0]));
		idx += DRM_GMM_SHORT_PARAM_12;
		memcpy(&work_tone_array3x35[i][idx], &work_short_array9x12[color + 2][0],
				(DRM_GMM_SHORT_PARAM_12 - 1) * sizeof(work_short_array9x12[0][0]));
	}
	return;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void drm_gmm_calc_revise(void)
{
	int work_value;
	int i, j = 0;

	for (i = 0; i < DRM_GMM_TRI_COLOR_3; i++) {
		for (j = 4; j < DRM_GMM_COLOR_TONE_35; j++) {
			work_value = work_tone_array3x35[i][j] * DRM_GMM_CALC_COEFFICIENT;
			work_value += revise_gamma_x100[i][j];
			if (work_value < DRM_GMM_MIN_VALUE_0x0) {
				work_value = DRM_GMM_MIN_VALUE_0x0;
			}
			if (j == 4) {
				work_tone_array3x35[i][0] = DRM_GMM_MIN_VALUE_0x0;
				work_tone_array3x35[i][1] = work_value * 2 / 10 / DRM_GMM_CALC_COEFFICIENT;
				work_tone_array3x35[i][2] = work_value * 4 / 10 / DRM_GMM_CALC_COEFFICIENT;
				work_tone_array3x35[i][3] = work_value * 6 / 10 / DRM_GMM_CALC_COEFFICIENT;
			}
			work_tone_array3x35[i][j] = work_value / DRM_GMM_CALC_COEFFICIENT;
		}
	}
	return;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void drm_gmm_convert_short_write_table(void)
{
	int i = 0;
	int color = 0;
	int idx = 0;


	for (i = 0; i < DRM_GMM_TRI_COLOR_3; i++) {
		color = i * 3;
		idx = 0;
		memcpy(&work_short_array9x12[color][0], &work_tone_array3x35[i][idx],
				DRM_GMM_SHORT_PARAM_12 * sizeof(work_short_array9x12[0][0]));
		idx += DRM_GMM_SHORT_PARAM_12;
		memcpy(&work_short_array9x12[color + 1][0], &work_tone_array3x35[i][idx],
				DRM_GMM_SHORT_PARAM_12 * sizeof(work_short_array9x12[0][0]));
		idx += DRM_GMM_SHORT_PARAM_12;
		memcpy(&work_short_array9x12[color + 2][0], &work_tone_array3x35[i][idx],
				(DRM_GMM_SHORT_PARAM_12 - 1) * sizeof(work_short_array9x12[0][0]));
	}
	return;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void drm_gmm_convert_write_table(u32 band)
{
	int i, j = 0;

	for (i = 0; i < DRM_GMM_ADDR_COUNT_9; i++) {
		for (j = 0; j < (DRM_GMM_SHORT_PARAM_12); j++) {
			if (work_short_array9x12[i][j] > DRM_GMM_MAX_VALUE_0xFFF) {
				work_short_array9x12[i][j] = DRM_GMM_MAX_VALUE_0xFFF;
			}
			gamma_array10x9x24[band][i][j * 2] = (work_short_array9x12[i][j] & 0xFF00) >> 8;
			gamma_array10x9x24[band][i][j * 2 + 1] = work_short_array9x12[i][j] & 0x00FF;
		}
	}
	return;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void drm_gmm_default_gamma_table(void)
{
	memcpy(gamma_array10x9x24, gamma_default_array10x9x24, sizeof(gamma_default_array10x9x24));
	return;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_gmm_ctrl_gamma_table(struct dsi_panel *panel)
{
	int ret = 0;

	if (!panel) {
		DSI_DEBUG("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);
	ret = drm_gmm_get_gamma_table(panel, 0x00);
	if (ret) {
		gmm_ctx.read_status = false;
		DSI_ERR("[%s] failed to get gamma table, ret=%d\n",
		       panel->name, ret);
	} else {
		gmm_ctx.read_status = true;
		drm_gmm_calc_gamma_table();
		ret = drm_gmm_set_gamma_table(panel, 0x02);
		if (ret) {
			DSI_ERR("[%s] failed to set gamma table, ret=%d\n",
			       panel->name, ret);
		}
	}
	mutex_unlock(&panel->panel_lock);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_gmm_active_gamma_table(struct dsi_panel *panel)
{
	int ret = 0;

	if (!panel) {
		DSI_DEBUG("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (!gmm_ctx.read_status) {
		ret = drm_gmm_get_gamma_table(panel, 0x00);
		if (ret) {
			gmm_ctx.read_status = false;
			DSI_WARN("[%s] write default gmm table, read_status=\%dn",
				panel->name, gmm_ctx.read_status);
			drm_gmm_default_gamma_table();
		} else {
			gmm_ctx.read_status = true;
			drm_gmm_calc_gamma_table();
		}
	}

	ret = drm_gmm_set_gamma_table(panel, 0x02);
	if (ret) {
		DSI_ERR("[%s] failed to set gamma table, ret=%d\n",
		       panel->name, ret);
	}
	return ret;
}