/*
 * e-puck2 image processing: color/grey/difference blob detection on camera images.
 * Plain C (no MicroPython dependencies), used by the epuck2 module (modepuck2.c).
 *
 * Images are stored row by row: greyscale 1 byte per pixel, RGB565 2 bytes per pixel big endian
 * (RRRRRGGG GGGBBBBB), as returned by epuck2.camera_get_image().
 */

#ifndef EPUCK2_IMAGE_H
#define EPUCK2_IMAGE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct {
    uint16_t width;
    uint16_t height;
    uint8_t bpp; // Bytes per pixel: 1 greyscale, 2 RGB565.
} img_fmt_t;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
} img_roi_t;

typedef struct {
    uint32_t pixels;
    uint16_t cx; // Centroid.
    uint16_t cy;
    uint16_t x_min; // Bounding box (included).
    uint16_t y_min;
    uint16_t x_max;
    uint16_t y_max;
} img_blob_t;

typedef enum {
    IMG_SEL_COLOR, // HSV range (RGB565 only).
    IMG_SEL_GREY,  // Grey level range (luminance for RGB565).
    IMG_SEL_DIFF,  // Luminance difference between two images.
} img_sel_type_t;

// Pixel selection.
typedef struct {
    img_sel_type_t type;
    const uint8_t *img;
    const uint8_t *prev; // IMG_SEL_DIFF only.
    img_fmt_t fmt;
    uint16_t hue_min, hue_max; // 0..359, hue_min > hue_max means the range wraps around 0 (e.g. 340..20 for red).
    uint8_t sat_min, sat_max;  // 0..100.
    uint8_t val_min, val_max;  // 0..100.
    uint8_t grey_min, grey_max; // 0..255.
    uint8_t diff_threshold; // 1..255.
} img_sel_t;

// Luminance (0..255) of the pixel (x, y).
uint8_t img_luminance(const uint8_t *img, const img_fmt_t *fmt, uint32_t x, uint32_t y);

// Convert an RGB color (0..255) to HSV: hue 0..359, saturation and value 0..100.
void img_rgb_to_hsv(uint32_t r, uint32_t g, uint32_t b, uint16_t *h, uint8_t *s, uint8_t *v);

// Mean color of the area "roi" of an RGB565 image, as HSV (hue of the mean RGB color).
void img_get_color(const uint8_t *img, const img_fmt_t *fmt, const img_roi_t *roi, uint16_t *h, uint8_t *s, uint8_t *v);

// Size of the work memory needed by img_find_blobs() for an area "roi_width" pixels wide.
size_t img_blobs_work_size(uint16_t roi_width);

// Find the groups of selected pixels touching each other (8-connectivity) inside "roi".
// Fill "blobs" with the biggest ones (at least "min_pixels" pixels, at most "max_blobs"), sorted by size
// (biggest first). "work" must be img_blobs_work_size(roi->w) bytes, aligned to 4 bytes.
// Return the number of blobs found.
int img_find_blobs(const img_sel_t *sel, const img_roi_t *roi, uint32_t min_pixels,
    img_blob_t *blobs, int max_blobs, void *work);

#endif // EPUCK2_IMAGE_H
