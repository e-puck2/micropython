/*
 * e-puck2 image processing: color/grey/difference blob detection on camera images.
 *
 * Blob detection (connected components labeling, 8-connectivity) works row by row on runs of consecutive
 * selected pixels, merging the runs that touch a run of the previous row (union-find). Only the runs of the
 * previous and current rows and the labels in use are kept, so the memory needed depends only on the image width
 * (about 3 KB for 80 pixels, 6 KB for 160 pixels), not on the number of pixels or blobs.
 * When a group has no runs in the current row it is complete: it is compared with the biggest blobs found so far
 * and its label is reused.
 */

#include <string.h>
#include "epuck2_image.h"

#define LABEL_NONE 0xFFFF

typedef struct {
    uint16_t start;
    uint16_t end; // Included.
    uint16_t label;
} img_run_t;

typedef struct {
    uint32_t pixels;
    uint32_t sum_x;
    uint32_t sum_y;
    uint16_t parent;
    uint16_t x_min, x_max, y_min, y_max;
    uint16_t alive_row; // Row (+1) in which the label was last seen as root of a run.
    uint8_t used;
} img_label_t;

static inline void img_rgb565(const uint8_t *p, uint32_t *r, uint32_t *g, uint32_t *b) {
    uint32_t v = ((uint32_t)p[0] << 8) | p[1];
    uint32_t r5 = v >> 11, g6 = (v >> 5) & 0x3F, b5 = v & 0x1F;
    // Expand to 8 bits.
    *r = (r5 << 3) | (r5 >> 2);
    *g = (g6 << 2) | (g6 >> 4);
    *b = (b5 << 3) | (b5 >> 2);
}

uint8_t img_luminance(const uint8_t *img, const img_fmt_t *fmt, uint32_t x, uint32_t y) {
    const uint8_t *p = &img[(y * fmt->width + x) * fmt->bpp];
    if (fmt->bpp == 1) {
        return p[0];
    }
    uint32_t r, g, b;
    img_rgb565(p, &r, &g, &b);
    return (r * 77 + g * 150 + b * 29) >> 8; // ITU-R BT.601.
}

void img_rgb_to_hsv(uint32_t r, uint32_t g, uint32_t b, uint16_t *h, uint8_t *s, uint8_t *v) {
    uint32_t max = r > g ? (r > b ? r : b) : (g > b ? g : b);
    uint32_t min = r < g ? (r < b ? r : b) : (g < b ? g : b);
    int32_t delta = max - min;
    int32_t hue = 0;
    if (delta > 0) {
        if (max == r) {
            hue = 60 * ((int32_t)g - (int32_t)b) / delta;
        } else if (max == g) {
            hue = 120 + 60 * ((int32_t)b - (int32_t)r) / delta;
        } else {
            hue = 240 + 60 * ((int32_t)r - (int32_t)g) / delta;
        }
        if (hue < 0) {
            hue += 360;
        }
    }
    *h = hue;
    *s = max ? (delta * 100 + max / 2) / max : 0;
    *v = (max * 100 + 127) / 255;
}

void img_get_color(const uint8_t *img, const img_fmt_t *fmt, const img_roi_t *roi, uint16_t *h, uint8_t *s, uint8_t *v) {
    uint32_t sr = 0, sg = 0, sb = 0, r, g, b;
    uint32_t n = (uint32_t)roi->w * roi->h;
    for (uint32_t y = roi->y; y < (uint32_t)roi->y + roi->h; y++) {
        const uint8_t *p = &img[(y * fmt->width + roi->x) * 2];
        for (uint32_t x = 0; x < roi->w; x++, p += 2) {
            img_rgb565(p, &r, &g, &b);
            sr += r;
            sg += g;
            sb += b;
        }
    }
    img_rgb_to_hsv((sr + n / 2) / n, (sg + n / 2) / n, (sb + n / 2) / n, h, s, v);
}

static bool img_selected(const img_sel_t *sel, uint32_t x, uint32_t y) {
    switch (sel->type) {
        case IMG_SEL_COLOR: {
            uint32_t r, g, b;
            uint16_t hue;
            uint8_t sat, val;
            img_rgb565(&sel->img[(y * sel->fmt.width + x) * 2], &r, &g, &b);
            img_rgb_to_hsv(r, g, b, &hue, &sat, &val);
            if (sat < sel->sat_min || sat > sel->sat_max || val < sel->val_min || val > sel->val_max) {
                return false;
            }
            if (sel->hue_min <= sel->hue_max) {
                return hue >= sel->hue_min && hue <= sel->hue_max;
            }
            return hue >= sel->hue_min || hue <= sel->hue_max; // Range across 0 (e.g. red 340..20).
        }
        case IMG_SEL_GREY: {
            uint8_t l = img_luminance(sel->img, &sel->fmt, x, y);
            return l >= sel->grey_min && l <= sel->grey_max;
        }
        default: {
            int32_t d = (int32_t)img_luminance(sel->img, &sel->fmt, x, y) - img_luminance(sel->prev, &sel->fmt, x, y);
            if (d < 0) {
                d = -d;
            }
            return d >= sel->diff_threshold;
        }
    }
}

static inline uint32_t img_max_runs(uint16_t roi_width) {
    return roi_width / 2 + 1;
}

size_t img_blobs_work_size(uint16_t roi_width) {
    uint32_t max_runs = img_max_runs(roi_width);
    uint32_t n_labels = 2 * max_runs;
    return n_labels * sizeof(img_label_t) + 2 * max_runs * sizeof(img_run_t) + 2 * n_labels * sizeof(uint16_t);
}

static uint16_t img_find_root(img_label_t *labels, uint16_t a) {
    while (labels[a].parent != a) {
        labels[a].parent = labels[labels[a].parent].parent; // Path halving.
        a = labels[a].parent;
    }
    return a;
}

// Insert the label in the list of the biggest blobs (sorted by size, biggest first).
static int img_blob_insert(img_blob_t *blobs, int n, int max_blobs, const img_label_t *l) {
    if (n == max_blobs && l->pixels <= blobs[n - 1].pixels) {
        return n;
    }
    int i;
    if (n < max_blobs) {
        i = n++;
    } else {
        i = max_blobs - 1; // Replace the smallest one.
    }
    while (i > 0 && blobs[i - 1].pixels < l->pixels) {
        blobs[i] = blobs[i - 1];
        i--;
    }
    blobs[i].pixels = l->pixels;
    blobs[i].cx = (l->sum_x + l->pixels / 2) / l->pixels;
    blobs[i].cy = (l->sum_y + l->pixels / 2) / l->pixels;
    blobs[i].x_min = l->x_min;
    blobs[i].y_min = l->y_min;
    blobs[i].x_max = l->x_max;
    blobs[i].y_max = l->y_max;
    return n;
}

int img_find_blobs(const img_sel_t *sel, const img_roi_t *roi, uint32_t min_pixels,
    img_blob_t *blobs, int max_blobs, void *work) {
    uint32_t max_runs = img_max_runs(roi->w);
    uint32_t n_labels = 2 * max_runs;
    img_label_t *labels = (img_label_t *)work;
    img_run_t *prev = (img_run_t *)&labels[n_labels];
    img_run_t *cur = &prev[max_runs];
    uint16_t *free_stack = (uint16_t *)&cur[max_runs];
    uint16_t *merged = &free_stack[n_labels];
    uint32_t n_free = n_labels, n_prev = 0, n_cur, n_merged;
    int n_blobs = 0;

    for (uint32_t i = 0; i < n_labels; i++) {
        labels[i].used = 0;
        free_stack[i] = n_labels - 1 - i;
    }

    uint32_t x_end = (uint32_t)roi->x + roi->w;
    for (uint32_t y = roi->y; y < (uint32_t)roi->y + roi->h; y++) {
        // Runs of selected pixels of the current row.
        n_cur = 0;
        uint32_t x = roi->x;
        while (x < x_end) {
            if (!img_selected(sel, x, y)) {
                x++;
                continue;
            }
            uint32_t start = x;
            while (x < x_end && img_selected(sel, x, y)) {
                x++;
            }
            cur[n_cur].start = start;
            cur[n_cur].end = x - 1;
            cur[n_cur].label = LABEL_NONE;
            n_cur++;
        }

        // Connect each run to the runs of the previous row that touch it (also diagonally).
        n_merged = 0;
        uint32_t p = 0;
        for (uint32_t c = 0; c < n_cur; c++) {
            while (p < n_prev && prev[p].end + 1 < cur[c].start) {
                p++;
            }
            uint16_t label = LABEL_NONE;
            for (uint32_t q = p; q < n_prev && prev[q].start <= cur[c].end + 1; q++) {
                uint16_t r = img_find_root(labels, prev[q].label);
                if (label == LABEL_NONE) {
                    label = r;
                } else if (r != label) {
                    // The run joins two groups: merge r into label.
                    img_label_t *a = &labels[label], *b = &labels[r];
                    b->parent = label;
                    a->pixels += b->pixels;
                    a->sum_x += b->sum_x;
                    a->sum_y += b->sum_y;
                    if (b->x_min < a->x_min) a->x_min = b->x_min;
                    if (b->x_max > a->x_max) a->x_max = b->x_max;
                    if (b->y_min < a->y_min) a->y_min = b->y_min;
                    if (b->y_max > a->y_max) a->y_max = b->y_max;
                    merged[n_merged++] = r;
                }
            }
            if (label == LABEL_NONE) {
                if (n_free == 0) {
                    continue; // Can't happen: at most 2 * max_runs labels are in use.
                }
                label = free_stack[--n_free];
                img_label_t *l = &labels[label];
                l->used = 1;
                l->parent = label;
                l->pixels = l->sum_x = l->sum_y = 0;
                l->x_min = cur[c].start;
                l->x_max = cur[c].end;
                l->y_min = l->y_max = y;
                l->alive_row = 0;
            }
            img_label_t *l = &labels[label];
            uint32_t n = cur[c].end - cur[c].start + 1;
            l->pixels += n;
            l->sum_x += (cur[c].start + cur[c].end) * n / 2;
            l->sum_y += y * n;
            if (cur[c].start < l->x_min) l->x_min = cur[c].start;
            if (cur[c].end > l->x_max) l->x_max = cur[c].end;
            if (y > l->y_max) l->y_max = y;
            cur[c].label = label;
        }

        // Groups present in this row.
        for (uint32_t c = 0; c < n_cur; c++) {
            if (cur[c].label == LABEL_NONE) {
                continue;
            }
            cur[c].label = img_find_root(labels, cur[c].label);
            labels[cur[c].label].alive_row = y + 1;
        }
        // Groups of the previous row not present in this row are complete.
        for (uint32_t q = 0; q < n_prev; q++) {
            uint16_t r = img_find_root(labels, prev[q].label);
            if (labels[r].used && labels[r].alive_row != y + 1) {
                if (labels[r].pixels >= min_pixels) {
                    n_blobs = img_blob_insert(blobs, n_blobs, max_blobs, &labels[r]);
                }
                labels[r].used = 0;
                free_stack[n_free++] = r;
            }
        }
        // The labels merged into others aren't referenced anymore.
        for (uint32_t m = 0; m < n_merged; m++) {
            labels[merged[m]].used = 0;
            free_stack[n_free++] = merged[m];
        }

        img_run_t *t = prev;
        prev = cur;
        cur = t;
        n_prev = 0;
        for (uint32_t c = 0; c < n_cur; c++) {
            if (prev[c].label != LABEL_NONE) {
                prev[n_prev++] = prev[c];
            }
        }
    }

    // Groups touching the last row.
    for (uint32_t q = 0; q < n_prev; q++) {
        uint16_t r = img_find_root(labels, prev[q].label);
        if (labels[r].used) {
            if (labels[r].pixels >= min_pixels) {
                n_blobs = img_blob_insert(blobs, n_blobs, max_blobs, &labels[r]);
            }
            labels[r].used = 0;
        }
    }
    return n_blobs;
}
