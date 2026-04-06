#ifndef VISION_PRODUCER_H
#define VISION_PRODUCER_H

/*
 * Defines ROI map, frame structures, occupancy events,
 * and interfaces between frame ingestion and the Parking Manager.
 */

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>

#define VP_MAX_SPOTS          128      /* max parking stalls supported   */
#define VP_MAX_ROI_VERTICES   8       /* max polygon vertices per ROI   */
#define VP_FRAME_QUEUE_DEPTH  8       /* ring-buffer depth for frames   */
#define VP_CHANNEL_NAME       "/dev/parking/vp_events"  /* QNX channel  */

#define VP_DIFF_THRESHOLD     40      /* per-pixel absolute diff cutoff */
#define VP_OCCUPANCY_RATIO    0.18    /* fraction of ROI pixels changed */
#define VP_DEBOUNCE_FRAMES    8       /* consecutive frames to confirm  */
#define VP_SHADOW_LUMA_MIN    40      /* luma floor to suppress shadows */
#define VP_SHADOW_LUMA_MAX    100     /* luma ceiling for shadow band   */

/* Raw 8-bit grayscale frame (caller owns the pixel buffer) */
typedef struct {
    uint8_t  *pixels;       /* row-major, width * height bytes          */
    uint32_t  width;
    uint32_t  height;
    uint64_t  timestamp_ns; /* monotonic capture time (CLOCK_MONOTONIC) */
    uint32_t  frame_id;
} vp_frame_t;

typedef struct {
    uint32_t x_min, y_min;
    uint32_t x_max, y_max;
} vp_bbox_t;

typedef struct {
    uint32_t spot_id;                       /* unique stall ID [0..N)  */
    char     label[16];                     /* human label e.g. "A-07" */
    uint32_t n_vertices;
    struct { uint32_t x, y; } vertices[VP_MAX_ROI_VERTICES];
    vp_bbox_t bbox;
} vp_roi_t;

/* ROI Map – collection of all stalls */

typedef struct {
    uint32_t  n_spots;
    vp_roi_t  spots[VP_MAX_SPOTS];
} vp_roi_map_t;

typedef enum {
    VP_SPOT_UNKNOWN = 0,
    VP_SPOT_FREE,
    VP_SPOT_OCCUPIED
} vp_spot_state_t;

typedef struct {
    vp_spot_state_t state;
    uint32_t        debounce_count;
    vp_spot_state_t candidate;
    uint64_t        last_change_ns;
    uint32_t        change_count;
} vp_spot_ctx_t;

/* Occupancy event – sent to the Parking Manager */

typedef struct {
    uint32_t         spot_id;
    char             label[16];
    vp_spot_state_t  old_state;
    vp_spot_state_t  new_state;
    uint64_t         detected_ns;   /* when Vision Producer saw the change */
    uint32_t         frame_id;
    float            change_ratio;  /* fraction of ROI pixels that changed */
} vp_event_t;

typedef struct {
    /* Configuration */
    vp_roi_map_t    roi_map;
    uint32_t        diff_threshold;
    float           occupancy_ratio;
    uint32_t        debounce_frames;

    /* Per-spot tracking */
    vp_spot_ctx_t   spot_ctx[VP_MAX_SPOTS];

    /* Background reference frame (grayscale) */
    uint8_t        *reference_frame;
    uint32_t        ref_width;
    uint32_t        ref_height;
    bool            ref_valid;

    /* Frame ring buffer (producer/consumer between ingestion & analysis) */
    vp_frame_t      frame_queue[VP_FRAME_QUEUE_DEPTH];
    uint32_t        fq_head;          /* next slot to write              */
    uint32_t        fq_tail;          /* next slot to read               */
    pthread_mutex_t fq_mutex;
    pthread_cond_t  fq_not_empty;
    pthread_cond_t  fq_not_full;

    /* IPC: connection to Parking Manager */
    int             pm_coid;          /* QNX ConnectAttach result        */

    /* Control */
    volatile bool   running;
    pthread_t       ingest_thread;
    pthread_t       analysis_thread;

    /* Statistics */
    uint64_t        frames_processed;
    uint64_t        events_sent;
    uint64_t        total_latency_ns; /* sum for average computation     */
    uint64_t        max_latency_ns;
} vp_ctx_t;

int  vp_init(vp_ctx_t *ctx, const vp_roi_map_t *roi_map, int pm_coid);
int  vp_start(vp_ctx_t *ctx);
void vp_stop(vp_ctx_t *ctx);
void vp_destroy(vp_ctx_t *ctx);
int  vp_enqueue_frame(vp_ctx_t *ctx, vp_frame_t *frame);
int  vp_roi_map_load_json(vp_roi_map_t *map, const char *path);
void vp_roi_map_add_rect(vp_roi_map_t *map, uint32_t id, const char *label,
                         uint32_t x, uint32_t y, uint32_t w, uint32_t h);
void vp_roi_compute_bbox(vp_roi_t *roi);
bool vp_point_in_roi(const vp_roi_t *roi, uint32_t px, uint32_t py);
float vp_compute_change_ratio(const vp_ctx_t    *ctx,
                               const vp_roi_t    *roi,
                               const uint8_t     *frame,
                               uint32_t           width,
                               uint32_t           height);

vp_spot_state_t vp_classify_spot(float change_ratio, float threshold);
bool vp_shadow_pixel(uint8_t luma);
int  vp_load_reference(vp_ctx_t *ctx, const char *path,
                        uint32_t width, uint32_t height);
uint64_t vp_now_ns(void);
void     vp_log(const char *fmt, ...);

#endif
