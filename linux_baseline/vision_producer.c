#define _POSIX_C_SOURCE 200809L

#include "vision_producer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>

#ifdef VPLINUX
#include <sys/socket.h>
#ifdef __APPLE__
#define VP_SEND_FLAGS 0
#else
#define VP_SEND_FLAGS MSG_NOSIGNAL
#endif
#else
#include <sys/neutrino.h>
#include <sys/iomsg.h>
#endif

#ifdef VPLINUX
static int vp_send_exact_fd(int fd, const void *buf, size_t n)
{
    size_t done = 0;
    const uint8_t *p = (const uint8_t *)buf;
    while (done < n) {
        ssize_t s = send(fd, p + done, n - done, VP_SEND_FLAGS);
        if (s < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        done += (size_t)s;
    }
    return 0;
}

static int vp_recv_exact_fd(int fd, void *buf, size_t n)
{
    size_t done = 0;
    uint8_t *p = (uint8_t *)buf;
    while (done < n) {
        ssize_t r = recv(fd, p + done, n - done, 0);
        if (r == 0) return -1;
        if (r < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        done += (size_t)r;
    }
    return 0;
}
#endif

uint64_t vp_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

void vp_log(const char *fmt, ...)
{
    va_list ap;
    char buf[256];
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    uint64_t ns = vp_now_ns();
    fprintf(stderr, "[VP %llu.%06llu] %s\n",
            (unsigned long long)(ns / 1000000000ULL),
            (unsigned long long)((ns % 1000000000ULL) / 1000ULL),
            buf);
}

void vp_roi_compute_bbox(vp_roi_t *roi)
{
    assert(roi->n_vertices > 0);

    roi->bbox.x_min = roi->vertices[0].x;
    roi->bbox.x_max = roi->vertices[0].x;
    roi->bbox.y_min = roi->vertices[0].y;
    roi->bbox.y_max = roi->vertices[0].y;

    for (uint32_t i = 1; i < roi->n_vertices; i++) {
        uint32_t x = roi->vertices[i].x;
        uint32_t y = roi->vertices[i].y;
        if (x < roi->bbox.x_min) roi->bbox.x_min = x;
        if (x > roi->bbox.x_max) roi->bbox.x_max = x;
        if (y < roi->bbox.y_min) roi->bbox.y_min = y;
        if (y > roi->bbox.y_max) roi->bbox.y_max = y;
    }
}

bool vp_point_in_roi(const vp_roi_t *roi, uint32_t px, uint32_t py)
{
    if (px < roi->bbox.x_min || px > roi->bbox.x_max ||
        py < roi->bbox.y_min || py > roi->bbox.y_max)
        return false;

    uint32_t n = roi->n_vertices;
    bool inside = false;

    for (uint32_t i = 0, j = n - 1; i < n; j = i++) {
        int32_t xi = (int32_t)roi->vertices[i].x;
        int32_t yi = (int32_t)roi->vertices[i].y;
        int32_t xj = (int32_t)roi->vertices[j].x;
        int32_t yj = (int32_t)roi->vertices[j].y;
        int32_t x  = (int32_t)px;
        int32_t y  = (int32_t)py;

        bool intersect = ((yi > y) != (yj > y)) &&
                         (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

void vp_roi_map_add_rect(vp_roi_map_t *map,
                         uint32_t id, const char *label,
                         uint32_t x, uint32_t y,
                         uint32_t w, uint32_t h)
{
    assert(map->n_spots < VP_MAX_SPOTS);

    vp_roi_t *roi = &map->spots[map->n_spots++];
    memset(roi, 0, sizeof(*roi));
    roi->spot_id    = id;
    roi->n_vertices = 4;
    snprintf(roi->label, sizeof(roi->label), "%s", label ? label : "");

    roi->vertices[0].x = x;     roi->vertices[0].y = y;
    roi->vertices[1].x = x + w; roi->vertices[1].y = y;
    roi->vertices[2].x = x + w; roi->vertices[2].y = y + h;
    roi->vertices[3].x = x;     roi->vertices[3].y = y + h;

    vp_roi_compute_bbox(roi);
}

int vp_roi_map_load_json(vp_roi_map_t *map, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        vp_log("ERROR: cannot open ROI map file: %s", path);
        return -1;
    }

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    rewind(f);
    char *buf = malloc((size_t)sz + 1);
    if (!buf) { fclose(f); return -1; }
    fread(buf, 1, (size_t)sz, f);
    buf[sz] = '\0';
    fclose(f);

    map->n_spots = 0;

    char *p = buf;
    while ((p = strstr(p, "\"id\"")) != NULL) {
        if (map->n_spots >= VP_MAX_SPOTS) break;

        vp_roi_t *roi = &map->spots[map->n_spots];
        memset(roi, 0, sizeof(*roi));

        p += 4;
        while (*p && (*p == ':' || *p == ' ')) p++;
        roi->spot_id = (uint32_t)strtoul(p, &p, 10);

        char *lp = strstr(p, "\"label\"");
        if (!lp) break;
        lp = strchr(lp + 7, '"');
        if (!lp) break;
        lp++;
        uint32_t li = 0;
        while (*lp && *lp != '"' && li < sizeof(roi->label) - 1)
            roi->label[li++] = *lp++;
        roi->label[li] = '\0';

        char *vp_str = strstr(p, "\"vertices\"");
        if (!vp_str) break;
        vp_str = strchr(vp_str, '[');
        if (!vp_str) break;
        vp_str++;

        roi->n_vertices = 0;
        while (roi->n_vertices < VP_MAX_ROI_VERTICES) {
            char *bracket = strchr(vp_str, '[');
            if (!bracket) break;
            bracket++;
            uint32_t x = (uint32_t)strtoul(bracket, &bracket, 10);
            while (*bracket && (*bracket == ',' || *bracket == ' ')) bracket++;
            uint32_t y = (uint32_t)strtoul(bracket, &bracket, 10);
            roi->vertices[roi->n_vertices].x = x;
            roi->vertices[roi->n_vertices].y = y;
            roi->n_vertices++;

            char *end = strchr(bracket, ']');
            if (!end) break;
            vp_str = end + 1;

            char *nxt = vp_str;
            while (*nxt == ',' || *nxt == ' ' || *nxt == '\n' || *nxt == '\r') nxt++;
            if (*nxt == ']') break;
        }

        if (roi->n_vertices >= 3) {
            vp_roi_compute_bbox(roi);
            map->n_spots++;
        }
        p = lp;
    }

    free(buf);
    vp_log("ROI map loaded: %u spots from %s", map->n_spots, path);
    return (map->n_spots > 0) ? 0 : -1;
}

int vp_load_reference(vp_ctx_t *ctx, const char *path,
                      uint32_t width, uint32_t height)
{
    size_t nbytes = (size_t)width * (size_t)height;

    FILE *f = fopen(path, "rb");
    if (!f) {
        vp_log("ERROR: cannot open reference file: %s (%s)",
               path, strerror(errno));
        return -1;
    }

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    rewind(f);

    if ((size_t)sz != nbytes) {
        vp_log("ERROR: reference file size mismatch: expected %zu bytes (%ux%u) but got %ld",
               nbytes, width, height, sz);
        fclose(f);
        return -1;
    }

    free(ctx->reference_frame);
    ctx->reference_frame = (uint8_t *)malloc(nbytes);
    if (!ctx->reference_frame) {
        vp_log("ERROR: cannot allocate reference frame (%zu bytes)", nbytes);
        fclose(f);
        return -1;
    }

    size_t nread = fread(ctx->reference_frame, 1, nbytes, f);
    fclose(f);

    if (nread != nbytes) {
        vp_log("ERROR: short read on reference file: %zu / %zu", nread, nbytes);
        free(ctx->reference_frame);
        ctx->reference_frame = NULL;
        return -1;
    }

    ctx->ref_width  = width;
    ctx->ref_height = height;
    ctx->ref_valid  = true;

    for (uint32_t i = 0; i < ctx->roi_map.n_spots; i++) {
        ctx->spot_ctx[i].state          = VP_SPOT_UNKNOWN;
        ctx->spot_ctx[i].candidate      = VP_SPOT_UNKNOWN;
        ctx->spot_ctx[i].debounce_count = 0;
    }

    vp_log("reference frame loaded from %s (%ux%u, %zu bytes)",
           path, width, height, nbytes);
    return 0;
}

bool vp_shadow_pixel(uint8_t luma)
{
    return (luma >= VP_SHADOW_LUMA_MIN && luma <= VP_SHADOW_LUMA_MAX);
}

float vp_compute_change_ratio(const vp_ctx_t *ctx,
                              const vp_roi_t *roi,
                              const uint8_t  *frame,
                              uint32_t        width,
                              uint32_t        height)
{
    if (!ctx->ref_valid || !ctx->reference_frame) return 0.0f;
    if (ctx->ref_width != width || ctx->ref_height != height) return 0.0f;

    uint32_t total   = 0;
    uint32_t changed = 0;

    uint32_t x0 = roi->bbox.x_min;
    uint32_t x1 = (roi->bbox.x_max < width  - 1) ? roi->bbox.x_max : width  - 1;
    uint32_t y0 = roi->bbox.y_min;
    uint32_t y1 = (roi->bbox.y_max < height - 1) ? roi->bbox.y_max : height - 1;

    for (uint32_t y = y0; y <= y1; y++) {
        for (uint32_t x = x0; x <= x1; x++) {
            if (!vp_point_in_roi(roi, x, y)) continue;

            uint32_t idx  = y * width + x;
            uint8_t  f    = frame[idx];
            uint8_t  r    = ctx->reference_frame[idx];
            int16_t  diff = (int16_t)f - (int16_t)r;
            if (diff < 0) diff = -diff;

            total++;
            if ((uint8_t)diff > ctx->diff_threshold && !vp_shadow_pixel(f))
                changed++;
        }
    }

    if (total == 0) return 0.0f;
    return (float)changed / (float)total;
}

vp_spot_state_t vp_classify_spot(float change_ratio, float threshold)
{
    return (change_ratio >= threshold) ? VP_SPOT_OCCUPIED : VP_SPOT_FREE;
}

static int vp_send_event(vp_ctx_t *ctx, const vp_event_t *evt)
{
    uint32_t reply = 0;
#ifdef VPLINUX
    ssize_t sent = send(ctx->pm_coid, evt, sizeof(*evt), VP_SEND_FLAGS);
    if (sent != (ssize_t)sizeof(*evt)) {
        vp_log("ERROR: send(event) failed: %s", strerror(errno));
        return -1;
    }

    ssize_t got = recv(ctx->pm_coid, &reply, sizeof(reply), 0);
    if (got != (ssize_t)sizeof(reply)) {
        vp_log("ERROR: recv(reply) failed: %s", (got < 0) ? strerror(errno) : "short reply");
        return -1;
    }
#else
    int rc = MsgSend(ctx->pm_coid, evt, sizeof(*evt), &reply, sizeof(reply));
    if (rc == -1) {
        vp_log("ERROR: MsgSend failed: %s", strerror(errno));
        return -1;
    }
#endif
    return 0;
}

static bool fq_full(const vp_ctx_t *ctx)
{
    return ((ctx->fq_head + 1) % VP_FRAME_QUEUE_DEPTH) == ctx->fq_tail;
}

static bool fq_empty(const vp_ctx_t *ctx)
{
    return ctx->fq_head == ctx->fq_tail;
}

int vp_enqueue_frame(vp_ctx_t *ctx, vp_frame_t *frame)
{
    pthread_mutex_lock(&ctx->fq_mutex);
    while (fq_full(ctx) && ctx->running)
        pthread_cond_wait(&ctx->fq_not_full, &ctx->fq_mutex);

    if (!ctx->running) {
        pthread_mutex_unlock(&ctx->fq_mutex);
        return -1;
    }

    ctx->frame_queue[ctx->fq_head] = *frame;
    ctx->fq_head = (ctx->fq_head + 1) % VP_FRAME_QUEUE_DEPTH;

    pthread_cond_signal(&ctx->fq_not_empty);
    pthread_mutex_unlock(&ctx->fq_mutex);
    return 0;
}

static int fq_dequeue(vp_ctx_t *ctx, vp_frame_t *out)
{
    pthread_mutex_lock(&ctx->fq_mutex);
    while (fq_empty(ctx) && ctx->running)
        pthread_cond_wait(&ctx->fq_not_empty, &ctx->fq_mutex);

    if (fq_empty(ctx)) {
        pthread_mutex_unlock(&ctx->fq_mutex);
        return -1;
    }

    *out = ctx->frame_queue[ctx->fq_tail];
    ctx->fq_tail = (ctx->fq_tail + 1) % VP_FRAME_QUEUE_DEPTH;

    pthread_cond_signal(&ctx->fq_not_full);
    pthread_mutex_unlock(&ctx->fq_mutex);
    return 0;
}

static void *analysis_thread_fn(void *arg)
{
    vp_ctx_t *ctx = (vp_ctx_t *)arg;
    vp_log("analysis_thread: started");

    while (ctx->running) {
        vp_frame_t frame;
        if (fq_dequeue(ctx, &frame) != 0) break;

        uint64_t t_start = vp_now_ns();

        if (!ctx->ref_valid) {
            size_t nbytes = (size_t)frame.width * (size_t)frame.height;
            if (!ctx->reference_frame) {
                ctx->reference_frame = malloc(nbytes);
                if (!ctx->reference_frame) {
                    vp_log("ERROR: cannot allocate reference frame");
                    free(frame.pixels);
                    continue;
                }
                ctx->ref_width  = frame.width;
                ctx->ref_height = frame.height;
            }
            memcpy(ctx->reference_frame, frame.pixels, nbytes);
            ctx->ref_valid = true;
            for (uint32_t s = 0; s < ctx->roi_map.n_spots; s++) {
                ctx->spot_ctx[s].state = VP_SPOT_UNKNOWN;
                ctx->spot_ctx[s].candidate = VP_SPOT_UNKNOWN;
                ctx->spot_ctx[s].debounce_count = 0;
            }
            vp_log("analysis_thread: reference frame set (frame %u, %ux%u)",
                   frame.frame_id, frame.width, frame.height);
            free(frame.pixels);
            ctx->frames_processed++;
            continue;
        }

        for (uint32_t s = 0; s < ctx->roi_map.n_spots; s++) {
            const vp_roi_t *roi = &ctx->roi_map.spots[s];
            vp_spot_ctx_t *sctx = &ctx->spot_ctx[s];

            float ratio = vp_compute_change_ratio(ctx, roi, frame.pixels, frame.width, frame.height);
            vp_spot_state_t detected = vp_classify_spot(ratio, ctx->occupancy_ratio);

            if (detected == sctx->candidate) sctx->debounce_count++;
            else {
                sctx->candidate = detected;
                sctx->debounce_count = 1;
            }

            if (sctx->debounce_count >= ctx->debounce_frames && detected != sctx->state) {
                vp_event_t evt;
                evt.spot_id = roi->spot_id;
                snprintf(evt.label, sizeof(evt.label), "%s", roi->label);
                evt.old_state = sctx->state;
                evt.new_state = detected;
                evt.detected_ns = frame.timestamp_ns;
                evt.frame_id = frame.frame_id;
                evt.change_ratio = ratio;

                sctx->state = detected;
                sctx->last_change_ns = frame.timestamp_ns;
                sctx->change_count++;
                sctx->debounce_count = 0;

                if (vp_send_event(ctx, &evt) == 0) ctx->events_sent++;
            }
        }

        uint64_t latency = vp_now_ns() - t_start;
        ctx->total_latency_ns += latency;
        if (latency > ctx->max_latency_ns) ctx->max_latency_ns = latency;
        ctx->frames_processed++;

        free(frame.pixels);
    }

    vp_log("analysis_thread: exiting (frames=%llu events=%llu avg_lat=%llu ns max_lat=%llu ns)",
           (unsigned long long)ctx->frames_processed,
           (unsigned long long)ctx->events_sent,
           ctx->frames_processed ? (unsigned long long)(ctx->total_latency_ns / ctx->frames_processed) : 0ULL,
           (unsigned long long)ctx->max_latency_ns);
    return NULL;
}

int vp_init(vp_ctx_t *ctx, const vp_roi_map_t *roi_map, int pm_coid)
{
    memset(ctx, 0, sizeof(*ctx));
    if (roi_map) ctx->roi_map = *roi_map;

    ctx->diff_threshold = VP_DIFF_THRESHOLD;
    ctx->occupancy_ratio = VP_OCCUPANCY_RATIO;
    ctx->debounce_frames = VP_DEBOUNCE_FRAMES;
    ctx->pm_coid = pm_coid;
    ctx->running = false;

    for (uint32_t i = 0; i < VP_MAX_SPOTS; i++) {
        ctx->spot_ctx[i].state = VP_SPOT_UNKNOWN;
        ctx->spot_ctx[i].candidate = VP_SPOT_UNKNOWN;
    }

    if (pthread_mutex_init(&ctx->fq_mutex, NULL) != 0) return -1;
    if (pthread_cond_init(&ctx->fq_not_empty, NULL) != 0) return -1;
    if (pthread_cond_init(&ctx->fq_not_full, NULL) != 0) return -1;

    return 0;
}

int vp_start(vp_ctx_t *ctx)
{
    ctx->running = true;

    pthread_attr_t attr;
    pthread_attr_init(&attr);

    struct sched_param sp;
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    sp.sched_priority = 20;
    pthread_attr_setschedparam(&attr, &sp);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    if (pthread_create(&ctx->analysis_thread, &attr, analysis_thread_fn, ctx) != 0) {
#ifdef VPLINUX
        vp_log("WARN: failed to create SCHED_FIFO analysis_thread (%s); retrying default scheduling", strerror(errno));
        pthread_attr_destroy(&attr);
        pthread_attr_init(&attr);
        if (pthread_create(&ctx->analysis_thread, &attr, analysis_thread_fn, ctx) != 0) {
            ctx->running = false;
            pthread_attr_destroy(&attr);
            return -1;
        }
#else
        ctx->running = false;
        pthread_attr_destroy(&attr);
        return -1;
#endif
    }

    pthread_attr_destroy(&attr);
    return 0;
}

void vp_stop(vp_ctx_t *ctx)
{
    ctx->running = false;
    pthread_cond_broadcast(&ctx->fq_not_empty);
    pthread_cond_broadcast(&ctx->fq_not_full);
    pthread_join(ctx->analysis_thread, NULL);
}

void vp_destroy(vp_ctx_t *ctx)
{
    free(ctx->reference_frame);
    ctx->reference_frame = NULL;
    pthread_mutex_destroy(&ctx->fq_mutex);
    pthread_cond_destroy(&ctx->fq_not_empty);
    pthread_cond_destroy(&ctx->fq_not_full);
    memset(ctx, 0, sizeof(*ctx));
}
