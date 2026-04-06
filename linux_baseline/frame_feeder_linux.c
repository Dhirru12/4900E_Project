#define _POSIX_C_SOURCE 200809L

#include "vision_producer.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/un.h>

#ifdef __APPLE__
#define VP_LOCAL_SOCK_TYPE SOCK_STREAM
#else
#define VP_LOCAL_SOCK_TYPE SOCK_SEQPACKET
#endif

static int set_no_sigpipe(int fd)
{
#if defined(SO_NOSIGPIPE)
    int one = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one)) != 0)
        return -1;
#else
    (void)fd;
#endif
    return 0;
}

static void ff_try_enable_rt(void)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = 10;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        fprintf(stderr, "[FF-LINUX] WARN: pthread_setschedparam failed: %s\n", strerror(errno));
    }
}

typedef struct {
    uint32_t frame_id;
    uint32_t width;
    uint32_t height;
} ff_frame_hdr_t;

#define FF_MAX_FRAME_BYTES   (3840u * 2160u)
#define FF_DRAIN_TIMEOUT_MS  5000

static void sleep_ns(uint64_t ns)
{
    struct timespec ts;
    ts.tv_sec  = (time_t)(ns / 1000000000ULL);
    ts.tv_nsec = (long)(ns % 1000000000ULL);
    nanosleep(&ts, NULL);
}

static int recv_exact(int fd, void *buf, size_t n)
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

static void ff_drain(vp_ctx_t *ctx, int timeout_ms)
{
    int steps = timeout_ms / 10;
    for (int i = 0; i < steps; i++) {
        pthread_mutex_lock(&ctx->fq_mutex);
        bool empty = (ctx->fq_head == ctx->fq_tail);
        pthread_mutex_unlock(&ctx->fq_mutex);
        if (empty) break;
        sleep_ns(10000000ULL);
    }
}

static void build_default_roi(vp_roi_map_t *map)
{
    memset(map, 0, sizeof(*map));
    uint32_t sw = 80, sh = 120, gap = 10, ox = 40, oy = 60;
    for (uint32_t row = 0; row < 2; row++) {
        for (uint32_t col = 0; col < 3; col++) {
            char lbl[16];
            snprintf(lbl, sizeof(lbl), "S%u%u", row, col);
            vp_roi_map_add_rect(map, row * 3 + col, lbl,
                                ox + col * (sw + gap),
                                oy + row * (sh + gap),
                                sw, sh);
        }
    }
}

static int ff_connect_pm_linux(void)
{
    int fd = socket(AF_UNIX, VP_LOCAL_SOCK_TYPE, 0);
    if (fd < 0) return -1;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", VP_PM_UNIX_SOCK_PATH);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        close(fd);
        return -1;
    }
    if (set_no_sigpipe(fd) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static int ff_run_tcp_server(vp_ctx_t *ctx, uint16_t port)
{
    int listen_fd = -1, conn_fd = -1;
    uint64_t frames = 0;
    uint8_t *pixel_buf = NULL;
    size_t pixel_buf_cap = 0;

    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) return -1;

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) return -1;
    if (listen(listen_fd, 1) < 0) return -1;

    fprintf(stderr, "[FF-LINUX] listening on port %u\n", (unsigned)port);
    conn_fd = accept(listen_fd, NULL, NULL);
    if (conn_fd < 0) return -1;
    close(listen_fd);
    listen_fd = -1;

    while (ctx->running) {
        ff_frame_hdr_t hdr;
        if (recv_exact(conn_fd, &hdr, sizeof(hdr)) != 0) break;

        size_t nbytes = (size_t)hdr.width * (size_t)hdr.height;
        if (hdr.width == 0 || hdr.height == 0 || nbytes > FF_MAX_FRAME_BYTES) break;

        if (nbytes > pixel_buf_cap) {
            free(pixel_buf);
            pixel_buf = malloc(nbytes);
            if (!pixel_buf) break;
            pixel_buf_cap = nbytes;
        }

        if (recv_exact(conn_fd, pixel_buf, nbytes) != 0) break;

        uint8_t *owned = malloc(nbytes);
        if (!owned) break;
        memcpy(owned, pixel_buf, nbytes);

        vp_frame_t frame = {
            .pixels = owned,
            .width = hdr.width,
            .height = hdr.height,
            .timestamp_ns = vp_now_ns(),
            .frame_id = hdr.frame_id
        };

        if (vp_enqueue_frame(ctx, &frame) != 0) {
            free(owned);
            break;
        }
        frames++;
    }

    free(pixel_buf);
    if (conn_fd >= 0) close(conn_fd);
    if (listen_fd >= 0) close(listen_fd);
    fprintf(stderr, "[FF-LINUX] done – %llu frames\n", (unsigned long long)frames);
    return 0;
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <port> [roi.json] [--ref file w h]\n", argv[0]);
        return 1;
    }

    uint16_t port = (uint16_t)atoi(argv[1]);
    const char *roi_path = NULL;
    const char *ref_path = NULL;
    uint32_t ref_w = 0, ref_h = 0;

    signal(SIGPIPE, SIG_IGN);
    ff_try_enable_rt();

    int i = 2;
    while (i < argc) {
        if (strcmp(argv[i], "--ref") == 0 && i + 3 < argc) {
            ref_path = argv[i + 1];
            ref_w = (uint32_t)atoi(argv[i + 2]);
            ref_h = (uint32_t)atoi(argv[i + 3]);
            i += 4;
        } else if (!roi_path) {
            roi_path = argv[i++];
        } else {
            return 1;
        }
    }

    vp_roi_map_t roi_map;
    memset(&roi_map, 0, sizeof(roi_map));
    if (roi_path) {
        if (vp_roi_map_load_json(&roi_map, roi_path) != 0) return 1;
    } else {
        build_default_roi(&roi_map);
    }

    int pm_fd = ff_connect_pm_linux();
    if (pm_fd < 0) {
        fprintf(stderr, "[FF-LINUX] failed to connect to %s\n", VP_PM_UNIX_SOCK_PATH);
        return 1;
    }

    vp_ctx_t ctx;
    if (vp_init(&ctx, &roi_map, pm_fd) != 0) return 1;
    if (ref_path && vp_load_reference(&ctx, ref_path, ref_w, ref_h) != 0) return 1;
    if (vp_start(&ctx) != 0) return 1;

    ff_run_tcp_server(&ctx, port);
    ff_drain(&ctx, FF_DRAIN_TIMEOUT_MS);
    vp_stop(&ctx);
    vp_destroy(&ctx);
    close(pm_fd);
    return 0;
}
