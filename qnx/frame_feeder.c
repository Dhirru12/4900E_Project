#define _POSIX_C_SOURCE 200809L

/*
 * frame_feeder.c  –  TCP-only final version
 * Smart Parking System – Vision Producer Frame Feeder
 *
 * PURPOSE
 * -------
 * Receives grayscale frames over TCP from a Windows/host sender and
 * forwards them into the Vision Producer queue for occupancy analysis.
 *
 * FORMAT
 * -----------
 * Per frame:
 *   [uint32 frame_id]
 *   [uint32 width]
 *   [uint32 height]
 *   [width * height bytes grayscale pixels]
 */

#include "vision_producer.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <sys/neutrino.h>

/* Constants */

#define FF_MAX_FRAME_BYTES   (3840u * 2160u)   /* 4K grayscale upper bound */
#define FF_PM_INFO_PATH      "/tmp/pm_channel_id"
#define FF_DRAIN_TIMEOUT_MS  5000

typedef struct {
    uint32_t frame_id;
    uint32_t width;
    uint32_t height;
} ff_frame_hdr_t;

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
        if (r == 0) return -1;   /* peer closed */
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
        sleep_ns(10000000ULL); /* 10 ms */
    }
}

static void build_default_roi(vp_roi_map_t *map)
{
    fprintf(stderr, "[FF] no ROI map provided – using synthetic 6-spot grid\n");

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

static int ff_get_pm_endpoint(void)
{
    FILE *f = fopen(FF_PM_INFO_PATH, "r");
    if (!f) {
        fprintf(stderr,
                "[FF] ERROR: cannot open %s: %s\n",
                FF_PM_INFO_PATH, strerror(errno));
        fprintf(stderr,
                "[FF] Start parking_manager_qnx first, and ensure it writes: <pid> <chid>\n");
        return -1;
    }

    int pid = -1;
    int chid = -1;
    int n = fscanf(f, "%d %d", &pid, &chid);
    fclose(f);

    if (n != 2 || pid <= 0 || chid < 0) {
        fprintf(stderr,
                "[FF] ERROR: invalid content in %s (expected: <pid> <chid>)\n",
                FF_PM_INFO_PATH);
        return -1;
    }

    int coid = ConnectAttach(0, pid, chid, 0, 0);
    if (coid == -1) {
        fprintf(stderr,
                "[FF] ERROR: ConnectAttach(pid=%d, chid=%d) failed: %s\n",
                pid, chid, strerror(errno));
        return -1;
    }

    fprintf(stderr, "[FF] connected to Parking Manager pid=%d chid=%d coid=%d\n",
            pid, chid, coid);
    return coid;
}

static int ff_run_tcp_server(vp_ctx_t *ctx, uint16_t port)
{
    int listen_fd = -1;
    int conn_fd = -1;
    uint64_t frames = 0;
    uint8_t *pixel_buf = NULL;
    size_t pixel_buf_cap = 0;

    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        perror("[FF-TCP] socket");
        return -1;
    }

    {
        int opt = 1;
        if (setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            perror("[FF-TCP] setsockopt");
            close(listen_fd);
            return -1;
        }
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[FF-TCP] bind");
        close(listen_fd);
        return -1;
    }

    if (listen(listen_fd, 1) < 0) {
        perror("[FF-TCP] listen");
        close(listen_fd);
        return -1;
    }

    fprintf(stderr, "[FF-TCP] listening on port %u\n", (unsigned)port);
    fprintf(stderr, "[FF-TCP] waiting for host connection...\n");

    conn_fd = accept(listen_fd, NULL, NULL);
    if (conn_fd < 0) {
        perror("[FF-TCP] accept");
        close(listen_fd);
        return -1;
    }

    close(listen_fd);
    listen_fd = -1;

    fprintf(stderr, "[FF-TCP] host connected\n");

    while (ctx->running) {
        ff_frame_hdr_t hdr;

        if (recv_exact(conn_fd, &hdr, sizeof(hdr)) != 0) {
            fprintf(stderr, "[FF-TCP] connection closed after %llu frames\n",
                    (unsigned long long)frames);
            break;
        }

        size_t nbytes = (size_t)hdr.width * (size_t)hdr.height;

        if (hdr.width == 0 || hdr.height == 0 || nbytes > FF_MAX_FRAME_BYTES) {
            fprintf(stderr,
                    "[FF-TCP] invalid frame header id=%u w=%u h=%u\n",
                    hdr.frame_id, hdr.width, hdr.height);
            break;
        }

        if (nbytes > pixel_buf_cap) {
            free(pixel_buf);
            pixel_buf = (uint8_t *)malloc(nbytes);
            if (!pixel_buf) {
                fprintf(stderr, "[FF-TCP] OOM allocating %zu bytes\n", nbytes);
                break;
            }
            pixel_buf_cap = nbytes;
        }

        if (recv_exact(conn_fd, pixel_buf, nbytes) != 0) {
            fprintf(stderr, "[FF-TCP] short pixel read on frame %u\n", hdr.frame_id);
            break;
        }

        uint8_t *owned = (uint8_t *)malloc(nbytes);
        if (!owned) {
            fprintf(stderr, "[FF-TCP] OOM allocating owned frame %u\n", hdr.frame_id);
            break;
        }
        memcpy(owned, pixel_buf, nbytes);

        vp_frame_t frame;
        frame.pixels       = owned;
        frame.width        = hdr.width;
        frame.height       = hdr.height;
        frame.timestamp_ns = vp_now_ns();
        frame.frame_id     = hdr.frame_id;

        if (vp_enqueue_frame(ctx, &frame) != 0) {
            free(owned);
            fprintf(stderr, "[FF-TCP] enqueue failed on frame %u\n", hdr.frame_id);
            break;
        }

        frames++;
        if (frames % 100 == 0) {
            fprintf(stderr, "[FF-TCP] %llu frames ingested\n",
                    (unsigned long long)frames);
        }
    }

    free(pixel_buf);

    if (conn_fd >= 0) close(conn_fd);
    if (listen_fd >= 0) close(listen_fd);

    fprintf(stderr, "[FF-TCP] done – %llu total frames received\n",
            (unsigned long long)frames);
    return 0;
}

#ifdef FF_STANDALONE

static void print_usage(const char *prog)
{
    fprintf(stderr,
        "\nUsage:\n"
        "  %s <port> [roi.json] [--ref <file.raw> <width> <height>]\n\n"
        "Options:\n"
        "  --ref <file> <W> <H>  Load a pre-captured empty-lot reference frame.\n"
        "                        The file must be raw 8-bit grayscale, W*H bytes.\n"
        "                        Without --ref the first video frame is used\n"
        "                        (which is wrong if that frame already has cars).\n\n"
        "Examples:\n"
        "  %s 5000 roi_map_output.json --ref empty_ref_1100x720.raw 1100 720\n"
        "  %s 5000 roi_map_output.json\n"
        "  %s 5000\n\n",
        prog, prog, prog, prog);
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    uint16_t port = (uint16_t)atoi(argv[1]);
    if (port == 0) {
        fprintf(stderr, "[FF] invalid TCP port: %s\n", argv[1]);
        return 1;
    }

    const char *roi_path = NULL;
    const char *ref_path = NULL;
    uint32_t    ref_w    = 0;
    uint32_t    ref_h    = 0;

    int i = 2;
    while (i < argc) {
        if (strcmp(argv[i], "--ref") == 0) {
            if (i + 3 >= argc) {
                fprintf(stderr, "[FF] --ref requires <file> <width> <height>\n");
                return 1;
            }
            ref_path = argv[i + 1];
            ref_w    = (uint32_t)atoi(argv[i + 2]);
            ref_h    = (uint32_t)atoi(argv[i + 3]);
            if (ref_w == 0 || ref_h == 0) {
                fprintf(stderr, "[FF] invalid --ref dimensions: %s x %s\n",
                        argv[i + 2], argv[i + 3]);
                return 1;
            }
            i += 4;
        } else if (!roi_path) {
            roi_path = argv[i];
            i++;
        } else {
            fprintf(stderr, "[FF] unexpected argument: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    vp_roi_map_t roi_map;
    memset(&roi_map, 0, sizeof(roi_map));

    if (roi_path) {
        if (vp_roi_map_load_json(&roi_map, roi_path) != 0) {
            fprintf(stderr, "[FF] failed to load ROI map: %s\n", roi_path);
            return 1;
        }
    } else {
        build_default_roi(&roi_map);
    }

    int pm_coid = ff_get_pm_endpoint();
    if (pm_coid < 0) {
        return 1;
    }

    vp_ctx_t ctx;
    if (vp_init(&ctx, &roi_map, pm_coid) != 0) {
        fprintf(stderr, "[FF] vp_init failed\n");
        ConnectDetach(pm_coid);
        return 1;
    }

    if (ref_path) {
        if (vp_load_reference(&ctx, ref_path, ref_w, ref_h) != 0) {
            fprintf(stderr, "[FF] failed to load reference: %s\n", ref_path);
            vp_destroy(&ctx);
            ConnectDetach(pm_coid);
            return 1;
        }
        fprintf(stderr, "[FF] using pre-loaded empty-lot reference (%ux%u)\n",
                ref_w, ref_h);
    } else {
        fprintf(stderr, "[FF] WARNING: no --ref given; first video frame will "
                "be used as reference (incorrect if it contains parked cars)\n");
    }

    if (vp_start(&ctx) != 0) {
        fprintf(stderr, "[FF] vp_start failed\n");
        vp_destroy(&ctx);
        ConnectDetach(pm_coid);
        return 1;
    }

    fprintf(stderr, "[FF] TCP-only feeder started\n");

    ff_run_tcp_server(&ctx, port);

    ff_drain(&ctx, FF_DRAIN_TIMEOUT_MS);
    vp_stop(&ctx);
    vp_destroy(&ctx);

    ConnectDetach(pm_coid);

    fprintf(stderr, "[FF] shutdown complete\n");
    return 0;
}

#endif
