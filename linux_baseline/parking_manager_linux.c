#define _POSIX_C_SOURCE 200809L

#include "vision_producer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>

#ifdef __APPLE__
#define VP_LOCAL_SOCK_TYPE SOCK_STREAM
#define VP_SEND_FLAGS 0
#else
#define VP_LOCAL_SOCK_TYPE SOCK_SEQPACKET
#define VP_SEND_FLAGS MSG_NOSIGNAL
#endif

#define PM_SCHED_PRIORITY 30
#define PM_MAX_SPOTS      VP_MAX_SPOTS

typedef struct {
    uint32_t        spot_id;
    char            label[16];
    vp_spot_state_t state;
    uint64_t        last_change_ns;
    uint32_t        total_changes;
} pm_spot_t;

typedef struct {
    pm_spot_t       spots[PM_MAX_SPOTS];
    uint32_t        n_spots;
    uint32_t        n_free;
    uint32_t        n_occupied;
    uint32_t        n_unknown;
    uint64_t        events_received;
    pthread_mutex_t lock;
} pm_state_t;

static pm_state_t g_state;
static volatile sig_atomic_t g_stop = 0;
static int g_listen_fd = -1;

static void pm_log(const char *fmt, ...);

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

static void pm_try_enable_rt(void)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = PM_SCHED_PRIORITY;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        pm_log("WARN: pthread_setschedparam failed: %s", strerror(errno));
    }
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

static int send_exact(int fd, const void *buf, size_t n)
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

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
    if (g_listen_fd >= 0) close(g_listen_fd);
    unlink(VP_PM_UNIX_SOCK_PATH);
}

static uint64_t pm_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static void pm_log(const char *fmt, ...)
{
    va_list ap;
    char buf[256];
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uint64_t ns = pm_now_ns();
    fprintf(stderr, "[PM-LINUX %llu.%06llu] %s\n",
            (unsigned long long)(ns / 1000000000ULL),
            (unsigned long long)((ns % 1000000000ULL) / 1000ULL),
            buf);
}

static const char *state_str(vp_spot_state_t s)
{
    switch (s) {
        case VP_SPOT_FREE:     return "FREE";
        case VP_SPOT_OCCUPIED: return "OCCUPIED";
        default:               return "UNKNOWN";
    }
}

static pm_spot_t *pm_get_spot(pm_state_t *st, uint32_t spot_id, const char *label)
{
    for (uint32_t i = 0; i < st->n_spots; i++) {
        if (st->spots[i].spot_id == spot_id) return &st->spots[i];
    }
    if (st->n_spots >= PM_MAX_SPOTS) return NULL;

    pm_spot_t *s = &st->spots[st->n_spots++];
    memset(s, 0, sizeof(*s));
    s->spot_id = spot_id;
    s->state = VP_SPOT_UNKNOWN;
    snprintf(s->label, sizeof(s->label), "%s", (label && *label) ? label : "UNLABELED");
    return s;
}

static void pm_recount(pm_state_t *st)
{
    st->n_free = st->n_occupied = st->n_unknown = 0;
    for (uint32_t i = 0; i < st->n_spots; i++) {
        switch (st->spots[i].state) {
            case VP_SPOT_FREE: st->n_free++; break;
            case VP_SPOT_OCCUPIED: st->n_occupied++; break;
            default: st->n_unknown++; break;
        }
    }
}

static void pm_apply_event(pm_state_t *st, const vp_event_t *evt)
{
    pthread_mutex_lock(&st->lock);
    pm_spot_t *spot = pm_get_spot(st, evt->spot_id, evt->label);
    if (!spot) {
        pthread_mutex_unlock(&st->lock);
        return;
    }

    vp_spot_state_t prev = spot->state;
    spot->state = evt->new_state;
    spot->last_change_ns = evt->detected_ns;
    spot->total_changes++;
    st->events_received++;
    pm_recount(st);
    uint64_t total = st->n_spots;
    pthread_mutex_unlock(&st->lock);

    pm_log("%s %s -> %s ratio=%.3f frame=%u [F:%u O:%u U:%u / %llu]",
           evt->label, state_str(prev), state_str(evt->new_state), evt->change_ratio,
           evt->frame_id, st->n_free, st->n_occupied, st->n_unknown,
           (unsigned long long)total);
}

static int pm_run_linux(pm_state_t *st)
{
    pm_try_enable_rt();

    int listen_fd = socket(AF_UNIX, VP_LOCAL_SOCK_TYPE, 0);
    if (listen_fd < 0) return -1;
    g_listen_fd = listen_fd;

    unlink(VP_PM_UNIX_SOCK_PATH);
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", VP_PM_UNIX_SOCK_PATH);

    if (set_no_sigpipe(listen_fd) != 0) return -1;
    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) return -1;
    if (listen(listen_fd, 4) != 0) return -1;

    pm_log("listening on %s", VP_PM_UNIX_SOCK_PATH);

    while (!g_stop) {
        int conn_fd = accept(listen_fd, NULL, NULL);
        if (conn_fd < 0) {
            if (errno == EINTR) continue;
            if (g_stop) break;
            return -1;
        }

        if (set_no_sigpipe(conn_fd) != 0) { close(conn_fd); continue; }

        while (!g_stop) {
            vp_event_t evt;
            uint32_t ack = 0;
            if (recv_exact(conn_fd, &evt, sizeof(evt)) != 0) break;
            pm_apply_event(st, &evt);
            if (send_exact(conn_fd, &ack, sizeof(ack)) != 0) break;
        }
        close(conn_fd);
    }

    close(listen_fd);
    unlink(VP_PM_UNIX_SOCK_PATH);
    g_listen_fd = -1;
    return 0;
}

int main(void)
{
    memset(&g_state, 0, sizeof(g_state));
    pthread_mutex_init(&g_state.lock, NULL);
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    pm_log("Linux Parking Manager started");
    pm_run_linux(&g_state);
    pm_log("shutting down – %llu events processed", (unsigned long long)g_state.events_received);
    pthread_mutex_destroy(&g_state.lock);
    return 0;
}
