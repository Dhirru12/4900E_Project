#define _POSIX_C_SOURCE 200809L

/*
 * Receives occupancy events from the Vision Producer and maintains
 * the authoritative parking lot state.
 */

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

#include <sys/neutrino.h>
#include <sys/iomsg.h>

#define PM_CHANNEL_PATH      "/tmp/pm_channel_id"   /* QNX: write chid here */
#define PM_SCHED_PRIORITY    30                      /* above VP (20)        */
#define PM_SUMMARY_INTERVAL  10                      /* print every N events */
#define PM_MAX_SPOTS         VP_MAX_SPOTS

typedef struct {
    uint32_t        spot_id;
    char            label[16];
    vp_spot_state_t state;
    uint64_t        last_change_ns;
    uint32_t        total_changes;
} pm_spot_t;

typedef struct {
    pm_spot_t   spots[PM_MAX_SPOTS];
    uint32_t    n_spots;
    uint32_t    n_free;
    uint32_t    n_occupied;
    uint32_t    n_unknown;
    uint64_t    events_received;
    pthread_mutex_t lock;
} pm_state_t;

static pm_state_t g_state;
static volatile sig_atomic_t g_stop = 0;

void pm_print_summary(pm_state_t *st);

/* Signal handler */

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
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
    fprintf(stderr, "[PM %llu.%06llu] %s\n",
            (unsigned long long)(ns / 1000000000ULL),
            (unsigned long long)((ns % 1000000000ULL) / 1000ULL),
            buf);
}

static const char *state_str(vp_spot_state_t s)
{
    switch (s) {
        case VP_SPOT_FREE:     return "FREE    ";
        case VP_SPOT_OCCUPIED: return "OCCUPIED";
        default:               return "UNKNOWN ";
    }
}

static pm_spot_t *pm_get_spot(pm_state_t *st, uint32_t spot_id,
                               const char *label)
{
    for (uint32_t i = 0; i < st->n_spots; i++) {
        if (st->spots[i].spot_id == spot_id)
            return &st->spots[i];
    }
    if (st->n_spots >= PM_MAX_SPOTS) return NULL;

    pm_spot_t *s = &st->spots[st->n_spots++];
    memset(s, 0, sizeof(*s));
    s->spot_id       = spot_id;
    s->state         = VP_SPOT_UNKNOWN;
    s->last_change_ns = 0;
    s->total_changes = 0;
    snprintf(s->label, sizeof(s->label), "%s", (label && *label) ? label : "UNLABELED");
    return s;
}

static void pm_recount(pm_state_t *st)
{
    st->n_free = st->n_occupied = st->n_unknown = 0;
    for (uint32_t i = 0; i < st->n_spots; i++) {
        switch (st->spots[i].state) {
            case VP_SPOT_FREE:     st->n_free++;     break;
            case VP_SPOT_OCCUPIED: st->n_occupied++; break;
            default:               st->n_unknown++;  break;
        }
    }
}

/* Process one event */

static void pm_apply_event(pm_state_t *st, const vp_event_t *evt)
{
    pthread_mutex_lock(&st->lock);

    pm_spot_t *spot = pm_get_spot(st, evt->spot_id, evt->label);
    if (!spot) {
        pm_log("WARN: spot table full – dropping event for %s", evt->label);
        pthread_mutex_unlock(&st->lock);
        return;
    }

    vp_spot_state_t prev = spot->state;
    spot->state          = evt->new_state;
    spot->last_change_ns = evt->detected_ns;
    spot->total_changes++;
    st->events_received++;

    pm_recount(st);

    uint64_t total = st->n_spots;
    uint64_t ev    = st->events_received;

    pthread_mutex_unlock(&st->lock);

    /* Log every transition */
    pm_log("%-10s  %s → %s  ratio=%.3f  frame=%u  "
           "[FREE:%u  OCC:%u  UNK:%u / %llu total]",
           evt->label,
           state_str(prev),
           state_str(evt->new_state),
           evt->change_ratio,
           evt->frame_id,
           st->n_free, st->n_occupied, st->n_unknown,
           (unsigned long long)total);

    (void)ev;
}

/* MsgReceive on a named channel */

static int pm_run_qnx(pm_state_t *st)
{
    struct sched_param sp;
    sp.sched_priority = PM_SCHED_PRIORITY;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        pm_log("WARN: could not set SCHED_FIFO (need elevated caps): %s",
               strerror(errno));

    int chid = ChannelCreate(0);
    if (chid == -1) {
        pm_log("ERROR: ChannelCreate failed: %s", strerror(errno));
        return -1;
    }

    FILE *f = fopen(PM_CHANNEL_PATH, "w");
    if (f) {
        fprintf(f, "%d %d\n", getpid(), chid);
        fclose(f);
        pm_log("channel pid=%d chid=%d written to %s", getpid(), chid, PM_CHANNEL_PATH);
    }

    int pid  = getpid();
    int coid = ConnectAttach(0, pid, chid, _NTO_SIDE_CHANNEL, 0);
    if (coid == -1)
        pm_log("WARN: self-connect failed: %s", strerror(errno));

    pm_log("listening on chid=%d  pid=%d", chid, pid);
    pm_log("Vision Producer connect: ConnectAttach(0, %d, %d, 0, 0)", pid, chid);

    while (!g_stop) {
        vp_event_t  evt;
        uint32_t    reply = 0;
        int rcvid = MsgReceive(chid, &evt, sizeof(evt), NULL);

        if (rcvid == -1) {
            if (errno == EINTR) continue;
            pm_log("ERROR: MsgReceive failed: %s", strerror(errno));
            break;
        }

        /* Reply immediately to unblock the Vision Producer */
        MsgReply(rcvid, 0, &reply, sizeof(reply));

        pm_apply_event(st, &evt);
    }

    ChannelDestroy(chid);
    if (coid != -1) ConnectDetach(coid);
    remove(PM_CHANNEL_PATH);
    return 0;
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    memset(&g_state, 0, sizeof(g_state));
    pthread_mutex_init(&g_state.lock, NULL);

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    pm_log("Parking Manager started");

    pm_run_qnx(&g_state);

    pm_log("shutting down – %llu events processed",
           (unsigned long long)g_state.events_received);

    pthread_mutex_destroy(&g_state.lock);
    return 0;
}
