#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <mqueue.h>
#include <errno.h>
#include <time.h>        /* for clock_gettime */
#include <signal.h>      /* for signal handling */
#include <sys/poll.h>    /* for poll() */
#include <fcntl.h>       /* for open() */

#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>   /* SPI (spidev) */

static int file_spi = -1;  
static uint32_t spi_speed_hz = 1000000; /* 1 MHz */
static uint8_t  spi_bits      = 8;
static uint8_t  spi_mode      = SPI_MODE_3; /* TMC5130: mode 3 */

/* -------- SPI status byte (bits 39..32 of any SPI reply) -------- */
#define SPI_STATUS_STOP_R       (1u << 7) // bit7: status_stop_r, most significant bit (MSB)
#define SPI_STATUS_STOP_L       (1u << 6) // bit6: status_stop_l
#define SPI_STATUS_POS_REACHED  (1u << 5) // bit5: position_reached
#define SPI_STATUS_VEL_REACHED  (1u << 4) // bit4: velocity_reached
#define SPI_STATUS_STANDSTILL   (1u << 3) // bit3: standstill
#define SPI_STATUS_SG2          (1u << 2) // bit2: sg2
#define SPI_STATUS_DRIVER_ERR   (1u << 1) // bit1: driver_error
#define SPI_STATUS_RESET_FLAG   (1u << 0) // bit0: reset_flag, least significant bit (LSB)

/* TMC5130 register address for GSTAT (needed by HAO when building the read) */
#ifndef TMC5130_GSTAT
#define TMC5130_GSTAT 0x01
#endif

int tmc5130init(int bus, int cs);

#define Q_TS_OPAO    "/status_req_queue"
#define Q_TS_HAO     "/poll_req_queue"
#define Q_OPAO_HAO   "/status_req_opao2hao"
#define Q_HAO_OPAO   "/status_res_hao2opao"
#define Q_HAO_BAO    "/poll_req_hao2bao"
#define Q_BAO_HAO    "/poll_req_hao2bao_ack"

#define STATUS_REQ   "STATUS_REQ"
#define POLL_REQ     "POLL_REQ"
#define STATUS_RES   "STATUS_RES"
#define POLL_RES     "POLL_RES"

#define MSG_SIZE     64
#define INTERVAL_SEC 1

static mqd_t mq_ts_opao  = (mqd_t)-1;
static mqd_t mq_ts_hao   = (mqd_t)-1;
static mqd_t mq_opao_hao = (mqd_t)-1;
static mqd_t mq_hao_opao = (mqd_t)-1;
static mqd_t mq_hao_bao  = (mqd_t)-1;
static mqd_t mq_bao_hao  = (mqd_t)-1;

static int hao_counter     = 0;
static int last_bao_value  = 0;

void decode_tmc5130_readout(unsigned char *raw, int cnt, int *last_RESET, int *last_DRV_ERR, int *last_UV_CP)
{
    (void)cnt; /* BAO counter, not a length */

    /* Expect exactly 5 SPI bytes: [0]=SPI_STATUS, [1..4]=GSTAT */
    static uint8_t  prev_spi_status = 0xFF;       /* force first print */
    static uint32_t prev_gstat      = 0xFFFFFFFF; /* force first print */

    const uint8_t spi_status = raw[0];

    /* 32‑bit GSTAT value is big‑endian in bytes 1..4 */
    const uint32_t gstat =
        ((uint32_t)raw[1] << 24) |
        ((uint32_t)raw[2] << 16) |
        ((uint32_t)raw[3] <<  8) |
        ((uint32_t)raw[4] <<  0);

    /* Log SPI_STATUS (only when it changes) */
    if (spi_status != prev_spi_status) {
        printf("[DIAG] SPI_STATUS=0x%02X  ", spi_status);
        if (spi_status & SPI_STATUS_STOP_R){
            printf("STOP_R ");
        }
        if (spi_status & SPI_STATUS_STOP_L){
            printf("STOP_L ");
        }
        if (spi_status & SPI_STATUS_POS_REACHED){       
            printf("POS_REACHED ");
        }
        if (spi_status & SPI_STATUS_VEL_REACHED){
            printf("VEL_REACHED ");
        }
        if (spi_status & SPI_STATUS_STANDSTILL){
            printf("STANDSTILL ");
        }
        if (spi_status & SPI_STATUS_SG2){
            printf("SG2 ");
        }
        if (spi_status & SPI_STATUS_DRIVER_ERR){
            printf("DRIVER_ERR ");
        }
        if (spi_status & SPI_STATUS_RESET_FLAG){
            printf("RESET_FLAG ");
        }
        printf("\n");
        fflush(stdout);
        prev_spi_status = spi_status;
    }

    last_bao_value++; /* keep your existing counter semantics */

    /* GSTAT bits: bit0=RESET, bit1=DRV_ERR, bit2=UV_CP */
    const int reset  = (gstat & 0x01) ? 1 : 0;
    const int drvErr = (gstat & 0x02) ? 1 : 0;
    const int uvCp   = (gstat & 0x04) ? 1 : 0;

    /* Keep the existing out parameters (labels T/P/H in STATUS_RES) */
    *last_RESET = reset;        /* T  -> RESET   */
    *last_DRV_ERR = drvErr;     /* P  -> DRV_ERR */
    *last_UV_CP = uvCp;         /* H  -> UV_CP   */

    /* Log GSTAT (only when it changes) */
    if (gstat != prev_gstat) {
        printf("[DIAG] GSTAT(0x01)=0x%08X  RESET=%d  DRV_ERR=%d  UV_CP=%d\n",
               gstat, reset, drvErr, uvCp);

        if (reset) {
            printf("       RESET set → a reset occurred (power-on/soft). "
                   "Sticky; clear by writing 0x01 to GSTAT.\n");
        }
        if (drvErr) {
            printf("       DRV_ERR set → driver error latched. "
                   "Check DRVSTATUS(0x6F): OT/OTPW/S2GA/S2GB/OLA/OLB. "
                   "Sticky; clear GSTAT bit by writing '1'.\n");
        }
        if (uvCp) {
            printf("       UV_CP set → charge-pump undervoltage. "
                   "Check VM and VCP/Cp capacitors. "
                   "Sticky; clear GSTAT bit by writing '1'.\n");
        }
        fflush(stdout);
        prev_gstat = gstat;
    }
}

void cleanup_and_exit(int signo) {
    if (mq_ts_opao  != (mqd_t)-1) { mq_close(mq_ts_opao);  mq_unlink(Q_TS_OPAO); }
    if (mq_ts_hao   != (mqd_t)-1) { mq_close(mq_ts_hao);   mq_unlink(Q_TS_HAO);  }
    if (mq_opao_hao != (mqd_t)-1) { mq_close(mq_opao_hao); mq_unlink(Q_OPAO_HAO);}
    if (mq_hao_opao != (mqd_t)-1) { mq_close(mq_hao_opao); mq_unlink(Q_HAO_OPAO);}
    if (mq_hao_bao  != (mqd_t)-1) { mq_close(mq_hao_bao);  mq_unlink(Q_HAO_BAO); }
    if (mq_bao_hao  != (mqd_t)-1) { mq_close(mq_bao_hao);  mq_unlink(Q_BAO_HAO); }
    if (file_spi >= 0) close(file_spi);
    printf("\nCleaned up on Ctrl-C\n");
    exit(EXIT_SUCCESS);
}

void *time_sequencer(void *arg) {
    (void)arg;
    struct timespec ts      = { INTERVAL_SEC, 0 };
    struct timespec ts_half = { 0, INTERVAL_SEC * 1000000000L / 2 };
    struct timespec now;

    while (1) {
        nanosleep(&ts_half, NULL);
        mq_send(mq_ts_hao, POLL_REQ, strlen(POLL_REQ) + 1, 0);
        clock_gettime(CLOCK_MONOTONIC, &now);
        printf("[TS]: [%5ld.%09ld] Sent to HAO: %s\n", now.tv_sec, now.tv_nsec, POLL_REQ);
        fflush(stdout);

        nanosleep(&ts_half, NULL);
        mq_send(mq_ts_hao, POLL_REQ, strlen(POLL_REQ) + 1, 0);
        clock_gettime(CLOCK_MONOTONIC, &now);
        printf("[TS]: [%5ld.%09ld] Sent to HAO: %s\n", now.tv_sec, now.tv_nsec, POLL_REQ);
        fflush(stdout);

        nanosleep(&ts, NULL);
        mq_send(mq_ts_opao, STATUS_REQ, strlen(STATUS_REQ) + 1, 0);
        clock_gettime(CLOCK_MONOTONIC, &now);
        printf("[TS]: [%5ld.%09ld] Sent to OPAO: %s\n", now.tv_sec, now.tv_nsec, STATUS_REQ);
        fflush(stdout);
    }
    return NULL;
}

/* OPAO: unchanged behavior */
void *opao(void *arg) {
    (void)arg;
    char buf[MSG_SIZE];
    struct timespec now;
    ssize_t len;

    struct pollfd pfds[2] = {
        { .fd = (int)mq_ts_opao,  .events = POLLIN },
        { .fd = (int)mq_hao_opao, .events = POLLIN }
    };
    
    while (1) {
        if (poll(pfds, 2, -1) == -1) { perror("[OPAO]: poll"); break; }

        if (pfds[0].revents & POLLIN) {
            len = mq_receive(mq_ts_opao, buf, MSG_SIZE, NULL);
            if (len == -1) { perror("[OPAO]: mq_receive TS→OPAO"); break; }

            if (strcmp(buf, STATUS_REQ) == 0) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[OPAO]: [%5ld.%09ld] From TS: %s\n", now.tv_sec, now.tv_nsec, buf);
                fflush(stdout);
                mq_send(mq_opao_hao, buf, len, 0);
            }
        }

        if (pfds[1].revents & POLLIN) {
            len = mq_receive(mq_hao_opao, buf, MSG_SIZE, NULL);
            if (len == -1) { perror("[OPAO]: mq_receive HAO→OPAO"); break; }

            if (strncmp(buf, STATUS_RES ":", strlen(STATUS_RES)+1) == 0) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[OPAO]: [%5ld.%09ld] %s From HAO\n", now.tv_sec, now.tv_nsec, buf);
                printf("----------------------------------------------------\n");
                fflush(stdout);
            }
        }
    }
    return NULL;
}

/* HAO: build "POLL_REQ\0 + 1-byte address (0x01)" for GSTAT over SPI */
void *hao(void *arg) {
    (void)arg;
    char buf[MSG_SIZE + 8];
    struct timespec now;
    ssize_t len;

    struct pollfd pfds[3] = {
        { .fd = (int)mq_ts_hao,   .events = POLLIN },
        { .fd = (int)mq_opao_hao, .events = POLLIN },
        { .fd = (int)mq_bao_hao,  .events = POLLIN }
    };

    int last_RESET = 0, last_DRV_ERR = 0, last_UV_CP = 0;
    
    while (1) {
        if (poll(pfds, 3, -1) == -1) { perror("[HAO]: poll"); break; }

        /* TS → HAO: POLL_REQ */
        if (pfds[0].revents & POLLIN) {
            len = mq_receive(mq_ts_hao, buf, MSG_SIZE, NULL);
            if (len == -1) { perror("[HAO]: mq_receive TS→HAO"); break; }

            if (strcmp(buf, POLL_REQ) == 0) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[HAO]: [%5ld.%09ld] From TS: %s\n", now.tv_sec, now.tv_nsec, buf);
                fflush(stdout);

                unsigned char msg[MSG_SIZE];
                size_t base = strlen(POLL_REQ) + 1;
                memcpy(msg, POLL_REQ, base); /* includes '\0' */
                msg[base + 0] = (uint8_t)(TMC5130_GSTAT & 0x7F); /* read address for SPI */
                printf("[HAO]: Sending address 0x%02X to BAO\n", msg[base + 0]); // Added line
                printf("[HAO]: sending SPI read request for GSTAT (addr=0x%02X) to BAO\n", msg[base]);

                mq_send(mq_hao_bao, (char*)msg, base + 1, 0);
            }
        }

        /* OPAO → HAO: STATUS_REQ */
        if (pfds[1].revents & POLLIN) {
            len = mq_receive(mq_opao_hao, buf, MSG_SIZE, NULL);
            if (len == -1) { perror("[HAO]: mq_receive OPAO→HAO"); break; }

            if (strcmp(buf, STATUS_REQ) == 0) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[HAO]: [%5ld.%09ld] From OPAO: %s\n", now.tv_sec, now.tv_nsec, buf);
                fflush(stdout);
                char resp[MSG_SIZE];
                snprintf(resp, MSG_SIZE, STATUS_RES ":%d T=%d P=%d H=%d",
                    last_bao_value, last_RESET, last_DRV_ERR, last_UV_CP); // keep API labels
                mq_send(mq_hao_opao, resp, strlen(resp)+1, 0);
            }
        }

        /* BAO → HAO: POLL_RES:<n>\0 + 5 raw SPI bytes */
        if (pfds[2].revents & POLLIN) {
            len = mq_receive(mq_bao_hao, buf, MSG_SIZE+8, NULL);
            if (len == -1) { perror("[HAO]: mq_receive BAO→HAO"); break; }
            
            if (strncmp(buf, POLL_RES ":", strlen(POLL_RES) + 1) == 0) {
                int got = len;
                fprintf(stderr, "[HAO]: Received from BAO → ");
                for (int i = 0; i < got; i++) fprintf(stderr, "%02X ", (unsigned char)buf[i]);
                fprintf(stderr, "\n");

                char *hdr = buf; /* "POLL_RES:<n>\0" */
                unsigned char *raw = (unsigned char*)(buf + strlen(hdr) + 1);
                int cnt = atoi(hdr + strlen(POLL_RES) + 1);

                decode_tmc5130_readout(raw, cnt, &last_RESET, &last_DRV_ERR, &last_UV_CP);

                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[HAO]: [...] GSTAT bits => RESET=%d DRV_ERR=%d UV_CP=%d\n", last_RESET, last_DRV_ERR, last_UV_CP);
                fflush(stdout);
            }
        }
    }
    return NULL;
}

/* BAO: do the two 5‑byte SPI transactions to read GSTAT and send back the second reply */
void *bao(void *arg) {
    (void)arg;
    char buf[MSG_SIZE+8];
    struct timespec now;
    ssize_t len;
    int bao_counter = 0;
    struct pollfd p_fb = { .fd = (int)mq_hao_bao, .events = POLLIN };
    
    unsigned char rx2[5];

    while (1) {
        if (poll(&p_fb, 1, -1) == -1) { perror("[BAO]: poll"); break; }

        if (p_fb.revents & POLLIN) {
            len = mq_receive(mq_hao_bao, buf, MSG_SIZE+8, NULL);
            if (len == -1) { perror("[BAO]: mq_receive HAO→BAO"); break; }

            if (memcmp(buf, POLL_REQ, strlen(POLL_REQ)+1) == 0) {
                size_t base = strlen(POLL_REQ) + 1;
                int dlen = (int)(len - (ssize_t)base);

                clock_gettime(CLOCK_MONOTONIC, &now);
                printf("[BAO]: [%5ld.%09ld] From HAO: %s + payload(%d)\n",
                       now.tv_sec, now.tv_nsec, POLL_REQ, dlen);
                       
                uint8_t addr = (uint8_t)buf[base] & 0x7F;  /* read address */
                printf("[BAO]: Received address 0x%02X from HAO\n", addr);

                fflush(stdout);
                
                if (dlen < 1) {
                    fprintf(stderr, "[BAO]: no SPI address payload\n");
                    continue;
                }

                if (file_spi < 0) {
                    fprintf(stderr, "[BAO]: SPI not initialized (fd=%d)\n", file_spi);
                    continue;
                }

                uint8_t tx1[5] = { addr, 0,0,0,0 };
                uint8_t tx2[5] = { addr, 0,0,0,0 };
                uint8_t rx1[5] = { 0 };
                memset(rx2, 0, sizeof(rx2));

                /* First transfer: request read (discard reply) */
                struct spi_ioc_transfer tr1 = {0};
                tr1.tx_buf = (unsigned long)tx1;
                tr1.rx_buf = (unsigned long)rx1;
                tr1.len    = sizeof(tx1);
                tr1.speed_hz = spi_speed_hz;
                tr1.bits_per_word = spi_bits;

                /* Second transfer: returns the data */
                struct spi_ioc_transfer tr2 = {0};
                tr2.tx_buf = (unsigned long)tx2;
                tr2.rx_buf = (unsigned long)rx2;
                tr2.len    = sizeof(tx2);
                tr2.speed_hz = spi_speed_hz;
                tr2.bits_per_word = spi_bits;

                int ret;
                ret = ioctl(file_spi, SPI_IOC_MESSAGE(1), &tr1);
                if (ret < 1) perror("[BAO]: SPI_IOC_MESSAGE tr1");

                /* Allow CS to deassert between frames (separate call) */
                ret = ioctl(file_spi, SPI_IOC_MESSAGE(1), &tr2);
                if (ret < 1) perror("[BAO]: SPI_IOC_MESSAGE tr2");

                printf("[BAO]: SPI R %02X -> reply:", addr);
                for (int i = 0; i < 5; i++) printf(" %02X", rx2[i]);
                printf("\n");

                /* Reply: "POLL_RES:<n>\0" + 5 bytes */
                char hdr[MSG_SIZE];
                int hlen = snprintf(hdr, MSG_SIZE, POLL_RES ":%d", ++bao_counter) + 1;
                unsigned char msg[hlen + 5];
                memcpy(msg, hdr, hlen);
                memcpy(msg + hlen, rx2, 5);

                fprintf(stderr, "[BAO]: Sending to HAO → ");
                for (int i = 0; i < hlen + 5; i++) fprintf(stderr, "%02X ", ((unsigned char*)msg)[i]);
                fprintf(stderr, "\n");

                mq_send(mq_bao_hao, (char*)msg, hlen + 5, 0);
            }
        }
    }
    return NULL;
}

/* Open /dev/spidev<bus>.<cs> and configure mode/speed/bpw */
int tmc5130init(int bus, int cs)
{
    char dev[32];
    snprintf(dev, sizeof(dev), "/dev/spidev%d.%d", bus, cs);
    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        perror("tmc5130init: open spidev");
        return 0; /* keep your main() behavior (we'll avoid early exit by returning 1 on success) */
    }

    /* Configure SPI mode, bpw, speed */
    if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0) perror("SPI_IOC_WR_MODE");
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits) < 0) perror("SPI_IOC_WR_BITS_PER_WORD");
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_hz) < 0) perror("SPI_IOC_WR_MAX_SPEED_HZ");

    uint8_t rd_mode=0, rd_bits=0; uint32_t rd_speed=0;
    ioctl(fd, SPI_IOC_RD_MODE, &rd_mode);
    ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &rd_bits);
    ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &rd_speed);
    printf("SPI init OK: dev=%s mode=%u bpw=%u speed=%u Hz\n", dev, rd_mode, rd_bits, rd_speed);

    file_spi = fd;
    return 1; /* success */
}

int main(void) {
    if (tmc5130init(0, 0) == 0) { /* bus=0, cs=0 → /dev/spidev0.0 */
        // printf("TMC5130 failed to init.\n");
        // return 0;
    }

    pthread_t ts_tid, op_tid, ha_tid, bao_tid;
    signal(SIGINT, cleanup_and_exit);

    struct mq_attr attr = {
        .mq_flags   = 0,
        .mq_maxmsg  = 10,
        .mq_msgsize = MSG_SIZE,
        .mq_curmsgs = 0
    };
    struct mq_attr attr_raw = attr;
    attr_raw.mq_msgsize = MSG_SIZE + 8; 

    /* create all six queues (nonblocking) */
    mq_ts_opao  = mq_open(Q_TS_OPAO,  O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr);
    mq_ts_hao   = mq_open(Q_TS_HAO,   O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr);
    mq_opao_hao = mq_open(Q_OPAO_HAO, O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr);
    mq_hao_opao = mq_open(Q_HAO_OPAO, O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr);
    mq_hao_bao  = mq_open(Q_HAO_BAO,  O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr);
    mq_bao_hao  = mq_open(Q_BAO_HAO,  O_CREAT|O_RDWR|O_NONBLOCK, 0666, &attr_raw);

    if (mq_ts_opao==(mqd_t)-1 || mq_ts_hao==(mqd_t)-1 || mq_opao_hao==(mqd_t)-1 ||
        mq_hao_opao==(mqd_t)-1 || mq_hao_bao==(mqd_t)-1 || mq_bao_hao==(mqd_t)-1) {
        perror("mq_open");
        exit(EXIT_FAILURE);
    }

    pthread_create(&ts_tid,  NULL, time_sequencer, NULL);
    pthread_create(&op_tid,  NULL, opao,          NULL);
    pthread_create(&ha_tid,  NULL, hao,           NULL);
    pthread_create(&bao_tid, NULL, bao,           NULL);

    pthread_join(ts_tid,   NULL);
    pthread_join(op_tid,   NULL);
    pthread_join(ha_tid,   NULL);
    pthread_join(bao_tid,  NULL);

    cleanup_and_exit(0);
    return 0;
}
