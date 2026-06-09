/* GENERATED Teensy<->Pi host-bridge frame contract -- do not edit by hand. */
/* Produced by scripts/generate_ethercat_config.py (make config). */
#ifndef TEENSY_BRIDGE_LAYOUT_H
#define TEENSY_BRIDGE_LAYOUT_H

#define TEENSY_MAGIC 0xA7ECu
#define TEENSY_VERSION 1u
#define TEENSY_MOSI_LEN 27
#define TEENSY_MISO_LEN 59
#define TEENSY_FRAME_LEN 59
#define TEENSY_OUT_BYTES 16
#define TEENSY_IN_BYTES 39
#define TEENSY_MOSI_HDR 8
#define TEENSY_MISO_HDR 18
#define TEENSY_STREAM_OFF 24
#define TEENSY_SAMPLE_BYTES 0
#define TEENSY_SAMPLE_STRIDE 4
#define TEENSY_MAX_SAMPLES_PER_FRAME 0
#define TEENSY_DEFAULT_LEAD 0

/* MOSI flags / MISO status bits. */
#define TEENSY_FLAG_ENABLE      (1u<<0)
#define TEENSY_FLAG_FAULT_RESET (1u<<1)
#define TEENSY_FLAG_QUICK_STOP  (1u<<2)
#define TEENSY_ST_LINK          (1u<<0)
#define TEENSY_ST_OPERATIONAL   (1u<<1)
#define TEENSY_ST_FAULT         (1u<<2)
#define TEENSY_ST_HOST_TIMEOUT  (1u<<3)

/* One process-data pin: name, frame byte offset, bit pos/len, type, dir. */
typedef struct {
    const char *name;
    int frame_off;   /* byte offset within the MOSI (out) or MISO (in) frame */
    int bit_pos;     /* bit offset within frame_off (for 'b' pins) */
    int bit_len;
    char type;       /* 'b' bit, 'u' u32, 's' s32 */
    char dir;        /* 'o' output (host->drive), 'i' input (drive->host) */
} teensy_pin_t;

static const teensy_pin_t TEENSY_PINS[] = {
    { "drive0-controlword", 8, 0, 16, 'u', 'o' },
    { "drive0-target-position", 10, 0, 32, 's', 'o' },
    { "drive0-target-velocity", 14, 0, 32, 's', 'o' },
    { "drive0-touch-probe-function", 18, 0, 16, 'u', 'o' },
    { "drive0-digital-outputs", 20, 0, 32, 'u', 'o' },
    { "drive0-error-code", 18, 0, 16, 'u', 'i' },
    { "drive0-statusword", 20, 0, 16, 'u', 'i' },
    { "drive0-actual-position", 22, 0, 32, 's', 'i' },
    { "drive0-digital-inputs", 26, 0, 32, 'u', 'i' },
    { "drive0-actual-velocity", 30, 0, 32, 's', 'i' },
    { "drive0-follow-error", 34, 0, 32, 's', 'i' },
    { "drive0-touch-probe-status", 38, 0, 16, 'u', 'i' },
    { "drive0-touch-probe-pos1-positive", 40, 0, 32, 's', 'i' },
    { "drive0-touch-probe-pos1-negative", 44, 0, 32, 's', 'i' },
    { "drive0-touch-probe-pos2-positive", 48, 0, 32, 's', 'i' },
    { "drive0-touch-probe-pos2-negative", 52, 0, 32, 's', 'i' },
    { "drive0-op-mode-display", 56, 0, 8, 's', 'i' },
};
#define TEENSY_PIN_COUNT 17

/* Streamed motion fields: which output pin sources each sample slice. */
typedef struct {
    int sample_off;  /* byte offset within the motion sample payload */
    int pin_index;   /* index into TEENSY_PINS providing the value */
    int len;         /* byte length */
} teensy_motion_t;

static const teensy_motion_t TEENSY_MOTION[] = {
    { 0, -1, 0 } /* none */
};
#define TEENSY_MOTION_COUNT 0

#endif /* TEENSY_BRIDGE_LAYOUT_H */
