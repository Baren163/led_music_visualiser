#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "led_strip.h"

#include "renderer.h"
#include "storage.h"


#if CONFIG_FREERTOS_HZ < 1000
#warning "Set CONFIG_FREERTOS_HZ=1000 for accurate frame timing (see sdkconfig.defaults)"
#endif


// ============================================================
// GENERAL CONFIGURATION
// ============================================================

#define LED_GPIO            0
#define LED_COUNT           144
#define LED_HALF            (LED_COUNT / 2)

#define FRAME_PERIOD_US     (FRAME_PERIOD_MS * 1000)

#define RENDERER_TASK_PRIORITY  10      // highest of the application tasks
#define RENDERER_TASK_STACK     4096
#define RENDERER_QUEUE_LENGTH   4

// If the renderer falls more than this many frames behind the clock
// (e.g. a flash write stalled the CPU) it catches up without drawing
// the missed frames instead of racing through the backlog.
#define MAX_LATE_FRAMES     2

// Larger gaps than this are handled as a full seek.
#define MAX_CATCH_UP_FRAMES 50

// After a seek, the drums are rebuilt by replaying this many frames
// (2 s, matching led_strip_visualizer.py).
#define DRUM_REPLAY_FRAMES  (FRAME_RATE_HZ * 2)


static const char *TAG = "renderer";

static led_strip_handle_t led_strip;


// ============================================================
// LED COLOURS
// ============================================================

// Vocal and Other colours now come from the song file.

#define BASS_R      255
#define BASS_G      0
#define BASS_B      0

#define DRUM_R      255
#define DRUM_G      65
#define DRUM_B      0



// ============================================================
// COMPONENT BRIGHTNESS
// ============================================================
//
// Master brightness multipliers for each visual component.
// Range: 0-255
//
// 255 = full brightness
// 128 = about 50% brightness
//   0 = off
//
// These scale only the final LED brightness; they do not change
// beat detection, fades, note positions, or bass LED counts.

#define BASS_BRIGHTNESS      60
#define DRUM_BRIGHTNESS      60
#define OTHER_BRIGHTNESS     120
#define VOCAL_BRIGHTNESS     30


// ============================================================
// IDLE / TRANSFER ANIMATIONS
// ============================================================

// Idle: slow warm wave travelling along the strip (mirrored).
#define IDLE_R               200
#define IDLE_G               110
#define IDLE_B               0
#define IDLE_BRIGHTNESS      40
#define IDLE_WAVES           2.0f       // wave crests per half strip
#define IDLE_SPEED           1.2f       // radians per second

// Transfer: progress bar filling from both ends toward the centre.
#define TRANSFER_R           180
#define TRANSFER_G           220
#define TRANSFER_B           0
#define TRANSFER_BRIGHTNESS  40

#define TWO_PI               6.2831853f


// ============================================================
// LED REGIONS (not mirrored)
//
//  0  - 17     Bass
//  18 - 26     Kick
//  27 - 35     Hi-hat
//  36          Gap
//  37 - 84     Other (48 LEDs, note position p -> LED 37 + p)
//  85          Gap
//  86 - 143    Vocals
// ============================================================

#define BASS_START          0
#define BASS_END            17

#define KICK_START          18
#define HIHAT_START         27
#define DRUM_BLOCK_LENGTH   9

#define OTHER_START         37
#define OTHER_LED_COUNT     48

#define VOCAL_START         86
#define VOCAL_END           143

#define VOCALS_VOLUME_LOWER_THRESHOLD 153
#define VOCALS_VOLUME_UPPER_THRESHOLD 230


// ============================================================
// DRUM FADE CONFIGURATION
// ============================================================

// Amount removed from brightness every 40 ms frame.
//
// Higher = faster fade.
// Lower  = slower fade.

#define KICK_FADE_AMOUNT       35
#define HIHAT_FADE_AMOUNT      50

// Drum spatial brightness profile across each 9-LED block.
// Values are 0-255 multipliers applied to the current beat brightness.
// The centre stays bright while the three LEDs toward each edge taper off.
static const uint8_t DRUM_LED_PROFILE[DRUM_BLOCK_LENGTH] = {
    0, 64, 140, 255, 255, 255, 140, 64, 0
};


// ============================================================
// OTHER NOTE STATE
// ============================================================

// Other brightness is 0-255 scaled by 256 so slow fades stay smooth.
#define OTHER_FULL_LEVEL       (255 << 8)


// ============================================================
// SONG FILE RECORDS (see protocol.h for the container layout)
// ============================================================

typedef struct __attribute__((packed))
{
    uint8_t bass;
    uint8_t drums;
    uint16_t vocal_colour;      // RGB555
    uint8_t vocal_volume;
    uint8_t vocal_onset;

} song_frame_t;


typedef struct __attribute__((packed))
{
    uint16_t frame;
    uint8_t position;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t fade;

} song_note_t;


_Static_assert(sizeof(song_frame_t) == SONG_FRAME_SIZE, "song_frame_t must be 6 bytes");
_Static_assert(sizeof(song_note_t) == SONG_NOTE_SIZE, "song_note_t must be 7 bytes");


// ============================================================
// DRUM DATA
// ============================================================

typedef struct
{
    bool kick;
    bool hihat;

    uint8_t kick_intensity;
    uint8_t hihat_intensity;

} drum_data_t;


// Current displayed drum brightness.
//
// These persist between frames so that a beat can fade
// gradually rather than immediately switching off.

static uint8_t kick_brightness = 0;
static uint8_t hihat_brightness = 0;


// ============================================================
// LED FRAMEBUFFER
// ============================================================

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;

} rgb_t;


static rgb_t led_buffer[LED_COUNT];


// Per-LED Other state. Each LED keeps the colour and fade of the
// last note that lit it.
static uint16_t other_level[OTHER_LED_COUNT];
static rgb_t other_colour[OTHER_LED_COUNT];
static uint8_t other_fade[OTHER_LED_COUNT];


// ============================================================
// RENDER TASK STATE
// ============================================================

typedef enum
{
    RCMD_OPEN,
    RCMD_SEEK,
    RCMD_STILL,
    RCMD_PAUSE,
    RCMD_RESUME,
    RCMD_IDLE,
    RCMD_OFF,
    RCMD_TRANSFER,

} rcmd_type_t;


typedef struct
{
    rcmd_type_t type;
    uint32_t frame;
    bool playing;
    char track_id[TRACK_ID_LEN];

} rcmd_t;


static QueueHandle_t s_cmd_queue;
static SemaphoreHandle_t s_cmd_done;
static bool s_cmd_result;

static volatile renderer_mode_t s_mode = RENDERER_OFF;

// Open song. Frames are streamed from the file; notes live in RAM.
static FILE *s_file = NULL;
static uint32_t s_total_frames = 0;
static uint8_t *s_notes = NULL;
static uint32_t s_note_count = 0;
static uint32_t s_next_note = 0;            // first note not yet applied
static volatile uint32_t s_frame = 0;
static char s_loaded_track[TRACK_ID_LEN];

// Timing anchor: s_anchor_frame is due at s_anchor_time_us.
static int64_t s_anchor_time_us = 0;
static uint32_t s_anchor_frame = 0;

// Idle / transfer animation timing.
static int64_t s_anim_start_us = 0;
static uint32_t s_anim_frame = 0;
static volatile uint8_t s_progress = 0;


// ============================================================
// LED INITIALISATION
// ============================================================

static void configure_led(void)
{
    ESP_LOGI(
        TAG,
        "Configuring %d WS2812 LEDs on GPIO %d",
        LED_COUNT,
        LED_GPIO
    );

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format =
            LED_STRIP_COLOR_COMPONENT_FMT_GRB,

        .flags = {
            .invert_out = false,
        },
    };


    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,

        .flags = {
            .with_dma = false,
        },
    };


    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(
            &strip_config,
            &rmt_config,
            &led_strip
        )
    );


    ESP_ERROR_CHECK(
        led_strip_clear(led_strip)
    );
}


// ============================================================
// BUFFER HELPERS
// ============================================================

static void clear_led_buffer(void)
{
    memset(
        led_buffer,
        0,
        sizeof(led_buffer)
    );
}


// ------------------------------------------------------------
// Directly sets one physical LED.
// ------------------------------------------------------------

static void set_physical_led(
    int index,
    uint8_t r,
    uint8_t g,
    uint8_t b
)
{
    if (index < 0 || index >= LED_COUNT)
        return;


    led_buffer[index].r = r;
    led_buffer[index].g = g;
    led_buffer[index].b = b;
}


// ------------------------------------------------------------
// Set an LED on the first half AND its mirrored LED.
// Only used by the idle and transfer animations; song frames
// are not mirrored.
// ------------------------------------------------------------

static void set_mirrored_led(
    int first_half_index,
    uint8_t r,
    uint8_t g,
    uint8_t b
)
{
    int mirrored_index =
        (LED_COUNT - 1) - first_half_index;


    set_physical_led(
        first_half_index,
        r,
        g,
        b
    );


    set_physical_led(
        mirrored_index,
        r,
        g,
        b
    );
}


// ============================================================
// SEND COMPLETE BUFFER TO LED STRIP
// ============================================================

static void send_led_buffer(void)
{
    for (int i = 0; i < LED_COUNT; i++)
    {
        ESP_ERROR_CHECK(
            led_strip_set_pixel(
                led_strip,
                i,
                led_buffer[i].r,
                led_buffer[i].g,
                led_buffer[i].b
            )
        );
    }


    ESP_ERROR_CHECK(
        led_strip_refresh(led_strip)
    );
}


static void clear_strip(void)
{
    clear_led_buffer();
    send_led_buffer();
}


// ============================================================
// DRUM DECODING
// ============================================================

static drum_data_t decode_drums(uint8_t drums)
{
    drum_data_t result;


    // bit 0
    result.kick =
        (drums & 0x01) != 0;


    // bit 1
    result.hihat =
        (drums & 0x02) != 0;


    // bits 2,3,4
    result.kick_intensity =
        (drums >> 2) & 0x07;


    // bits 5,6,7
    result.hihat_intensity =
        (drums >> 5) & 0x07;


    return result;
}


// ============================================================
// BASS
// ============================================================
//
// Bass value determines NUMBER of illuminated LEDs, filling
// upward from LED 0.
//
// bass = 0
//
//  [..................]
//
// bass ≈ 128
//
//  [#########.........]
//
// bass = 255
//
//  [##################]
// ============================================================

static void render_bass(uint8_t bass)
{
    const int region_length =
        BASS_END - BASS_START + 1;


    int leds_on =
        ((int)bass * region_length) / 255;


    for (int i = 0; i < leds_on; i++)
    {
        set_physical_led(
            BASS_START + i,
            ((uint16_t)BASS_R * BASS_BRIGHTNESS) / 255,
            ((uint16_t)BASS_G * BASS_BRIGHTNESS) / 255,
            ((uint16_t)BASS_B * BASS_BRIGHTNESS) / 255
        );
    }
}


// ============================================================
// DRUM FADE HELPER
// ============================================================

static uint8_t fade_value(
    uint8_t value,
    uint8_t fade_amount
)
{
    if (value <= fade_amount)
        return 0;


    return value - fade_amount;
}


// ============================================================
// DRUMS
// ============================================================
//
// Kick:   LEDs 18-26 (9 LEDs)
// Hi-hat: LEDs 27-35 (9 LEDs)
//
// Beat intensity determines the centre brightness. Each block uses
// DRUM_LED_PROFILE so its edges taper toward black instead of every
// LED flashing at exactly the same brightness.
//
// After a beat, brightness decreases every frame.
// ============================================================

// Advances the drum fade state by one frame without drawing.
static void update_drums(drum_data_t drums)
{
    // Fade previous beats.
    kick_brightness =
        fade_value(
            kick_brightness,
            KICK_FADE_AMOUNT
        );

    hihat_brightness =
        fade_value(
            hihat_brightness,
            HIHAT_FADE_AMOUNT
        );

    // New kick. Intensity 0-7 -> brightness 0-255.
    if (drums.kick)
    {
        uint8_t new_brightness =
            (drums.kick_intensity * 255) / 7;

        if (new_brightness > kick_brightness)
            kick_brightness = new_brightness;
    }

    // New hi-hat.
    if (drums.hihat)
    {
        uint8_t new_brightness =
            (drums.hihat_intensity * 255) / 7;

        if (new_brightness > hihat_brightness)
            hihat_brightness = new_brightness;
    }
}


static void render_drum_block(
    int region_start,
    uint8_t brightness
)
{
    for (int offset = 0; offset < DRUM_BLOCK_LENGTH; offset++)
    {
        uint8_t spatial_level =
            ((uint32_t)brightness *
             DRUM_LED_PROFILE[offset] *
             DRUM_BRIGHTNESS) / (255u * 255u);

        uint8_t r =
            ((uint16_t)DRUM_R * spatial_level) / 255;

        uint8_t g =
            ((uint16_t)DRUM_G * spatial_level) / 255;

        uint8_t b =
            ((uint16_t)DRUM_B * spatial_level) / 255;

        set_physical_led(
            region_start + offset,
            r,
            g,
            b
        );
    }
}


static void render_drums(void)
{
    render_drum_block(
        KICK_START,
        kick_brightness
    );

    render_drum_block(
        HIHAT_START,
        hihat_brightness
    );
}


// ============================================================
// OTHER NOTE EVENTS
// ============================================================
//
// Notes are a separate list sorted by frame, loaded into RAM when
// a song opens. s_next_note points at the first note not yet
// applied, so each frame only compares one note's frame with the
// current frame.
//
// Each of the 48 Other LEDs keeps its own level, colour and fade.
// Every frame, in this order:
//   1. Fade:  level = level * (255 - fade) / 255
//   2. Notes: a note sets its LED to full level, its colour and fade
//   3. Draw:  LED 37 + p = colour * level, times OTHER_BRIGHTNESS
// fade 0 holds an LED until another note replaces it.
// ============================================================

// 7-byte records are not aligned, so copy them out.
static void read_note(uint32_t index, song_note_t *note)
{
    memcpy(note, s_notes + index * SONG_NOTE_SIZE, SONG_NOTE_SIZE);
}


static uint16_t note_frame(uint32_t index)
{
    uint16_t frame;
    memcpy(&frame, s_notes + index * SONG_NOTE_SIZE, sizeof(frame));
    return frame;
}


// Index of the first note with frame >= `frame` (binary search).
static uint32_t first_note_from(uint32_t frame)
{
    uint32_t low = 0;
    uint32_t high = s_note_count;

    while (low < high)
    {
        uint32_t mid = low + (high - low) / 2;

        if (note_frame(mid) < frame)
            low = mid + 1;
        else
            high = mid;
    }

    return low;
}


static void reset_other_state(void)
{
    memset(other_level, 0, sizeof(other_level));
    memset(other_colour, 0, sizeof(other_colour));
    memset(other_fade, 0, sizeof(other_fade));
}


// Advances the Other state to `frame_index` without drawing.
static void update_other(uint32_t frame_index)
{
    // 1. Fade every lit LED by its own fade value.
    for (int i = 0; i < OTHER_LED_COUNT; i++)
    {
        if (other_level[i] > 0)
        {
            other_level[i] =
                ((uint32_t)other_level[i] * (255 - other_fade[i])) / 255;
        }
    }

    // 2. Start this frame's notes at full level. Notes for frames
    // already passed (should not happen) are skipped, not applied.
    while (s_next_note < s_note_count &&
           note_frame(s_next_note) <= frame_index)
    {
        song_note_t note;
        read_note(s_next_note, &note);

        if (note.frame == frame_index && note.position < OTHER_LED_COUNT)
        {
            other_level[note.position] = OTHER_FULL_LEVEL;
            other_colour[note.position] = (rgb_t){ note.r, note.g, note.b };
            other_fade[note.position] = note.fade;
        }

        s_next_note++;
    }
}


static void render_other(void)
{
    for (int i = 0; i < OTHER_LED_COUNT; i++)
    {
        uint8_t level =
            ((uint32_t)(other_level[i] >> 8) * OTHER_BRIGHTNESS) / 255;

        set_physical_led(
            OTHER_START + i,
            ((uint16_t)other_colour[i].r * level) / 255,
            ((uint16_t)other_colour[i].g * level) / 255,
            ((uint16_t)other_colour[i].b * level) / 255
        );
    }
}


// Rebuilds the Other LEDs as they would be just before `frame`:
// for each LED, take the last note before `frame` and apply the
// fades of every frame since it.
static void restore_other_state(uint32_t frame)
{
    reset_other_state();

    s_next_note = first_note_from(frame);

    int32_t last_note[OTHER_LED_COUNT];

    for (int i = 0; i < OTHER_LED_COUNT; i++)
        last_note[i] = -1;

    for (uint32_t k = 0; k < s_next_note; k++)
    {
        song_note_t note;
        read_note(k, &note);

        if (note.position < OTHER_LED_COUNT)
            last_note[note.position] = (int32_t)k;
    }

    for (int i = 0; i < OTHER_LED_COUNT; i++)
    {
        if (last_note[i] < 0)
            continue;

        song_note_t note;
        read_note((uint32_t)last_note[i], &note);

        // Fades are applied on the frames after the note, up to and
        // including frame - 1. Frame `frame` fades when it is drawn.
        uint32_t age = (frame - 1) - note.frame;
        float factor = powf((255 - note.fade) / 255.0f, (float)age);

        other_level[i] = (uint16_t)(OTHER_FULL_LEVEL * factor);
        other_colour[i] = (rgb_t){ note.r, note.g, note.b };
        other_fade[i] = note.fade;
    }
}


// ============================================================
// VOCALS
// ============================================================
//
// The volume window [LOWER, UPPER] maps onto all 58 vocal LEDs,
// filling upward from LED 86 in the frame's vocal colour.
// ============================================================

static uint8_t rgb555_channel(uint16_t bits, int shift)
{
    uint16_t value = (bits >> shift) & 0x1F;
    return (uint8_t)((value * 255 + 15) / 31);
}


static void render_vocals(
    uint16_t colour,
    uint8_t volume
)
{
    const int vocals_length = VOCAL_END - VOCAL_START + 1;

    int leds_on = (int)((float)(volume - VOCALS_VOLUME_LOWER_THRESHOLD) / (VOCALS_VOLUME_UPPER_THRESHOLD - VOCALS_VOLUME_LOWER_THRESHOLD) * vocals_length);
    if (leds_on < 0) leds_on = 0;
    if (leds_on > vocals_length) leds_on = vocals_length;

    uint8_t r = ((uint16_t)rgb555_channel(colour, 0) * VOCAL_BRIGHTNESS) / 255;
    uint8_t g = ((uint16_t)rgb555_channel(colour, 5) * VOCAL_BRIGHTNESS) / 255;
    uint8_t b = ((uint16_t)rgb555_channel(colour, 10) * VOCAL_BRIGHTNESS) / 255;

    for (int i = 0; i < leds_on; i++)
    {
        set_physical_led(
            VOCAL_START + i,
            r,
            g,
            b
        );
    }
}


// ============================================================
// RENDER ONE COMPLETE SONG FRAME
// ============================================================

static void reset_visual_state(void)
{
    kick_brightness = 0;
    hihat_brightness = 0;

    reset_other_state();
}


// Advances drums and Other to `frame_index` without drawing.
static void update_state(
    const song_frame_t *frame,
    uint32_t frame_index
)
{
    update_drums(
        decode_drums(frame->drums)
    );

    update_other(
        frame_index
    );
}


static void render_frame(
    const song_frame_t *frame,
    uint32_t frame_index
)
{
    update_state(
        frame,
        frame_index
    );


    clear_led_buffer();


    render_bass(
        frame->bass
    );


    render_drums();


    render_other();


    render_vocals(
        frame->vocal_colour,
        frame->vocal_volume
    );


    send_led_buffer();
}


// ============================================================
// IDLE AND TRANSFER ANIMATIONS
// ============================================================

static void render_idle_animation(int64_t elapsed_us)
{
    clear_led_buffer();

    float t = (float)elapsed_us / 1000000.0f;

    for (int i = 0; i < LED_HALF; i++)
    {
        float phase =
            ((float)i / LED_HALF) * TWO_PI * IDLE_WAVES -
            t * IDLE_SPEED;

        // Sharpen the sine so the crests read as soft blobs.
        float v = 0.5f + 0.5f * sinf(phase);
        v = v * v * v;

        uint8_t level = (uint8_t)(v * IDLE_BRIGHTNESS);

        set_mirrored_led(
            i,
            ((uint16_t)IDLE_R * level) / 255,
            ((uint16_t)IDLE_G * level) / 255,
            ((uint16_t)IDLE_B * level) / 255
        );
    }

    send_led_buffer();
}


static void render_transfer_animation(int64_t elapsed_us)
{
    clear_led_buffer();

    int filled = ((int)s_progress * LED_HALF) / 100;

    if (filled > LED_HALF)
        filled = LED_HALF;

    for (int i = 0; i < filled; i++)
    {
        set_mirrored_led(
            i,
            ((uint16_t)TRANSFER_R * TRANSFER_BRIGHTNESS) / 255,
            ((uint16_t)TRANSFER_G * TRANSFER_BRIGHTNESS) / 255,
            ((uint16_t)TRANSFER_B * TRANSFER_BRIGHTNESS) / 255
        );
    }

    // Pulsing head so a stalled transfer is still visibly "alive".
    if (filled < LED_HALF)
    {
        float t = (float)elapsed_us / 1000000.0f;
        float pulse = 0.5f + 0.5f * sinf(t * TWO_PI * 1.5f);
        uint8_t level = (uint8_t)(pulse * TRANSFER_BRIGHTNESS);

        set_mirrored_led(
            filled,
            ((uint16_t)TRANSFER_R * level) / 255,
            ((uint16_t)TRANSFER_G * level) / 255,
            ((uint16_t)TRANSFER_B * level) / 255
        );
    }

    send_led_buffer();
}


// ============================================================
// SONG FILE ACCESS
// ============================================================

static bool seek_file_to_frame(uint32_t frame)
{
    long offset =
        (long)sizeof(song_header_t) +
        (long)frame * SONG_FRAME_SIZE;

    return fseek(s_file, offset, SEEK_SET) == 0;
}


static bool read_frame(song_frame_t *frame)
{
    return fread(frame, sizeof(*frame), 1, s_file) == 1;
}


static void close_file(void)
{
    if (s_file != NULL)
    {
        fclose(s_file);
        s_file = NULL;
    }

    free(s_notes);
    s_notes = NULL;

    memset(s_loaded_track, 0, sizeof(s_loaded_track));
    s_total_frames = 0;
    s_note_count = 0;
    s_next_note = 0;
    s_frame = 0;
}


static void start_animation(renderer_mode_t mode)
{
    s_mode = mode;
    s_anim_start_us = esp_timer_get_time();
    s_anim_frame = 0;
}


// Restores all fade state to what it would be just before `frame`
// and positions the file there.
static bool restore_state(uint32_t frame)
{
    reset_visual_state();

    // Drums: replay the preceding frames without drawing.
    uint32_t start = (frame > DRUM_REPLAY_FRAMES) ? frame - DRUM_REPLAY_FRAMES : 0;

    if (start < frame && seek_file_to_frame(start))
    {
        song_frame_t replay;

        for (uint32_t i = start; i < frame && read_frame(&replay); i++)
        {
            update_drums(
                decode_drums(replay.drums)
            );
        }
    }

    // Other: rebuild each LED from its last note.
    restore_other_state(frame);

    return seek_file_to_frame(frame);
}


static void end_song(void)
{
    s_frame = s_total_frames;
    s_mode = RENDERER_ENDED;
    clear_strip();
}


// Moves playback to `frame` with the fade state rebuilt. When not
// playing, `frame` is drawn once and frozen on the strip.
static void position_playback(uint32_t frame, bool playing)
{
    if (frame >= s_total_frames || !restore_state(frame))
    {
        end_song();
        return;
    }

    s_frame = frame;

    if (playing)
    {
        s_anchor_frame = frame;
        s_anchor_time_us = esp_timer_get_time();
        s_mode = RENDERER_PLAYING;
        return;
    }

    song_frame_t still;

    if (read_frame(&still))
        render_frame(&still, frame);
    else
        clear_strip();

    // The file and state are now one frame ahead, but resuming
    // always restores state first, so that does not matter.
    s_mode = RENDERER_PAUSED;
}


static bool open_track(const rcmd_t *cmd)
{
    close_file();

    char path[48];
    storage_track_path(cmd->track_id, path, sizeof(path));

    s_file = fopen(path, "rb");

    if (s_file == NULL)
    {
        ESP_LOGE(TAG, "Could not open %s", path);
        goto fail;
    }

    fseek(s_file, 0, SEEK_END);
    long file_size = ftell(s_file);
    rewind(s_file);

    song_header_t header;

    if (fread(&header, sizeof(header), 1, s_file) != 1 ||
        !song_header_is_valid(&header, (uint32_t)file_size))
    {
        ESP_LOGE(TAG, "%s is not a valid song file", path);
        goto fail;
    }

    s_total_frames = header.frame_count;
    s_note_count = header.note_count;

    // Load the notes into RAM (a few KB per song).
    if (s_note_count > 0)
    {
        size_t notes_size = s_note_count * SONG_NOTE_SIZE;

        s_notes = malloc(notes_size);

        if (s_notes == NULL)
        {
            ESP_LOGE(TAG, "Not enough memory for %lu notes", (unsigned long)s_note_count);
            goto fail;
        }

        if (!seek_file_to_frame(s_total_frames) ||
            fread(s_notes, 1, notes_size, s_file) != notes_size)
        {
            ESP_LOGE(TAG, "Could not read notes from %s", path);
            goto fail;
        }
    }

    memcpy(s_loaded_track, cmd->track_id, TRACK_ID_LEN);

    ESP_LOGI(
        TAG,
        "Opened %.22s: %lu frames, %lu notes, starting at frame %lu (%s)",
        cmd->track_id,
        (unsigned long)s_total_frames,
        (unsigned long)s_note_count,
        (unsigned long)cmd->frame,
        cmd->playing ? "playing" : "paused"
    );

    position_playback(cmd->frame, cmd->playing);
    return true;

fail:
    close_file();
    s_mode = RENDERER_OFF;
    clear_strip();
    return false;
}


// ============================================================
// COMMAND HANDLING
// ============================================================

static bool handle_command(const rcmd_t *cmd)
{
    switch (cmd->type)
    {
    case RCMD_OPEN:
        return open_track(cmd);

    case RCMD_SEEK:
    case RCMD_RESUME:
        if (s_file == NULL)
            return false;

        position_playback(cmd->frame, true);
        return true;

    case RCMD_STILL:
        if (s_file == NULL)
            return false;

        position_playback(cmd->frame, false);
        return true;

    case RCMD_PAUSE:
        // Freeze: the last frame stays on the strip.
        if (s_mode == RENDERER_PLAYING)
            s_mode = RENDERER_PAUSED;
        return true;

    case RCMD_IDLE:
        if (s_mode != RENDERER_IDLE)
        {
            close_file();
            start_animation(RENDERER_IDLE);
        }
        return true;

    case RCMD_TRANSFER:
        close_file();
        s_progress = 0;
        start_animation(RENDERER_TRANSFER);
        return true;

    case RCMD_OFF:
        close_file();
        s_mode = RENDERER_OFF;
        clear_strip();
        return true;
    }

    return false;
}


// ============================================================
// RENDER TASK
// ============================================================

// Absolute time of the next piece of work, or -1 if there is none.
static int64_t next_deadline_us(void)
{
    switch (s_mode)
    {
    case RENDERER_PLAYING:
        return s_anchor_time_us +
               (int64_t)(s_frame - s_anchor_frame) * FRAME_PERIOD_US;

    case RENDERER_IDLE:
    case RENDERER_TRANSFER:
        return s_anim_start_us +
               (int64_t)s_anim_frame * FRAME_PERIOD_US;

    default:
        return -1;
    }
}


static void play_next_frame(void)
{
    int64_t now = esp_timer_get_time();

    uint32_t due =
        s_anchor_frame +
        (uint32_t)((now - s_anchor_time_us) / FRAME_PERIOD_US);

    song_frame_t frame;

    // --------------------------------------------------------
    // Fell behind: catch up without drawing the missed frames.
    // --------------------------------------------------------

    if (due > s_frame + MAX_LATE_FRAMES)
    {
        ESP_LOGW(
            TAG,
            "Renderer %lu frames late, catching up",
            (unsigned long)(due - s_frame)
        );

        if (due >= s_total_frames)
        {
            end_song();
            return;
        }

        if (due - s_frame <= MAX_CATCH_UP_FRAMES)
        {
            // Step the fade state through the missed frames so held
            // notes and fades stay exact.
            for (; s_frame < due; s_frame++)
            {
                if (!read_frame(&frame))
                {
                    end_song();
                    return;
                }

                update_state(&frame, s_frame);
            }
        }
        else
        {
            if (!restore_state(due))
            {
                end_song();
                return;
            }

            s_frame = due;
        }
    }

    if (!read_frame(&frame))
    {
        ESP_LOGI(TAG, "Visualisation finished");
        end_song();
        return;
    }

    render_frame(&frame, s_frame);
    s_frame++;
}


static void play_next_animation_frame(void)
{
    int64_t elapsed = esp_timer_get_time() - s_anim_start_us;

    if (s_mode == RENDERER_IDLE)
        render_idle_animation(elapsed);
    else
        render_transfer_animation(elapsed);

    s_anim_frame++;

    // After a long stall (flash erase) resynchronise instead of
    // rendering a burst of catch-up frames.
    uint32_t due = (uint32_t)(elapsed / FRAME_PERIOD_US);

    if (due > s_anim_frame + MAX_LATE_FRAMES)
        s_anim_frame = due + 1;
}


static void renderer_task(void *arg)
{
    rcmd_t cmd;

    for (;;)
    {
        // ----------------------------------------------------
        // Sleep on the command queue until the next frame is due.
        // ----------------------------------------------------

        int64_t deadline = next_deadline_us();
        TickType_t wait_ticks;

        if (deadline < 0)
        {
            wait_ticks = portMAX_DELAY;
        }
        else
        {
            int64_t remaining = deadline - esp_timer_get_time();

            wait_ticks = (remaining > 0)
                ? (TickType_t)((remaining * configTICK_RATE_HZ) / 1000000)
                : 0;
        }

        if (xQueueReceive(s_cmd_queue, &cmd, wait_ticks) == pdTRUE)
        {
            s_cmd_result = handle_command(&cmd);
            xSemaphoreGive(s_cmd_done);
            continue;
        }

        if (deadline < 0)
            continue;

        // ----------------------------------------------------
        // Less than one tick left: spin for the remainder.
        // ----------------------------------------------------

        while (esp_timer_get_time() < deadline)
        {
        }

        if (s_mode == RENDERER_PLAYING)
            play_next_frame();
        else
            play_next_animation_frame();
    }
}


// ============================================================
// PUBLIC API
// ============================================================

static bool send_command(const rcmd_t *cmd)
{
    xQueueSend(s_cmd_queue, cmd, portMAX_DELAY);

    if (xSemaphoreTake(s_cmd_done, pdMS_TO_TICKS(2000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Render task did not acknowledge command %d", cmd->type);
        return false;
    }

    return s_cmd_result;
}


void renderer_init(void)
{
    configure_led();

    s_cmd_queue = xQueueCreate(RENDERER_QUEUE_LENGTH, sizeof(rcmd_t));
    s_cmd_done = xSemaphoreCreateBinary();

    configASSERT(s_cmd_queue != NULL);
    configASSERT(s_cmd_done != NULL);

    xTaskCreate(
        renderer_task,
        "renderer",
        RENDERER_TASK_STACK,
        NULL,
        RENDERER_TASK_PRIORITY,
        NULL
    );
}


bool renderer_open(const char *track_id, uint32_t start_frame, bool playing)
{
    rcmd_t cmd = {
        .type = RCMD_OPEN,
        .frame = start_frame,
        .playing = playing,
    };

    memcpy(cmd.track_id, track_id, TRACK_ID_LEN);

    return send_command(&cmd);
}


void renderer_seek(uint32_t frame)
{
    rcmd_t cmd = { .type = RCMD_SEEK, .frame = frame };
    send_command(&cmd);
}


void renderer_show_still(uint32_t frame)
{
    rcmd_t cmd = { .type = RCMD_STILL, .frame = frame };
    send_command(&cmd);
}


void renderer_pause(void)
{
    rcmd_t cmd = { .type = RCMD_PAUSE };
    send_command(&cmd);
}


void renderer_resume(uint32_t frame)
{
    rcmd_t cmd = { .type = RCMD_RESUME, .frame = frame };
    send_command(&cmd);
}


void renderer_idle(void)
{
    rcmd_t cmd = { .type = RCMD_IDLE };
    send_command(&cmd);
}


void renderer_off(void)
{
    rcmd_t cmd = { .type = RCMD_OFF };
    send_command(&cmd);
}


void renderer_show_transfer(void)
{
    rcmd_t cmd = { .type = RCMD_TRANSFER };
    send_command(&cmd);
}


void renderer_set_progress(uint8_t percent)
{
    s_progress = (percent > 100) ? 100 : percent;
}


renderer_mode_t renderer_get_mode(void)
{
    return s_mode;
}


uint32_t renderer_get_frame(void)
{
    return s_frame;
}


void renderer_get_loaded_track(char out[TRACK_ID_LEN])
{
    // Only modified by the render task while it processes a command,
    // and commands are synchronous, so this is stable for the caller.
    memcpy(out, s_loaded_track, TRACK_ID_LEN);
}
