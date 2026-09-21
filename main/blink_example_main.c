#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_spiffs.h"

#include "led_strip.h"


// ============================================================
// GENERAL CONFIGURATION
// ============================================================

#define LED_GPIO            0
#define LED_COUNT           144

#define FRAME_RATE_HZ       25
#define FRAME_PERIOD_US     (1000000 / FRAME_RATE_HZ)

#define STORAGE_PATH        "/storage"
#define STORAGE_PARTITION   "storage"
#define SONG_PATH           "/storage/song.bin"


static const char *TAG = "audio_visualiser";

static led_strip_handle_t led_strip;


// ============================================================
// LED COLOURS
// ============================================================

// These can be adjusted later.

#define BASS_R      255
#define BASS_G      0
#define BASS_B      0

#define DRUM_R      255
#define DRUM_G      65
#define DRUM_B      0

#define OTHER_R     200
#define OTHER_G     110
#define OTHER_B     0

#define VOCAL_R     180
#define VOCAL_G     220
#define VOCAL_B     0



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
// LED REGIONS
//
// FIRST HALF:
//  0  - 17     Bass
//  18 - 26     Kick
//  27 - 35     Hi-hat
//  36 - 37     Gap before Other
//  38 - 57     Other main note-position region
//  58 - 59     Gap after Other
//  60 - 71     Vocal spectrum
//
// SECOND HALF:
// mirrored automatically
// ============================================================

#define BASS_START          0
#define BASS_END            17

#define KICK_START          18
#define KICK_END            26

#define HIHAT_START         27
#define HIHAT_END           35

#define OTHER_MAIN_START    38
#define OTHER_MAIN_END      57

// The Other glow is allowed to occupy one LED beyond the main region on
// each side. This uses one LED from each 2-LED positional gap, leaving
// one physically dark LED between Other and the neighbouring component.
#define OTHER_RENDER_START  (OTHER_MAIN_START - 1)
#define OTHER_RENDER_END    (OTHER_MAIN_END + 1)
#define OTHER_RENDER_LENGTH (OTHER_RENDER_END - OTHER_RENDER_START + 1)
#define OTHER_MAIN_LENGTH   (OTHER_MAIN_END - OTHER_MAIN_START + 1)

#define VOCAL_START         60
#define VOCAL_END           71

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
static const uint8_t DRUM_LED_PROFILE[9] = {
    0, 64, 140, 255, 255, 255, 140, 64, 0
};

// 'Other' note fade. Amount removed from every active other LED per frame.
#define OTHER_FADE_AMOUNT      45

// Adjacent LEDs receive this fraction of the main note brightness.
// 64 / 255 ~= 25%.
#define OTHER_ADJACENT_SCALE   64


// ============================================================
// BINARY FILE FORMAT
//
// 14 bytes per frame:
//
// byte 0        bass
// byte 1        drums
// byte 2        other brightness (0-255)
// byte 3        other metadata:
//                 bit 0      note detected
//                 bits 1-4   LED position (0-13)
//                 bits 5-7   reserved
// bytes 4-11   vocals_spectrum[8]
// byte 12       vocals_volume
// byte 13       vocals_onset
// ============================================================

typedef struct
{
    uint8_t bass;
    uint8_t drums;

    uint8_t other_brightness;
    uint8_t other_metadata;

    uint8_t vocals_spectrum[8];
    uint8_t vocals_volume;
    uint8_t vocals_onset;

} audio_frame_t;


_Static_assert(
    sizeof(audio_frame_t) == 14,
    "audio_frame_t must be exactly 14 bytes"
);


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


// Persistent brightness for the Other render region, including one glow LED
// into each reserved positional gap. This lets previous notes fade while new
// notes can appear elsewhere without touching drums or vocals.
static uint8_t other_led_brightness[OTHER_RENDER_LENGTH] = {0};


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
// SPIFFS STORAGE
// ============================================================

static void configure_storage(void)
{
    esp_vfs_spiffs_conf_t config = {
        .base_path = STORAGE_PATH,
        .partition_label = STORAGE_PARTITION,
        .max_files = 4,
        .format_if_mount_failed = false,
    };


    esp_err_t result =
        esp_vfs_spiffs_register(&config);


    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to mount SPIFFS: %s",
            esp_err_to_name(result)
        );

        return;
    }


    size_t total = 0;
    size_t used = 0;


    ESP_ERROR_CHECK(
        esp_spiffs_info(
            STORAGE_PARTITION,
            &total,
            &used
        )
    );


    ESP_LOGI(
        TAG,
        "SPIFFS mounted: %u / %u bytes used",
        (unsigned int)used,
        (unsigned int)total
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
//
// Example:
//
// LED 0  -> also LED 143
// LED 17 -> also LED 126
// LED 71 -> also LED 72
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
// Bass value determines NUMBER of illuminated LEDs.
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
//
// This is mirrored on the second half.
// ============================================================

static void render_bass(uint8_t bass)
{
    const int region_length =
        BASS_END - BASS_START + 1;


    int leds_on =
        ((int)bass * region_length) / 255;


    // Make 255 definitely produce all 18 LEDs.

    if (bass == 255)
        leds_on = region_length;


    for (int i = 0; i < leds_on; i++)
    {
        set_mirrored_led(
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

static void render_drum_block(
    int region_start,
    uint8_t brightness
)
{
    for (int offset = 0; offset < 9; offset++)
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

        set_mirrored_led(
            region_start + offset,
            r,
            g,
            b
        );
    }
}


static void render_drums(drum_data_t drums)
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
// Binary data:
//   brightness      = 0-255
//   metadata bit 0  = note detected
//   metadata 1-4    = one of 14 note positions (0-13)
//
// The 14 logical positions are spread across 20 main-position LEDs.
// A detected note lights the selected LED at full brightness and its
// immediate neighbours at OTHER_ADJACENT_SCALE. The glow may extend one LED
// into either reserved 2-LED gap, so one dark LED always remains between
// Other and the neighbouring drum/vocal component.
// ============================================================

static int other_position_to_led(uint8_t position)
{
    if (position > 13)
        position = 13;

    // Map 0..13 evenly onto the 20-LED MAIN region, rounded to the nearest
    // LED, then offset by one because index 0 of other_led_brightness is the
    // left-side glow LED in the reserved gap.
    int main_offset =
        ((int)position * (OTHER_MAIN_LENGTH - 1) + 6) / 13;

    return main_offset + 1;
}


static void raise_other_led(
    int offset,
    uint8_t brightness
)
{
    if (offset < 0 || offset >= OTHER_RENDER_LENGTH)
        return;

    if (brightness > other_led_brightness[offset])
        other_led_brightness[offset] = brightness;
}


static void render_other(
    uint8_t brightness,
    uint8_t metadata
)
{
    // Fade all previous Other LEDs first.
    for (int i = 0; i < OTHER_RENDER_LENGTH; i++)
    {
        other_led_brightness[i] =
            fade_value(
                other_led_brightness[i],
                OTHER_FADE_AMOUNT
            );
    }

    bool note_detected =
        (metadata & 0x01) != 0;

    uint8_t position =
        (metadata >> 1) & 0x0F;

    if (note_detected && position < 14 && brightness > 0)
    {
        int main_offset =
            other_position_to_led(position);

        uint8_t adjacent_brightness =
            ((uint16_t)brightness *
             OTHER_ADJACENT_SCALE) / 255;

        raise_other_led(
            main_offset,
            brightness
        );

        raise_other_led(
            main_offset - 1,
            adjacent_brightness
        );

        raise_other_led(
            main_offset + 1,
            adjacent_brightness
        );
    }

    for (int offset = 0; offset < OTHER_RENDER_LENGTH; offset++)
    {
        uint8_t level =
            ((uint16_t)other_led_brightness[offset] *
             OTHER_BRIGHTNESS) / 255;

        uint8_t r =
            ((uint16_t)OTHER_R * level) / 255;

        uint8_t g =
            ((uint16_t)OTHER_G * level) / 255;

        uint8_t b =
            ((uint16_t)OTHER_B * level) / 255;

        set_mirrored_led(
            OTHER_RENDER_START + offset,
            r,
            g,
            b
        );
    }
}


// ============================================================
// VOCAL SPECTRUM
// ============================================================

static void render_vocals(
    const uint8_t spectrum[8],
    uint8_t volume,
    uint8_t onset
)
{

    const int vocals_length = VOCAL_END - VOCAL_START + 1;

    // I want the number of LEDs on to equal VOCALS_LENGTH when the volume equals 230 and I want leds_on to equal 0 when volume equals 153
    int leds_on = (int)((float)(volume - VOCALS_VOLUME_LOWER_THRESHOLD) / (VOCALS_VOLUME_UPPER_THRESHOLD - VOCALS_VOLUME_LOWER_THRESHOLD) * vocals_length);
    if (leds_on < 0) leds_on = 0;
    if (leds_on > vocals_length) leds_on = vocals_length;

    for (int i = 0; i < leds_on; i++)
    {
        set_mirrored_led(
            VOCAL_START + i,
            ((uint16_t)VOCAL_R * VOCAL_BRIGHTNESS) / 255,
            ((uint16_t)VOCAL_G * VOCAL_BRIGHTNESS) / 255,
            ((uint16_t)VOCAL_B * VOCAL_BRIGHTNESS) / 255
        );
    }

}


// ============================================================
// RENDER ONE COMPLETE AUDIO FRAME
// ============================================================

static void render_frame(
    const audio_frame_t *frame
)
{
    clear_led_buffer();


    drum_data_t drums =
        decode_drums(frame->drums);


    render_bass(
        frame->bass
    );


    render_drums(
        drums
    );


    render_other(
        frame->other_brightness,
        frame->other_metadata
    );


    render_vocals(
        frame->vocals_spectrum,
        frame->vocals_volume,
        frame->vocals_onset
    );


    send_led_buffer();
}


// ============================================================
// PLAY SONG VISUALISATION
// ============================================================

static void play_song(void)
{
    FILE *file =
        fopen(
            SONG_PATH,
            "rb"
        );


    if (file == NULL)
    {
        ESP_LOGE(
            TAG,
            "Could not open %s",
            SONG_PATH
        );

        return;
    }


    // --------------------------------------------------------
    // Determine file size and number of frames
    // --------------------------------------------------------

    fseek(
        file,
        0,
        SEEK_END
    );


    long file_size =
        ftell(file);


    rewind(file);


    if (
        file_size %
        sizeof(audio_frame_t)
        != 0
    )
    {
        ESP_LOGW(
            TAG,
            "File size is not a multiple of 12 bytes"
        );
    }


    uint32_t total_frames =
        file_size /
        sizeof(audio_frame_t);


    ESP_LOGI(
        TAG,
        "Song file: %ld bytes",
        file_size
    );


    ESP_LOGI(
        TAG,
        "Frames: %lu",
        (unsigned long)total_frames
    );


    ESP_LOGI(
        TAG,
        "Starting visualisation"
    );


    // Reset drum fade state.

    kick_brightness = 0;
    hihat_brightness = 0;
    memset(
        other_led_brightness,
        0,
        sizeof(other_led_brightness)
    );


    audio_frame_t frame;

    uint32_t frame_number = 0;


    int64_t start_time =
        esp_timer_get_time();


    // ========================================================
    // MAIN PLAYBACK LOOP
    // ========================================================

    while (
        fread(
            &frame,
            sizeof(frame),
            1,
            file
        ) == 1
    )
    {
        // Absolute timestamp for this frame.

        int64_t target_time =
            start_time +
            (
                (int64_t)frame_number *
                FRAME_PERIOD_US
            );


        // ----------------------------------------------------
        // Wait for correct frame time
        // ----------------------------------------------------

        while (true)
        {
            int64_t now =
                esp_timer_get_time();


            int64_t remaining =
                target_time - now;


            if (remaining <= 0)
                break;


            // Sleep for most of long waits.

            if (remaining > 3000)
            {
                vTaskDelay(
                    pdMS_TO_TICKS(1)
                );
            }
            else
            {
                // Very short remaining time.

                taskYIELD();
            }
        }


        // ----------------------------------------------------
        // Render this frame
        // ----------------------------------------------------

        render_frame(
            &frame
        );


        frame_number++;
    }


    fclose(file);


    ESP_ERROR_CHECK(
        led_strip_clear(led_strip)
    );


    ESP_LOGI(
        TAG,
        "Visualisation finished"
    );
}


// ============================================================
// MAIN
// ============================================================

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "Starting Audio Visualiser"
    );


    configure_led();


    configure_storage();


    play_song();
}