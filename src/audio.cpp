#include "audio.h"
#include "pins.h"
#include <SPIFFS.h>
#include <driver/i2s.h>
#include <queue>
#include <mutex>

namespace Audio
{
// Audio configuration
#define I2S_SAMPLE_RATE 44100
#define I2S_CHANNEL_NUM 1
#define I2S_BITS_PER_SAMPLE 16
#define DMA_BUF_COUNT 8
#define DMA_BUF_LEN 1024
#define AUDIO_QUEUE_SIZE 10
#define AUDIO_TASK_STACK 4096

    // Thread safety
    static std::mutex audioMutex;
    static QueueHandle_t audioQueue;
    static TaskHandle_t audioTaskHandle;
    static volatile bool audioStopRequest = false;

    struct AudioCommand
    {
        const char *filename;
        bool loop;
        PlaybackMode mode;
        uint8_t volume; // 0-255 for mixing levels
    };

    // Mixing buffer
    static int16_t mixBuffer[DMA_BUF_LEN / 2]; // 16-bit samples
    static const int MAX_MIXED_SOUNDS = 4;
    static AudioCommand activeSounds[MAX_MIXED_SOUNDS];
    static size_t activeCount = 0;

    // Add queue for pending sounds
    static QueueHandle_t pendingQueue;
#define PENDING_QUEUE_SIZE 5

    struct AudioFile
    {
        File file;
        size_t position;
        bool active;
    };

    static AudioFile audioFiles[MAX_MIXED_SOUNDS];
    static std::queue<AudioCommand> pendingSounds;

    static void mixSamples(int16_t *out, const int16_t *in1, const int16_t *in2, size_t len, uint8_t vol1, uint8_t vol2)
    {
        for (size_t i = 0; i < len; i++)
        {
            int32_t mixed = ((int32_t)in1[i] * vol1 / 255) + ((int32_t)in2[i] * vol2 / 255);
            out[i] = constrain(mixed, INT16_MIN, INT16_MAX);
        }
    }

    static void audioTask(void *parameter)
    {
        AudioCommand cmd;

        while (true)
        {
            // Check for new commands
            if (xQueueReceive(audioQueue, &cmd, 0) == pdTRUE)
            {
                switch (cmd.mode)
                {
                case PlaybackMode::REPLACE:
                    // Stop all current sounds
                    for (int i = 0; i < MAX_MIXED_SOUNDS; i++)
                    {
                        if (audioFiles[i].active)
                        {
                            audioFiles[i].file.close();
                            audioFiles[i].active = false;
                        }
                    }
                    activeCount = 0;
                    // Start new sound
                    audioFiles[0].file = SPIFFS.open(cmd.filename);
                    audioFiles[0].file.seek(44);
                    audioFiles[0].position = 44;
                    audioFiles[0].active = true;
                    activeSounds[0] = cmd;
                    activeCount = 1;
                    break;

                case PlaybackMode::MIX:
                    if (activeCount < MAX_MIXED_SOUNDS)
                    {
                        int slot = activeCount;
                        audioFiles[slot].file = SPIFFS.open(cmd.filename);
                        audioFiles[slot].file.seek(44);
                        audioFiles[slot].position = 44;
                        audioFiles[slot].active = true;
                        activeSounds[slot] = cmd;
                        activeCount++;
                    }
                    break;

                case PlaybackMode::QUEUE:
                    pendingSounds.push(cmd);
                    break;
                }
            }

            // Mix active sounds
            memset(mixBuffer, 0, sizeof(mixBuffer));
            bool allInactive = true;

            for (size_t i = 0; i < activeCount; i++)
            {
                if (!audioFiles[i].active)
                    continue;

                int16_t tempBuffer[DMA_BUF_LEN / 2];
                size_t bytesRead = audioFiles[i].file.read((uint8_t *)tempBuffer, sizeof(tempBuffer));

                if (bytesRead > 0)
                {
                    allInactive = false;
                    // Mix with existing buffer
                    mixSamples(mixBuffer, mixBuffer, tempBuffer, bytesRead / 2, 255, activeSounds[i].volume);

                    // Handle looping
                    if (audioFiles[i].file.position() >= audioFiles[i].file.size())
                    {
                        if (activeSounds[i].loop)
                        {
                            audioFiles[i].file.seek(44);
                        }
                        else
                        {
                            audioFiles[i].file.close();
                            audioFiles[i].active = false;
                        }
                    }
                }
            }

            // Check if we should start any pending sounds
            if (allInactive && !pendingSounds.empty())
            {
                AudioCommand nextSound = pendingSounds.front();
                pendingSounds.pop();

                audioFiles[0].file = SPIFFS.open(nextSound.filename);
                audioFiles[0].file.seek(44);
                audioFiles[0].position = 44;
                audioFiles[0].active = true;
                activeSounds[0] = nextSound;
                activeCount = 1;
                allInactive = false;
            }

            // Output mixed buffer if we have active sounds
            if (!allInactive)
            {
                size_t bytesWritten;
                i2s_write(I2S_NUM_0, mixBuffer, sizeof(mixBuffer), &bytesWritten, portMAX_DELAY);
            }
        }
    }

    static void IRAM_ATTR i2s_dma_callback(void *arg)
    {
        size_t bytesWritten;
        i2s_write(I2S_NUM_0, mixBuffer, sizeof(mixBuffer), &bytesWritten, 0);
    }

    static void setupI2S()
    {
        i2s_config_t i2s_config = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
            .sample_rate = I2S_SAMPLE_RATE,
            .bits_per_sample = (i2s_bits_per_sample_t)I2S_BITS_PER_SAMPLE,
            .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
            .communication_format = I2S_COMM_FORMAT_STAND_I2S,
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count = DMA_BUF_COUNT,
            .dma_buf_len = DMA_BUF_LEN,
            .use_apll = false,
            .tx_desc_auto_clear = true,
            .fixed_mclk = 0};

        i2s_pin_config_t pin_config = {
            .bck_io_num = Pins::I2S_BCLK,
            .ws_io_num = Pins::I2S_LRCLK,
            .data_out_num = Pins::I2S_DOUT,
            .data_in_num = I2S_PIN_NO_CHANGE};

        i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
        i2s_set_pin(I2S_NUM_0, &pin_config);

        // Register DMA interrupt handler with correct signature
        esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
                       i2s_dma_callback, NULL, NULL);
    }
    void setup()
    {
        SPIFFS.begin(true);
        setupI2S();
        audioQueue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(AudioCommand));
        pendingQueue = xQueueCreate(PENDING_QUEUE_SIZE, sizeof(AudioCommand));
        xTaskCreate(audioTask, "AudioPlayer", AUDIO_TASK_STACK, NULL, 1, &audioTaskHandle);
    }
    void playWAVAsync(const char* filename, bool loop, PlaybackMode mode, uint8_t volume)
    {
        // Check if sound is already playing
        for (size_t i = 0; i < activeCount; i++) {
            if (audioFiles[i].active && strcmp(activeSounds[i].filename, filename) == 0) {
                return; // Exit without playing if sound already exists
            }
        }
    
        // If we get here, sound isn't playing so proceed with normal play logic
        AudioCommand cmd = {filename, loop, mode, volume};
        xQueueSend(audioQueue, &cmd, portMAX_DELAY);
    }
    void stopAudio()
    {
        audioStopRequest = true;
        while (!uxQueueMessagesWaiting(audioQueue))
        {
            xQueueReset(audioQueue);
        }
    }

    void startEngine()
    {
        playWAVAsync(SOUND_START, false, PlaybackMode::REPLACE);
        playWAVAsync(SOUND_IDLE, true, PlaybackMode::QUEUE);
    }

    void enginePowerUp()
    {
        playWAVAsync(SOUND_POWERUP, false, PlaybackMode::REPLACE);
        playWAVAsync(SOUND_POWER, true, PlaybackMode::QUEUE);
    }

    void engineHydraulic()
    {
        playWAVAsync(SOUND_HYDRAULIC, true, PlaybackMode::MIX, 192); // 75% volume when mixed
    }

    void enginePowerDown()
    {
        playWAVAsync(SOUND_POWERDOWN, false, PlaybackMode::REPLACE);
        playWAVAsync(SOUND_IDLE, true, PlaybackMode::QUEUE);
    }

    void tracks()
    {
        playWAVAsync(SOUND_TRACK, true, PlaybackMode::MIX, 192);
    }

    void stopEngine()
    {
        playWAVAsync(SOUND_STOP, false, PlaybackMode::REPLACE);
    }

    void playLowBatteryWarning()
    {
        playWAVAsync(SOUND_LOW_BATTERY, false, PlaybackMode::MIX, 255);
    }

    void playReverseBeep()
    {
        playWAVAsync(SOUND_REVERSE_BEEP, true, PlaybackMode::MIX, 192);
    }

    bool isPlaying(const char *filename)
    {
        for (size_t i = 0; i < activeCount; i++)
        {
            if (audioFiles[i].active && strcmp(activeSounds[i].filename, filename) == 0)
            {
                return true;
            }
        }
        return false;
    }

    void stopTrack(const char *filename)
    {
        std::lock_guard<std::mutex> lock(audioMutex);

        for (size_t i = 0; i < activeCount; i++)
        {
            if (audioFiles[i].active && strcmp(activeSounds[i].filename, filename) == 0)
            {
                audioFiles[i].file.close();
                audioFiles[i].active = false;

                // Shift remaining active sounds if needed
                if (i < activeCount - 1)
                {
                    memmove(&audioFiles[i], &audioFiles[i + 1],
                            (activeCount - i - 1) * sizeof(AudioFile));
                    memmove(&activeSounds[i], &activeSounds[i + 1],
                            (activeCount - i - 1) * sizeof(AudioCommand));
                }
                activeCount--;
                break;
            }
        }
    }

}