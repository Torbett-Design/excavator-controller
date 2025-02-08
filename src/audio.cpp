#include "audio.h"
#include "pins.h"
#include <SPIFFS.h>
#include <driver/i2s.h>
#include <queue>
#include <mutex>

namespace Audio {
    // Audio configuration
    #define I2S_SAMPLE_RATE     44100
    #define I2S_CHANNEL_NUM     1
    #define I2S_BITS_PER_SAMPLE 16
    #define DMA_BUF_COUNT       8
    #define DMA_BUF_LEN         1024
    #define AUDIO_QUEUE_SIZE    10
    #define AUDIO_TASK_STACK    4096

    // Audio file paths
    static const char* SOUND_START = "/start.wav";
    static const char* SOUND_IDLE = "/idle.wav";
    static const char* SOUND_POWERUP = "/powerup.wav";
    static const char* SOUND_POWER = "/power.wav";
    static const char* SOUND_HYDRAULIC = "/hydraulic.wav";
    static const char* SOUND_POWERDOWN = "/powerdown.wav";
    static const char* SOUND_STOP = "/stop.wav";
    static const char* SOUND_LOW_BATTERY = "/low-batt.wav";

    // Thread safety
    static std::mutex audioMutex;
    static QueueHandle_t audioQueue;
    static TaskHandle_t audioTaskHandle;
    static volatile bool audioStopRequest = false;

    struct AudioCommand {
        const char* filename;
        bool loop;
    };

    static void audioTask(void* parameter) {
        AudioCommand cmd;
        
        while (true) {
            if (xQueueReceive(audioQueue, &cmd, portMAX_DELAY) == pdTRUE) {
                File file = SPIFFS.open(cmd.filename);
                if (!file) continue;

                file.seek(44); // Skip WAV header
                size_t bytesRead;
                uint8_t buffer[1024];
                
                do {
                    bytesRead = file.read(buffer, sizeof(buffer));
                    if (bytesRead > 0) {
                        size_t bytesWritten;
                        i2s_write(I2S_NUM_0, buffer, bytesRead, &bytesWritten, portMAX_DELAY);
                    }
                    
                    if (cmd.loop && file.position() >= file.size()) {
                        file.seek(44);
                    }
                    
                    if (audioStopRequest || uxQueueMessagesWaiting(audioQueue) > 0) {
                        break;
                    }
                    
                } while (bytesRead > 0);
                
                file.close();
                audioStopRequest = false;
            }
        }
    }

    static void setupI2S() {
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
            .fixed_mclk = 0
        };

        i2s_pin_config_t pin_config = {
            .bck_io_num = Pins::I2S_BCLK,
            .ws_io_num = Pins::I2S_LRCLK,
            .data_out_num = Pins::I2S_DOUT,
            .data_in_num = I2S_PIN_NO_CHANGE
        };

        i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
        i2s_set_pin(I2S_NUM_0, &pin_config);
    }

    void setup() {
        SPIFFS.begin(true);
        setupI2S();
        audioQueue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(AudioCommand));
        xTaskCreate(audioTask, "AudioPlayer", AUDIO_TASK_STACK, NULL, 1, &audioTaskHandle);
    }

    void playWAVAsync(const char* filename, bool loop) {
        AudioCommand cmd = {filename, loop};
        xQueueSend(audioQueue, &cmd, portMAX_DELAY);
    }

    void stopAudio() {
        audioStopRequest = true;
        while (!uxQueueMessagesWaiting(audioQueue)) {
            xQueueReset(audioQueue);
        }
    }

    void startEngine() {
        playWAVAsync(SOUND_START);
        playWAVAsync(SOUND_IDLE, true);
    }

    void enginePowerUp() {
        playWAVAsync(SOUND_POWERUP);
        playWAVAsync(SOUND_POWER, true);
    }

    void engineHydraulic() {
        playWAVAsync(SOUND_HYDRAULIC, true);
    }

    void enginePowerDown() {
        playWAVAsync(SOUND_POWERDOWN);
        playWAVAsync(SOUND_IDLE, true);
    }

    void stopEngine() {
        playWAVAsync(SOUND_STOP);
    }

    void playLowBatteryWarning() {
        std::lock_guard<std::mutex> lock(audioMutex);
        playWAVAsync(SOUND_LOW_BATTERY);
    }
}
