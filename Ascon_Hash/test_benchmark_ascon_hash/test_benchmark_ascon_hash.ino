#include <Arduino.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <AsconESP32_Hash.h>
#include "esp_system.h"

int Ram_core_use0;
int Ram_core_use1;

#ifdef ESP32
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "soc/rtc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// Benchmark code
#define NUM_BLOCKS 1  // Giảm xuống để phù hợp với bộ nhớ Arduino
#define BLOCK_SIZE 2000   // 16 bytes = 128 bits // đã thay đổi ở đây // b0
#define MESSAGE_SIZE  (NUM_BLOCKS * BLOCK_SIZE)

struct CoreStats {
    float enc_throughput;
    float dec_throughput;
    float hash_throughput;
    int ram_use;           
    bool completed; 
} core_stats[2] = {{0,0,0,0,false},{0,0,0,0,false}};

portMUX_TYPE statsMutex = portMUX_INITIALIZER_UNLOCKED;

void print_total_throughput() {
    float total_enc = core_stats[0].enc_throughput + core_stats[1].enc_throughput;
    float total_dec = core_stats[0].dec_throughput + core_stats[1].dec_throughput;
    float total_hash = core_stats[0].hash_throughput + core_stats[1].hash_throughput;
  
    Serial.println("\n=== Total System Performance ===");
   
    Serial.printf("Total Encryption Throughput: %.2f MB/s\n", total_enc);
    Serial.printf("Total Decryption Throughput: %.2f MB/s\n", total_dec);
    Serial.printf("Total Hash Throughput: %.2f MB/s\n", total_hash);
    Serial.printf("Total Combined Throughput: %.2f MB/s\n", total_enc + total_dec + total_hash);
    Serial.println("--------------------");
}

SemaphoreHandle_t printSemaphore = NULL;
TaskHandle_t finishedTaskHandle = NULL;
volatile bool resultsPrinted = false;

void check_and_print_total() {
    bool all_completed = false;
    bool should_print = false;

    portENTER_CRITICAL(&statsMutex);
    all_completed = core_stats[0].completed && core_stats[1].completed;
    if (all_completed && !resultsPrinted) {
        should_print = true;
        resultsPrinted = true;
    }
    portEXIT_CRITICAL(&statsMutex);

    if (should_print) {
        vTaskDelay(pdMS_TO_TICKS(100));

        float total_enc = core_stats[0].enc_throughput + core_stats[1].enc_throughput;
        float total_dec = core_stats[0].dec_throughput + core_stats[1].dec_throughput;
        float total_hash = core_stats[0].hash_throughput + core_stats[1].hash_throughput;

        Serial.println("\n=== Final System Performance ===");
       Serial.printf("Core 0 - Enc: %.2f MB/s, Dec: %.2f MB/s, Hash: %.2f MB/s, RAM used: %d bytes\n",
              core_stats[0].enc_throughput, core_stats[0].dec_throughput, core_stats[0].hash_throughput, core_stats[0].ram_use);
Serial.printf("Core 1 - Enc: %.2f MB/s, Dec: %.2f MB/s, Hash: %.2f MB/s, RAM used: %d bytes\n",
              core_stats[1].enc_throughput, core_stats[1].dec_throughput, core_stats[1].hash_throughput, core_stats[1].ram_use);

               
        Serial.println("--------------------");

        Serial.printf("Total Encryption Throughput: %.2f MB/s\n", total_enc);
        Serial.printf("Total Decryption Throughput: %.2f MB/s\n", total_dec);
        Serial.printf("Total Hash Throughput: %.2f MB/s\n", total_hash);
        Serial.printf("Total Combined Throughput: %.2f MB/s\n", total_enc + total_dec + total_hash);
        Serial.println("--------------------\n");
    }
}

void print_results(const char* operation, uint64_t time_us, size_t size) {
    xSemaphoreTake(printSemaphore, portMAX_DELAY);

    float throughput = (float)size / (time_us / 1000000.0f); // bytes per second
    float mbps = throughput / (1024 * 1024); // convert to MB/s
    float cycles_per_byte = (float)(ESP.getCpuFreqMHz() * time_us) / (size ? size : 1);
    
    Serial.printf("%s:\n", operation);
    Serial.printf("Time: %llu microseconds\n", time_us);
    Serial.printf("Throughput: %.2f MB/s\n", mbps);
    Serial.printf("Cycles/byte: %.2f\n", cycles_per_byte);
    
    Serial.println("--------------------");
    xSemaphoreGive(printSemaphore);
}

int freeRam() {
#ifdef ESP32
    return ESP.getFreeHeap(); // Dùng cho ESP32
#else
    extern int __heap_start, *__brkval;
    int v;
    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval); // Dùng cho AVR
#endif
}


void generate_nonce(unsigned char* nonce) {
    for (size_t i = 0; i < CRYPTO_NPUBBYTES; i++) {
        nonce[i] = (unsigned char)esp_random();  
    }
}


void IRAM_ATTR benchmark_core(void* parameter) {
    int core_id = xPortGetCoreID();

  int ramBefore = freeRam();

    unsigned char* message = (unsigned char*)heap_caps_malloc(MESSAGE_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    unsigned char* ciphertext = (unsigned char*)heap_caps_malloc(MESSAGE_SIZE + CRYPTO_ABYTES, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    unsigned char* decrypted = (unsigned char*)heap_caps_malloc(MESSAGE_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
 unsigned char* hash = (unsigned char*)heap_caps_malloc(CRYPTO_BYTES, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);


 if (!message || !ciphertext || !decrypted || !hash ) {
        Serial.printf("Core %d: Memory allocation failed!\n", core_id);
        if (message) heap_caps_free(message);
        if (ciphertext) heap_caps_free(ciphertext);
        if (decrypted) heap_caps_free(decrypted);
        if (hash) heap_caps_free(hash);
      
        vTaskDelete(NULL);
        return;
    }

    // Test vectors (key/nonce/ad)
    unsigned char key[CRYPTO_KEYBYTES] = {
        0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B,
        0x0C, 0x0D, 0x0E, 0x0F
    };
    unsigned char nonce[CRYPTO_NPUBBYTES];
    generate_nonce(nonce);
    unsigned char ad[16] = {
        0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,
        0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F
    };
    size_t adlen = sizeof(ad);

    // Initialize message from sensordata
 
   for (size_t i = 0; i < MESSAGE_SIZE; i++) {
        message[i] = (unsigned char)i;
    }

    unsigned long long clen = 0, mlen = 0;
    uint64_t start_time = 0, end_time = 0;

    // Warm up caches
    crypto_aead_encrypt(ciphertext, &clen, message, MESSAGE_SIZE, ad, adlen, NULL, nonce, key);
 
   crypto_aead_decrypt(decrypted, &mlen, NULL, ciphertext, clen, ad, adlen, nonce, key);
crypto_hash(hash, message, MESSAGE_SIZE);
    portDISABLE_INTERRUPTS();  // Disable interrupts for accurate timing
    
    // Test encryption (run many times and average)
    start_time = esp_timer_get_time();
    for (int i = 0; i < 10; i++) {
        crypto_aead_encrypt(ciphertext, &clen, message, MESSAGE_SIZE, ad, adlen, NULL, nonce, key);
    }
    end_time = esp_timer_get_time();
    portENABLE_INTERRUPTS();
    uint64_t enc_time = (end_time - start_time) / 10;  // Average time per run
    
    portDISABLE_INTERRUPTS();
    // Test decryption (run many times and average)
    start_time = esp_timer_get_time();
    for (int i = 0; i < 10; i++) {
        crypto_aead_decrypt(decrypted, &mlen, NULL, ciphertext, clen, ad, adlen, nonce, key);
    }
    end_time = esp_timer_get_time();
    portENABLE_INTERRUPTS();
    uint64_t dec_time = (end_time - start_time) / 10;

     portDISABLE_INTERRUPTS();
    start_time = esp_timer_get_time();
    for (int i = 0; i < 10; i++) {
        crypto_hash(hash, message, MESSAGE_SIZE);
    }
    end_time = esp_timer_get_time();
    portENABLE_INTERRUPTS();
    uint64_t hash_time = (end_time - start_time) / 10;
	 int ramAfter = freeRam();
   int used = ramBefore - ramAfter;
    // Calculate throughputs
    float enc_throughput = (float)MESSAGE_SIZE / (enc_time / 1000000.0f) / (1024 * 1024);
    float dec_throughput = (float)MESSAGE_SIZE / (dec_time / 1000000.0f) / (1024 * 1024);
     float hash_throughput = (float)MESSAGE_SIZE / (hash_time / 1000000.0f) / (1024 * 1024);
	 
    // Update core statistics
    portENTER_CRITICAL(&statsMutex);
    core_stats[core_id].enc_throughput = enc_throughput;
    core_stats[core_id].dec_throughput = dec_throughput;
	 core_stats[core_id].hash_throughput = hash_throughput;
   core_stats[core_id].ram_use = used; 
    core_stats[core_id].completed = true;
    portEXIT_CRITICAL(&statsMutex);

    // Print results for this core
    Serial.printf("\n=== Results from Core %d ===\n", core_id);
    print_results("Encryption", enc_time, MESSAGE_SIZE);
    print_results("Decryption", dec_time, MESSAGE_SIZE);
	print_results("Ascon-Hash", hash_time, MESSAGE_SIZE);


/* ramBefore lượng ram còn trống trước khi chạy chương trình,ramAfter lượng ram còn trống sau khi chạy chương trình.
=> chạy chương trình ngốn Ram
=> Ram sau khi chạy chương trình < Ram trước khi chạy chương trình.
=> Ram tiêu thụ: Ram trước - Ram sau.
ví dụ :
Trước khi chạy bộ ,chúng ta có 5 chai nước 
Sau khi chạy bộ chỉ còn lại 3 chai.
=> số nước đã tốn cho việc chạy bộ = 5-3;
*/

    // Verify decryption
    if (memcmp(message, decrypted, MESSAGE_SIZE) != 0) {
        Serial.printf("Core %d: ERROR - Decryption failed!\n", core_id);
    } else {
        Serial.printf("Core %d: Verification successful\n", core_id);
    }
    
    check_and_print_total();
    // === Gửi nonce / ciphertext / tag qua Serial1 =====
    // Chỉ gửi từ core 0 để tránh in nhiều lần
  if (xPortGetCoreID() == 0) {
		Serial.print("Sensor data hash: ");
    for (size_t i = 0; i < CRYPTO_BYTES; i++) {
        Serial.printf("%02x", hash[i]);
    }
    Serial.println();
  }

    // Free memory
    heap_caps_free(message);
    heap_caps_free(ciphertext);
    heap_caps_free(decrypted);

    // Sleep a bit before next iteration
    vTaskDelay(pdMS_TO_TICKS(1000));
     // Task hoàn thành (nếu bạn muốn task lặp lại, không delete)
}
void setup() {
    Serial.begin(115200);
    // GPS uart
    
  
    #ifdef ESP32
    // Khởi tạo semaphore để đồng bộ việc in
    printSemaphore = xSemaphoreCreateMutex();
    // Đặt tần số CPU tối đa và điều chỉnh điện áp
    rtc_cpu_freq_config_t config;
    rtc_clk_cpu_freq_get_config(&config);
    config.freq_mhz = 240;  // Overclock to 240MHz (giữ nguyên như code gốc)
    config.div = 1;
    rtc_clk_cpu_freq_set_config_fast(&config);
    
    // Tắt Wi-Fi và Bluetooth để tiết kiệm năng lượng (giữ nguyên)
    esp_wifi_stop();
    esp_bt_controller_disable();
    
    Serial.println("\nASCON-128 Benchmark on ESP32 (Dual Core)");
    Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Testing with message size: %d bytes\n", MESSAGE_SIZE);
    Serial.println("Running on both cores...\n");
    
    // Chạy benchmark trên cả 2 core (giữ nguyên)
    xTaskCreatePinnedToCore(benchmark_core, "benchmark_core0", 32768, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(benchmark_core, "benchmark_core1", 32768, NULL, 1, NULL, 1);
    
    
    // Run benchmark (non-ESP32)
    #endif
}
void loop() {
    #ifdef ESP32
    // ESP32 không cần làm gì trong loop khi đang chạy task
    vTaskDelay(pdMS_TO_TICKS(1000));
    #else
    delay(1000);
    #endif
}

