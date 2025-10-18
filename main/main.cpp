// ===================================================================
//                        INCLUSÃO DE BIBLIOTECAS
// ===================================================================
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include "driver/i2s.h"
#include <math.h>
#include "esp_task_wdt.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "esp_task_wdt.h"
// ===================================================================
//                      CONFIGURAÇÕES GLOBAIS
// ===================================================================

// --- Configuração da Rede (Modo Access Point) ---
const char* ssid_ap = "EqualizadorESP32";
const char* password_ap = "12345678";

// --- Configuração dos Pinos e I2S ---
#define MASTE_CLK 4
#define I2S_BCLK 5
#define I2S_DOUT 6
#define I2S_LRCK 7
#define GPIO_FLT 15
#define GPIO_DEMP 16
#define GPIO_XSMT 17
#define GPIO_FMT 18
#define SAMPLE_RATE 22050
#define BUFFER_SIZE 256
#define FIR_ORDER 65 

// --- Coeficientes FIR (5 Bandas, 65 Taps, Ponto Fixo) ---
const int16_t fir_low[FIR_ORDER] = { 13, 13, 11, 6, -1, -11, -22, -32, -41, -47, -49, -46, -37, -23, -2, 24, 55, 88, 122, 156, 188, 216, 239, 256, 266, 269, 264, 252, 234, 209, 178, 143, 104, 3341, 104, 143, 178, 209, 234, 252, 264, 269, 266, 256, 239, 216, 188, 156, 122, 88, 55, 24, -2, -23, -37, -46, -49, -47, -41, -32, -22, -11, -1, 6, 11 };
const int16_t fir_low_mid[FIR_ORDER] = { -13, -16, -16, -13, -8, 0, 10, 20, 29, 36, 40, 39, 34, 23, 7, -13, -37, -64, -92, -120, -146, -168, -184, -193, -195, -188, -172, -148, -115, -75, -29, 23, 80, 5262, 80, 23, -29, -75, -115, -148, -172, -188, -195, -193, -184, -168, -146, -120, -92, -64, -37, -13, 7, 23, 34, 39, 40, 36, 29, 20, 10, 0, -8, -13, -16 };
const int16_t fir_mid[FIR_ORDER] = { -3, -2, 1, 6, 11, 15, 17, 16, 11, 3, -10, -25, -42, -59, -74, -85, -92, -92, -85, -69, -45, -12, 28, 75, 126, 180, 235, 289, 339, 384, 421, 448, 464, -10748, 464, 448, 421, 384, 339, 289, 235, 180, 126, 75, 28, -12, -45, -69, -85, -92, -92, -85, -74, -59, -42, -25, -10, 3, 11, 16, 17, 15, 11, 6, 1 };
const int16_t fir_high_mid[FIR_ORDER] = { -1, 3, 0, -4, 2, 4, -4, -5, 7, 6, -10, -9, 14, 13, -19, -19, 25, 26, -33, -35, 43, 47, -56, -62, 73, 82, -96, -108, 126, 143, -165, -188, 216, 11883, 216, -188, -165, 143, 126, -108, -96, 82, 73, -62, -56, 47, 43, -35, -33, 26, 25, -19, -19, 13, 14, -9, -10, 6, 7, -5, -4, 2, -4, 0, 3 };
const int16_t fir_high[FIR_ORDER] = { -1, 2, -1, -2, 3, -2, -3, 5, -4, -4, 7, -6, -6, 10, -8, -9, 14, -11, -13, 19, -16, -18, 26, -23, -25, 35, -32, -35, 48, -45, -49, 66, -64, -22064, -64, 66, -49, -45, 48, -35, -32, 35, -25, -23, 26, -18, -16, 19, -13, -11, 14, -9, -8, 10, -6, -6, 7, -4, -4, 5, -3, -2, -1, 2, -1 };

int32_t fir_coeffs[FIR_ORDER];
int32_t fir_coeffs2[FIR_ORDER];
QueueHandle_t fir_coeffs_Queue;

// --- Buffers e Variáveis Globais ---
int16_t history_low[FIR_ORDER] = {0}, history_low_mid[FIR_ORDER] = {0}, history_mid[FIR_ORDER] = {0}, history_high_mid[FIR_ORDER] = {0}, history_high[FIR_ORDER] = {0};
volatile float gain_low = 1.0, gain_low_mid = 1.0, gain_mid = 1.0, gain_high_mid = 1.0, gain_high = 1.0;
int16_t filter[FIR_ORDER] = {0};
// --- Servidor Web ---
AsyncWebServer server(80);

// --- Arquivos Embutidos na Flash ---
extern const uint8_t controle_html_start[] asm("_binary_controle_html_start");
extern const uint8_t controle_html_end[]   asm("_binary_controle_html_end");
size_t controle_len;
extern const uint8_t musica_wav_start[] asm("_binary_musica_wav_start");
extern const uint8_t musica_wav_end[]   asm("_binary_musica_wav_end");
extern const uint8_t musica_wav_size[]  asm("_binary_musica_wav_size");
size_t musica_len;
// size_t musica_len2;

// --- Função de Filtragem ---
int32_t apply_fir_filter(int16_t sample, const int16_t* coeffs, int16_t* history) {
    for (int i = FIR_ORDER - 1; i > 0; i--) { history[i] = history[i - 1]; }
    history[0] = sample;
    int64_t result = 0;
    for (int i = 0; i < FIR_ORDER; i++) { result += (int64_t)history[i] * coeffs[i]; }
    return (int32_t)(result >> 15);
}

int32_t apply_fir_filter_2(int16_t sample, int16_t* history)
{
    if (uxQueueMessagesWaiting(fir_coeffs_Queue) > 0) 
    {
        xQueueReceive(fir_coeffs_Queue, fir_coeffs2, portMAX_DELAY);
    }
    for (int i = FIR_ORDER - 1; i > 0; i--) { history[i] = history[i - 1]; }
    history[0] = sample;
    int64_t result = 0;
    for (int i = 0; i < FIR_ORDER; i++) { result += (int64_t)history[i] * fir_coeffs2[i]; }
    return (int32_t)(result >> 15);

}

// --- Tarefa de Áudio e Equalização ---
void audio_task(void* param) {
    //esp_task_wdt_delete(NULL);

    // Tamanho do arquivo WAV
    musica_len = musica_wav_end - musica_wav_start; 
    ESP_LOGI("PDS","Tamanho calculado é de %d" , musica_len);
    // musica_len2 = (size_t)musica_wav_size;
    // ESP_LOGI("PDS","Tamanho informado é de %d" , musica_len2);

    
    // Pular os 44 bytes do cabeçalho WAV
    const uint8_t *data_start = musica_wav_start + 44;
    const uint8_t *data_end   = musica_wav_end;

    // Calcula a quantidade de amostras de 16 bits de áudio
    size_t num_samples = (data_end - data_start) / sizeof(int16_t);
    ESP_LOGI("PDS","Tamanho da amostra é de %d" , num_samples);

    // Agrupa os bytes em amostras de 16 bits
    const int16_t *samples = (const int16_t *)data_start;

    // Índice da amostra a ser processada e enviada para a saída de áudio
    size_t i = 0;

    int16_t i2s_buffer_out[BUFFER_SIZE * 2];
    size_t buf_index = 0;

    while (true) {
        // esp_task_wdt_reset(); // reinicia o contador do watchdog

        int16_t original_sample = samples[i++];
        int32_t low_out      = apply_fir_filter_2(original_sample, history_low);
        // int32_t low_out      = apply_fir_filter(original_sample, fir_low, history_low);
        // int32_t low_mid_out  = apply_fir_filter(original_sample, fir_low_mid, history_low_mid);
        // int32_t mid_out      = apply_fir_filter(original_sample, fir_mid, history_mid);
        // int32_t high_mid_out = apply_fir_filter(original_sample, fir_high_mid, history_high_mid);
        // int32_t high_out     = apply_fir_filter(original_sample, fir_high, history_high);
        
        // mixagem multiplicativa dos ganhos com os áudios filtrados
        int64_t final_sum = (int64_t)(low_out * gain_low)
                        //   + (int64_t)(low_mid_out * gain_low_mid)
                        //   + (int64_t)(mid_out * gain_mid)
                        //   + (int64_t)(high_mid_out * gain_high_mid)
                        //   + (int64_t)(high_out * gain_high)
                          ; 
                
        if (final_sum > 32767) final_sum = 32767;
        if (final_sum < -32768) final_sum = -32768;
        
        // Prepara o buffer I2S (estéreo: mesma amostra em ambos os canais)
        i2s_buffer_out[buf_index++] = (int16_t)final_sum;
        i2s_buffer_out[buf_index++] = (int16_t)final_sum;

        // Quando o buffer estiver cheio, envie para o I2S
        // ESP_LOGI("PDS","Amostra %d de %d" , i, num_samples);
        // printf("Amostra %d de %d\n" , i, num_samples);
        if (buf_index >= BUFFER_SIZE * 2) {
            // ESP_LOGI("PDS","Enviando i2s");
            size_t bytes_written;
            i2s_write(I2S_NUM_0, i2s_buffer_out, sizeof(i2s_buffer_out), &bytes_written, portMAX_DELAY);
            buf_index = 0;
        }

        if (i >= num_samples) {
            i = 0;
            ESP_LOGI("PDS","Reiniciando áudio...");
            // printf("Reiniciando áudio...\n");
        }
        // vTaskDelay(1); // Pequena pausa para evitar sobrecarga da CPU
    }
}

// --- Setup ---
void setup() {
    fir_coeffs_Queue = xQueueCreate(1, sizeof(fir_coeffs));
    controle_len = controle_html_end - controle_html_start;
    Serial.begin(115200);
    delay(1000);
    pinMode(MASTE_CLK, OUTPUT);
    digitalWrite(MASTE_CLK, LOW);
    pinMode(GPIO_FLT, OUTPUT);
    digitalWrite(GPIO_FLT, HIGH);
    pinMode(GPIO_DEMP, OUTPUT);
    digitalWrite(GPIO_DEMP, LOW);
    pinMode(GPIO_XSMT, OUTPUT);
    digitalWrite(GPIO_XSMT, HIGH);
    pinMode(GPIO_FMT, OUTPUT);
    digitalWrite(GPIO_FMT, LOW);
    Serial.println("\nConfigurando o ESP32 em modo Access Point...");
    WiFi.softAP(ssid_ap, password_ap);
    IPAddress ip = WiFi.softAPIP();
    Serial.print("Access Point iniciado. Endereco IP: ");
    Serial.println(ip);
    if (MDNS.begin("equalizer")) {
        Serial.println("Servidor mDNS iniciado. Acesse em http://equalizer.local");
    }
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", controle_html_start, controle_len); });
    // server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *r) {
    //     if (r->hasParam("band") && r->hasParam("gain")) {
    //         String b = r->getParam("band")->value();
    //         float g = r->getParam("gain")->value().toFloat();
    //         if (b == "low") gain_low = g;
    //         else if (b == "low_mid") gain_low_mid = g;
    //         else if (b == "mid") gain_mid = g;
    //         else if (b == "high_mid") gain_high_mid = g;
    //         else if (b == "high") gain_high = g;
    //         // else if (b == "filter") filter = g;
    //         r->send(200, "text/plain", "OK");
    //     } else {
    //         r->send(400, "text/plain", "Faltam parametros");
    //     }
    // });
    server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){},
        NULL,  // não usamos upload handler
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        // data contém o corpo do POST
        StaticJsonDocument<2048> doc; // tamanho suficiente para 65 inteiros
        DeserializationError error = deserializeJson(doc, data, len);

        if (error) {
            Serial.print("Erro ao parsear JSON: ");
            Serial.println(error.c_str());
            request->send(400, "text/plain", "JSON inválido");
            return;
        }

        if (!doc.containsKey("coeffs")) {
            request->send(400, "text/plain", "Faltam coeficientes");
            return;
        }

        JsonArray arr = doc["coeffs"].as<JsonArray>();
        if (arr.size() != 65) {
            request->send(400, "text/plain", "Esperados 65 coeficientes");
            return;
        }

        for (int i = 0; i < 65; i++) {
            fir_coeffs[i] = arr[i].as<int16_t>();
        }

        xQueueOverwrite(fir_coeffs_Queue, fir_coeffs);

        // Serial.println("Coeficientes recebidos:");
        // for (int i = 0; i < 65; i++) {
        //     Serial.print(fir_coeffs[i]);
        //     Serial.print(i < 64 ? ", " : "\n");
        // }

        request->send(200, "text/plain", "OK");
        }
    );

    server.begin();
    Serial.println("Servidor Web iniciado.");
    i2s_config_t i2s_config = { .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), .sample_rate = SAMPLE_RATE, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, .communication_format = I2S_COMM_FORMAT_STAND_I2S, .intr_alloc_flags = 0, .dma_buf_count = 8, .dma_buf_len = BUFFER_SIZE, .use_apll = false };
    i2s_pin_config_t pin_config = { .bck_io_num = I2S_BCLK, .ws_io_num = I2S_LRCK, .data_out_num = I2S_DOUT, .data_in_num = I2S_PIN_NO_CHANGE };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM_0);
   
    // aumentei a prioridade da tarefa de audio pra reduzir ruido do wi-fi
    //esp_task_wdt_delete(xTaskGetIdleTaskHandleForCore(1));
    xTaskCreatePinnedToCore(audio_task, "EqualizerTask", 8192, NULL, configMAX_PRIORITIES - 1, NULL, 1);

    Serial.println("Setup concluido. Equalizador 5 Bandas funcional.");
}

void loop() {
    // Vazio
    vTaskDelay(pdMS_TO_TICKS(100));

}