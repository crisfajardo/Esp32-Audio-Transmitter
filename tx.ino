// ESP32 TRANSMISOR (TX) - Procesamiento de Audio y Transmisión Bluetooth
// Sistema de comunicación por Bluetooth Classic

#include "BluetoothSerial.h"
#include <driver/i2s.h>
#include <math.h>

// Configuración Bluetooth
BluetoothSerial SerialBT;
const char* btName = "ESP32_Audio_TX";
const char* btPin = "1234";

// Configuración I2S para audio
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE 16000
#define I2S_SAMPLE_BITS 16
#define I2S_READ_LEN (1024)

// Parámetros de procesamiento
#define BUFFER_SIZE 512  // Reducido para Bluetooth
#define PCM_BITS 16
#define MAX_AMPLITUDE ((1 << (PCM_BITS - 1)) - 1)
#define PACKET_HEADER 0xAA55  // Header para identificar paquetes

// Arrays para procesamiento de señal
float originalSignal[BUFFER_SIZE];
float amplifiedSignal[BUFFER_SIZE];
float shiftedSignal[BUFFER_SIZE];
float convolvedSignal[BUFFER_SIZE];
int16_t pcmSignal[BUFFER_SIZE];

// Estructura de paquete para transmisión
struct AudioPacket {
  uint16_t header;
  uint16_t size;
  uint32_t timestamp;
  int16_t data[BUFFER_SIZE];
  uint16_t checksum;
};

AudioPacket txPacket;

// Kernel gaussiano 3x3 para convolución 2D
float kernel[3][3] = {
  {0.0625, 0.125, 0.0625},
  {0.125,  0.25,  0.125},
  {0.0625, 0.125, 0.0625}
};

// Variables de control
bool btConnected = false;
unsigned long lastTxTime = 0;
uint32_t packetCounter = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 TX - Sistema Bluetooth de Audio");
  
  // Inicializar Bluetooth
  initBluetooth();
  
  // Inicializar I2S para captura de audio
  initI2S();
  
  // Configurar packet header
  txPacket.header = PACKET_HEADER;
  
  Serial.println("Sistema TX Bluetooth inicializado");
  Serial.println("Esperando conexión del receptor...");
}

void initBluetooth() {
  if (!SerialBT.begin(btName)) {
    Serial.println("Error iniciando Bluetooth");
    return;
  }
  
  SerialBT.setPin(btPin);
  Serial.printf("Bluetooth iniciado: %s\n", btName);
  Serial.printf("PIN: %s\n", btPin);
  
  // Callback para conexión
  SerialBT.register_callback(btCallback);
}

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  switch (event) {
    case ESP_SPP_SRV_OPEN_EVT:
      Serial.println("Cliente Bluetooth conectado");
      btConnected = true;
      break;
      
    case ESP_SPP_CLOSE_EVT:
      Serial.println("Cliente Bluetooth desconectado");
      btConnected = false;
      break;
      
    default:
      break;
  }
}

void initI2S() {
  i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
  
  Serial.println("I2S inicializado para captura de audio");
}

// 1. Función de amplificación optimizada
void amplifySignal(float* input, float* output, int size, float gain) {
  float maxVal = 0;
  
  // Aplicar ganancia
  for (int i = 0; i < size; i++) {
    output[i] = input[i] * gain;
    float absVal = fabs(output[i]);
    if (absVal > maxVal) {
      maxVal = absVal;
    }
  }
  
  // Normalizar si excede el rango
  if (maxVal > 1.0) {
    float normFactor = 0.95 / maxVal; // Dejar margen del 5%
    for (int i = 0; i < size; i++) {
      output[i] *= normFactor;
    }
  }
}

// 2. Función de desplazamiento con limitación
void shiftSignal(float* input, float* output, int size, float offset) {
  for (int i = 0; i < size; i++) {
    output[i] = input[i] + offset;
    
    // Limitar rango [-1, 1]
    if (output[i] > 1.0) output[i] = 1.0;
    if (output[i] < -1.0) output[i] = -1.0;
  }
}

// 3. Convolución 2D optimizada para tiempo real
void apply2DConvolution(float* input, float* output, int size) {
  // Buffer temporal para evitar sobreescritura
  static float temp[BUFFER_SIZE];
  
  for (int i = 0; i < size; i++) {
    float sum = 0;
    
    // Aplicar kernel 3x3 considerando vecinos
    for (int ki = -1; ki <= 1; ki++) {
      int idx = i + ki;
      
      // Manejo de bordes con reflejo
      if (idx < 0) idx = -idx;
      if (idx >= size) idx = 2 * size - idx - 2;
      
      sum += input[idx] * kernel[ki + 1][1]; // Usar fila central del kernel
    }
    
    temp[i] = sum;
  }
  
  // Copiar resultado normalizado
  float maxVal = 0;
  for (int i = 0; i < size; i++) {
    float absVal = fabs(temp[i]);
    if (absVal > maxVal) maxVal = absVal;
  }
  
  float normFactor = (maxVal > 0) ? 0.95 / maxVal : 1.0;
  for (int i = 0; i < size; i++) {
    output[i] = temp[i] * normFactor;
  }
}

// 4. Modulación PCM con dithering
void modulatePCM(float* input, int16_t* output, int size) {
  for (int i = 0; i < size; i++) {
    // Añadir dithering para reducir ruido de cuantización
    float dither = ((float)random(-32, 32)) / 32768.0;
    float sample = input[i] + dither * 0.5;
    
    // Cuantización con saturación
    int32_t quantized = (int32_t)(sample * MAX_AMPLITUDE);
    
    if (quantized > MAX_AMPLITUDE) quantized = MAX_AMPLITUDE;
    if (quantized < -MAX_AMPLITUDE) quantized = -MAX_AMPLITUDE;
    
    output[i] = (int16_t)quantized;
  }
}

// Calcular checksum simple
uint16_t calculateChecksum(int16_t* data, int size) {
  uint32_t sum = 0;
  for (int i = 0; i < size; i++) {
    sum += abs(data[i]);
  }
  return (uint16_t)(sum & 0xFFFF);
}

// Función principal de procesamiento y transmisión
void processAndTransmitAudio() {
  if (!btConnected) {
    delay(100);
    return;
  }
  
  size_t bytesRead;
  uint8_t i2sData[I2S_READ_LEN];
  
  // Leer datos del micrófono I2S
  esp_err_t result = i2s_read(I2S_PORT, (void*)i2sData, I2S_READ_LEN, &bytesRead, 100);
  
  if (result == ESP_OK && bytesRead > 0) {
    int samples = bytesRead / 2;
    samples = min(samples, BUFFER_SIZE);
    
    // Convertir datos I2S a float normalizado
    for (int i = 0; i < samples; i++) {
      int16_t sample = (i2sData[i*2 + 1] << 8) | i2sData[i*2];
      originalSignal[i] = (float)sample / MAX_AMPLITUDE;
    }
    
    // PROCESAMIENTO DIGITAL DE SEÑAL
    
    // PASO 1: Amplificación (ganancia 1.8)
    amplifySignal(originalSignal, amplifiedSignal, samples, 1.8);
    
    // PASO 2: Desplazamiento DC (offset 0.2)
    shiftSignal(amplifiedSignal, shiftedSignal, samples, 0.2);
    
    // PASO 3: Convolución 2D
    apply2DConvolution(shiftedSignal, convolvedSignal, samples);
    
    // PASO 4: Modulación PCM
    modulatePCM(convolvedSignal, pcmSignal, samples);
    
    // PREPARAR PAQUETE PARA TRANSMISIÓN
    txPacket.size = samples;
    txPacket.timestamp = millis();
    memcpy(txPacket.data, pcmSignal, samples * sizeof(int16_t));
    txPacket.checksum = calculateChecksum(pcmSignal, samples);
    
    // TRANSMITIR VÍA BLUETOOTH
    size_t packetSize = sizeof(txPacket.header) + sizeof(txPacket.size) + 
                       sizeof(txPacket.timestamp) + (samples * sizeof(int16_t)) + 
                       sizeof(txPacket.checksum);
                       
    size_t written = SerialBT.write((uint8_t*)&txPacket, packetSize);
    
    if (written == packetSize) {
      packetCounter++;
      lastTxTime = millis();
      
      // Estadísticas cada 100 paquetes
      if (packetCounter % 100 == 0) {
        Serial.printf("TX: %lu paquetes enviados, %d muestras/paquete\n", 
                      packetCounter, samples);
      }
    } else {
      Serial.printf("Error transmisión: enviado %d de %d bytes\n", written, packetSize);
    }
  }
}

void loop() {
  // Verificar estado de Bluetooth
  if (SerialBT.hasClient()) {
    if (!btConnected) {
      btConnected = true;
      Serial.println("Cliente reconectado");
    }
  } else {
    if (btConnected) {
      btConnected = false;
      Serial.println("Cliente desconectado");
    }
  }
  
  // Procesar y transmitir audio
  processAndTransmitAudio();
  
  // Control de flujo para evitar saturación
  delay(5);
}

// Función de diagnóstico
void printSystemStatus() {
  Serial.println("=== ESTADO DEL SISTEMA TX ===");
  Serial.printf("Bluetooth: %s\n", btConnected ? "CONECTADO" : "DESCONECTADO");
  Serial.printf("Paquetes enviados: %lu\n", packetCounter);
  Serial.printf("Última transmisión: %lu ms\n", lastTxTime);
  Serial.printf("Frecuencia muestreo: %d Hz\n", I2S_SAMPLE_RATE);
  Serial.printf("Tamaño buffer: %d muestras\n", BUFFER_SIZE);
  Serial.println("=============================");
}
