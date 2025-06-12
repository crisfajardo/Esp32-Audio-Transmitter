// ESP32 RECEPTOR (RX) - Recepción Bluetooth y Reproducción de Audio por DAC
// Complemento del sistema transmisor Bluetooth

#include "BluetoothSerial.h"
#include <driver/dac.h>
#include <math.h>

// Configuración Bluetooth
BluetoothSerial SerialBT;
const char* btName = "ESP32_Audio_RX";
const char* targetDeviceName = "ESP32_Audio_TX";

// Configuración DAC
#define DAC_CHANNEL DAC_CHANNEL_1  // GPIO25
#define DAC_RESOLUTION 255         // 8-bit DAC
#define PLAYBACK_SAMPLE_RATE 16000
#define PLAYBACK_DELAY_US (1000000 / PLAYBACK_SAMPLE_RATE)

// Parámetros de procesamiento (deben coincidir con TX)
#define BUFFER_SIZE 512
#define PCM_BITS 16
#define MAX_AMPLITUDE ((1 << (PCM_BITS - 1)) - 1)
#define PACKET_HEADER 0xAA55
#define AUDIO_BUFFER_SIZE 2048

// Estructura de paquete (debe coincidir con TX)
struct AudioPacket {
  uint16_t header;
  uint16_t size;
  uint32_t timestamp;
  int16_t data[BUFFER_SIZE];
  uint16_t checksum;
};

// Buffers para recepción y reproducción
AudioPacket rxPacket;
uint8_t receiveBuffer[sizeof(AudioPacket)];
int16_t audioBuffer[AUDIO_BUFFER_SIZE];
uint8_t dacBuffer[AUDIO_BUFFER_SIZE];

// Variables de control
bool btConnected = false;
bool isReceiving = false;
unsigned long lastRxTime = 0;
uint32_t packetsReceived = 0;
uint32_t packetsCorrupted = 0;
int audioBufferHead = 0;
int audioBufferTail = 0;
int audioBufferCount = 0;

// Variables para reproducción
hw_timer_t* playbackTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool playbackActive = false;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 RX - Sistema Receptor Bluetooth Audio");
  
  // Inicializar DAC
  initDAC();
  
  // Inicializar Bluetooth
  initBluetooth();
  
  // Inicializar timer para reproducción
  initPlaybackTimer();
  
  Serial.println("Sistema RX Bluetooth inicializado");
  Serial.println("Buscando transmisor...");
  
  // Intentar conectar al transmisor
  connectToTransmitter();
}

void initDAC() {
  // Configurar DAC en GPIO25
  dac_output_enable(DAC_CHANNEL);
  dac_output_voltage(DAC_CHANNEL, 0);
  
  Serial.println("DAC inicializado en GPIO25");
}

void initBluetooth() {
  if (!SerialBT.begin(btName)) {
    Serial.println("Error iniciando Bluetooth");
    return;
  }
  
  Serial.printf("Bluetooth iniciado: %s\n", btName);
  
  // Callback para eventos de conexión
  SerialBT.register_callback(btCallback);
}

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  switch (event) {
    case ESP_SPP_OPEN_EVT:
      Serial.println("Conectado al transmisor");
      btConnected = true;
      isReceiving = true;
      break;
      
    case ESP_SPP_CLOSE_EVT:
      Serial.println("Desconectado del transmisor");
      btConnected = false;
      isReceiving = false;
      stopPlayback();
      break;
      
    case ESP_SPP_DATA_IND_EVT:
      // Datos recibidos - se procesarán en el loop principal
      break;
      
    default:
      break;
  }
}

void connectToTransmitter() {
  Serial.printf("Intentando conectar a: %s\n", targetDeviceName);
  
  if (SerialBT.connect(targetDeviceName)) {
    Serial.println("Conexión exitosa!");
    btConnected = true;
    isReceiving = true;
  } else {
    Serial.println("Error de conexión. Reintentando en 5 segundos...");
    delay(5000);
  }
}

// Timer interrupt para reproducción de audio
void IRAM_ATTR onPlaybackTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  
  if (playbackActive && audioBufferCount > 0) {
    // Obtener muestra del buffer circular
    uint8_t dacValue = dacBuffer[audioBufferTail];
    audioBufferTail = (audioBufferTail + 1) % AUDIO_BUFFER_SIZE;
    audioBufferCount--;
    
    // Enviar al DAC
    dac_output_voltage(DAC_CHANNEL, dacValue);
  }
  
  portEXIT_CRITICAL_ISR(&timerMux);
}

void initPlaybackTimer() {
  // Configurar timer para reproducción a 16kHz
  playbackTimer = timerBegin(0, 80, true); // 80MHz/80 = 1MHz
  timerAttachInterrupt(playbackTimer, &onPlaybackTimer, true);
  timerAlarmWrite(playbackTimer, PLAYBACK_DELAY_US, true); // Período en microsegundos
  timerAlarmEnable(playbackTimer);
  
  Serial.println("Timer de reproducción inicializado");
}

void startPlayback() {
  portENTER_CRITICAL(&timerMux);
  playbackActive = true;
  portEXIT_CRITICAL(&timerMux);
  Serial.println("Reproducción iniciada");
}

void stopPlayback() {
  portENTER_CRITICAL(&timerMux);
  playbackActive = false;
  portEXIT_CRITICAL(&timerMux);
  
  // Silenciar DAC
  dac_output_voltage(DAC_CHANNEL, 128); // Punto medio (silencio)
  Serial.println("Reproducción detenida");
}

// Calcular checksum simple (debe coincidir con TX)
uint16_t calculateChecksum(int16_t* data, int size) {
  uint32_t sum = 0;
  for (int i = 0; i < size; i++) {
    sum += abs(data[i]);
  }
  return (uint16_t)(sum & 0xFFFF);
}

// Validar paquete recibido
bool validatePacket(AudioPacket* packet) {
  // Verificar header
  if (packet->header != PACKET_HEADER) {
    return false;
  }
  
  // Verificar tamaño
  if (packet->size > BUFFER_SIZE) {
    return false;
  }
  
  // Verificar checksum
  uint16_t calculatedChecksum = calculateChecksum(packet->data, packet->size);
  if (calculatedChecksum != packet->checksum) {
    return false;
  }
  
  return true;
}

// Convertir PCM 16-bit a DAC 8-bit
void convertPCMtoDAC(int16_t* pcmData, uint8_t* dacData, int size) {
  for (int i = 0; i < size; i++) {
    // Convertir de [-32768, 32767] a [0, 255]
    int32_t temp = pcmData[i] + 32768; // Desplazar a rango positivo
    dacData[i] = (uint8_t)(temp >> 8); // Escalar a 8 bits
    
    // Aplicar un filtro suave para reducir ruido
    if (i > 0) {
      dacData[i] = (dacData[i] + dacData[i-1]) / 2;
    }
  }
}

// Añadir audio al buffer circular
void addToAudioBuffer(uint8_t* audioData, int size) {
  portENTER_CRITICAL(&timerMux);
  
  for (int i = 0; i < size; i++) {
    if (audioBufferCount < AUDIO_BUFFER_SIZE) {
      dacBuffer[audioBufferHead] = audioData[i];
      audioBufferHead = (audioBufferHead + 1) % AUDIO_BUFFER_SIZE;
      audioBufferCount++;
    } else {
      // Buffer lleno, descartar muestra más antigua
      audioBufferTail = (audioBufferTail + 1) % AUDIO_BUFFER_SIZE;
      dacBuffer[audioBufferHead] = audioData[i];
      audioBufferHead = (audioBufferHead + 1) % AUDIO_BUFFER_SIZE;
    }
  }
  
  portEXIT_CRITICAL(&timerMux);
}

// Función principal de recepción y procesamiento
void receiveAndProcessAudio() {
  if (!btConnected || !isReceiving) {
    return;
  }
  
  // Verificar si hay datos disponibles
  if (SerialBT.available() >= sizeof(AudioPacket)) {
    size_t bytesRead = SerialBT.readBytes(receiveBuffer, sizeof(AudioPacket));
    
    if (bytesRead == sizeof(AudioPacket)) {
      // Copiar datos al buffer de paquete
      memcpy(&rxPacket, receiveBuffer, sizeof(AudioPacket));
      
      // Validar paquete
      if (validatePacket(&rxPacket)) {
        packetsReceived++;
        lastRxTime = millis();
        
        // Convertir PCM a formato DAC
        uint8_t dacData[BUFFER_SIZE];
        convertPCMtoDAC(rxPacket.data, dacData, rxPacket.size);
        
        // Añadir al buffer de reproducción
        addToAudioBuffer(dacData, rxPacket.size);
        
        // Iniciar reproducción si no está activa y tenemos suficientes datos
        if (!playbackActive && audioBufferCount > BUFFER_SIZE) {
          startPlayback();
        }
        
        // Estadísticas cada 100 paquetes
        if (packetsReceived % 100 == 0) {
          Serial.printf("RX: %lu paquetes recibidos, Buffer: %d/%d\n", 
                        packetsReceived, audioBufferCount, AUDIO_BUFFER_SIZE);
        }
        
      } else {
        packetsCorrupted++;
        Serial.printf("Paquete corrupto #%lu (Total corruptos: %lu)\n", 
                      packetsReceived + packetsCorrupted, packetsCorrupted);
      }
    }
  }
  
  // Detener reproducción si no hay datos por más de 500ms
  if (playbackActive && (millis() - lastRxTime) > 500) {
    Serial.println("Sin datos por 500ms - pausando reproducción");
    stopPlayback();
  }
}

void loop() {
  // Verificar conexión Bluetooth
  if (!btConnected) {
    stopPlayback();
    delay(1000);
    Serial.println("Reintentando conexión...");
    connectToTransmitter();
    return;
  }
  
  // Recibir y procesar audio
  receiveAndProcessAudio();
  
  // Mostrar estado del buffer cada 5 segundos
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 5000) {
    printSystemStatus();
    lastStatusTime = millis();
  }
  
  // Control de flujo
  delay(1);
}

// Función de diagnóstico
void printSystemStatus() {
  Serial.println("=== ESTADO DEL SISTEMA RX ===");
  Serial.printf("Bluetooth: %s\n", btConnected ? "CONECTADO" : "DESCONECTADO");
  Serial.printf("Recepción: %s\n", isReceiving ? "ACTIVA" : "INACTIVA");
  Serial.printf("Reproducción: %s\n", playbackActive ? "ACTIVA" : "PAUSADA");
  Serial.printf("Paquetes recibidos: %lu\n", packetsReceived);
  Serial.printf("Paquetes corruptos: %lu\n", packetsCorrupted);
  Serial.printf("Buffer audio: %d/%d muestras\n", audioBufferCount, AUDIO_BUFFER_SIZE);
  Serial.printf("Última recepción: %lu ms\n", lastRxTime);
  
  if (packetsReceived > 0) {
    float successRate = ((float)packetsReceived / (packetsReceived + packetsCorrupted)) * 100.0;
    Serial.printf("Tasa de éxito: %.2f%%\n", successRate);
  }
  
  Serial.println("==============================");
}

// Funciones de control adicionales
void adjustVolume(float factor) {
  // Esta función podría implementarse para controlar el volumen
  // modificando los valores antes de enviar al DAC
  Serial.printf("Ajuste de volumen: %.2f\n", factor);
}

void resetAudioBuffer() {
  portENTER_CRITICAL(&timerMux);
  audioBufferHead = 0;
  audioBufferTail = 0;
  audioBufferCount = 0;
  portEXIT_CRITICAL(&timerMux);
  Serial.println("Buffer de audio reiniciado");
}