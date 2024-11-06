#include <ESP8266WiFi.h>
#include <espnow.h>

// #define TRANSMITTER // Раскомментируйте для прошивки приемника

#define DEBUG // Проверка каналов через serial и вывод MAC-адреса приемника
// #define DISABLE_LED  // Отключение сетодиодов на плате

#define PPM_PIN 5 // D1 Пин подключения PPM (одинаковый для приемника и передатчика)

#define LED_PIN 2
#define MAX_CHANNELS 8

#define PPM_FRAME 22500
#define PPM_PULSE_WIDTH 300

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Укажите адрес приемника

volatile uint8_t channelIndex = 0;
volatile uint32_t lastPulseTime = 0;
uint32_t last_print = 0;
volatile bool ppmSignalLost = false;

#ifdef TRANSMITTER
#define TIME_LOOP 10
volatile uint16_t channelValues[MAX_CHANNELS] = {0};
volatile bool irqPPM = false;
volatile uint32_t irqTime = 0;
uint32_t pulseLength = 0;
uint32_t last_loop_time = 0;

#else

uint16_t channelValues[MAX_CHANNELS] = {0};
volatile bool pulseState = false;
uint32_t last_receive = 0;
bool connect = false;
#endif

void printInfo();
void IRAM_ATTR generatePPM();
void IRAM_ATTR handlePPMInterrupt();
void setupPPMTimer();
void onDataReceive(uint8_t *mac_addr, uint8_t *incomingData, uint8_t len);

void setup()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

#ifdef DEBUG

  Serial.begin(115200);
  Serial.println();
  if (esp_now_init() != 0)
  {
    Serial.println("Ошибка инициализации ESP-NOW");
    return;
  }
#ifndef TRANSMITTER
  Serial.printf("This mac: %s, ", WiFi.macAddress().c_str());
  Serial.println();
#endif

#endif

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

#ifdef TRANSMITTER
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), handlePPMInterrupt, FALLING);
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  // esp_now_register_send_cb(onDataTransmit);
#else
  pinMode(PPM_PIN, OUTPUT);
  digitalWrite(PPM_PIN, LOW);
  setupPPMTimer();
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(onDataReceive);
#endif
}

void loop()
{

#ifdef TRANSMITTER
  if ((millis() - last_loop_time) > TIME_LOOP)
  {
    last_loop_time = millis();
    if (millis() - (lastPulseTime / 1000) > 100)
      ppmSignalLost = true;

    if (!ppmSignalLost)
    {
      esp_now_send(broadcastAddress, (uint8_t *)channelValues, sizeof(channelValues));
    }

#ifndef DISABLE_LED
    digitalWrite(LED_PIN, ppmSignalLost);
#endif

  }

  if (irqPPM)
  {
    irqPPM = false;

    pulseLength = irqTime - lastPulseTime;
    lastPulseTime = irqTime;

    if (pulseLength > 3000)
    {
      channelIndex = 0;
      ppmSignalLost = false;
    }
    else if (channelIndex < MAX_CHANNELS)
    {
      channelValues[channelIndex++] = pulseLength;
    }
  }

#else

  if ((millis() - last_receive) > 100)
  {
    connect = false;
  }
  else
    connect = true;

#ifndef DISABLE_LED
  digitalWrite(LED_PIN, !connect);
#endif

#endif

#ifdef DEBUG
  if ((millis() - last_print) >= 100)
  {
    last_print = millis();
    printInfo();
  }
#endif
}

void printInfo()
{
  for (int i = 0; i < MAX_CHANNELS; i++)
  {
    Serial.print(channelValues[i]);
    Serial.print('\t');
  }
  Serial.println();
}

#ifdef TRANSMITTER

void IRAM_ATTR handlePPMInterrupt()
{
  irqPPM = true;
  irqTime = micros();
}

#else

void onDataReceive(uint8_t *mac_addr, uint8_t *incomingData, uint8_t len)
{
  last_receive = millis();
  if (len == sizeof(channelValues))
  {
    for (int i = 0; i < MAX_CHANNELS && i < len / 2; i++)
    {
      channelValues[i] = incomingData[i * 2] | (incomingData[i * 2 + 1] << 8);
      channelValues[i] = constrain(channelValues[i], 988, 2012);
    }
  }
}

void setupPPMTimer()
{
  timer1_isr_init();
  timer1_attachInterrupt(generatePPM);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(500);
}

void IRAM_ATTR generatePPM()
{
  volatile static uint16_t summChannelWidth = 0;
  volatile static bool end = false;
  if (pulseState)
  {
    digitalWrite(PPM_PIN, HIGH);
    timer1_write((channelValues[channelIndex] - PPM_PULSE_WIDTH) * 5);
    channelIndex++;
    summChannelWidth += channelValues[channelIndex];
    pulseState = false;
  }
  else
  {
    if (channelIndex >= MAX_CHANNELS)
    {
      if (!end)
      {
        digitalWrite(PPM_PIN, LOW);
        timer1_write(PPM_PULSE_WIDTH * 5);
        end = true;
      }
      else
      {
        channelIndex = 0;
        timer1_write((PPM_FRAME - (summChannelWidth + PPM_PULSE_WIDTH)) * 5);
        summChannelWidth = 0;
      }
    }
    else
    {
      digitalWrite(PPM_PIN, LOW);
      timer1_write(PPM_PULSE_WIDTH * 5);
      pulseState = true;
    }
  }
}

#endif
