#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <NewPing.h>
#include <string.h> // Add this for memcpy

#define SONAR_COUNT 4  // 초음파 센서 개수
const uint8_t TRIGGER_PINS[SONAR_COUNT] = {13, 11, 7, 3};
const uint8_t ECHO_PINS[SONAR_COUNT] = {12, 10, 6, 2};
const char* SENSOR_NAMES[SONAR_COUNT] = {"전방", "후방", "좌측", "우측"};
#define MAX_DISTANCE 400  // 최대 측정 거리 (cm)
#define MIN_PING_INTERVAL 30  // 각 센서 간의 최소 간격(ms) - 증가시켜 안정성 향상

// 전역 변수 선언
SemaphoreHandle_t sonarDataMutex;
QueueHandle_t serialOutputQueue;
#define SERIAL_QUEUE_LENGTH 10
#define SERIAL_ITEM_SIZE 128

// volatile 변수는 여러 태스크에서 접근하기 때문에 사용
volatile uint16_t sonarDistances[SONAR_COUNT] = {0};
volatile unsigned long lastMeasurementTime[SONAR_COUNT] = {0};

// NewPing 객체 배열
NewPing sonars[SONAR_COUNT] = {
  NewPing(TRIGGER_PINS[0], ECHO_PINS[0], MAX_DISTANCE),
  NewPing(TRIGGER_PINS[1], ECHO_PINS[1], MAX_DISTANCE),
  NewPing(TRIGGER_PINS[2], ECHO_PINS[2], MAX_DISTANCE),
  NewPing(TRIGGER_PINS[3], ECHO_PINS[3], MAX_DISTANCE)
};

// 태스크 핸들 선언
TaskHandle_t sonarTaskHandle;
TaskHandle_t processingTaskHandle;
TaskHandle_t serialOutputTaskHandle;

// 초음파 센서 태스크
void vSonarTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    for (uint8_t i = 0; i < SONAR_COUNT; i++) {
      unsigned long currentTime = millis();
      uint16_t distance = sonars[i].ping_cm();
      
      // 세마포어 획득
      if (xSemaphoreTake(sonarDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sonarDistances[i] = distance;
        lastMeasurementTime[i] = currentTime;
        xSemaphoreGive(sonarDataMutex);
      }
      
      // 절대 시간 지연 대신 상대적 지연 사용
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MIN_PING_INTERVAL));
    }
  }
}

// 데이터 처리 태스크
// 데이터 처리 태스크
void vProcessingTask(void *pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(100);
  uint16_t localDistances[SONAR_COUNT];
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    bool dataUpdated = false;
    
    // 세마포어 획득 시간 제한 설정
    if (xSemaphoreTake(sonarDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(localDistances, (const void*)sonarDistances, sizeof(localDistances));
      xSemaphoreGive(sonarDataMutex);
      dataUpdated = true;
    }
    
    if (dataUpdated) {
      // 각 센서 데이터를 개별적으로 처리하되 더 간단한 형식으로
      char buffer[SERIAL_ITEM_SIZE];
      
      // 모든 센서 데이터를 하나의 문자열로 결합 (더 간단한 방식)
      uint16_t d0 = (localDistances[0] > 0 && localDistances[0] < MAX_DISTANCE) ? localDistances[0] : 0;
      uint16_t d1 = (localDistances[1] > 0 && localDistances[1] < MAX_DISTANCE) ? localDistances[1] : 0;
      uint16_t d2 = (localDistances[2] > 0 && localDistances[2] < MAX_DISTANCE) ? localDistances[2] : 0;
      uint16_t d3 = (localDistances[3] > 0 && localDistances[3] < MAX_DISTANCE) ? localDistances[3] : 0;
      
      // 더 작은 문자열로 분할
      sprintf(buffer, "전:%d 후:%d 좌:%d 우:%d cm", d0, d1, d2, d3);
      
      // 큐에 전송
      xQueueSend(serialOutputQueue, buffer, pdMS_TO_TICKS(10));
      
      // 구분선 전송
      strcpy(buffer, "------------------------");
      xQueueSend(serialOutputQueue, buffer, pdMS_TO_TICKS(10));
    }
    
    // 절대 시간 지연 대신 상대적 지연 사용
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}


// 시리얼 출력 태스크
void vSerialOutputTask(void *pvParameters) {
  char buffer[SERIAL_ITEM_SIZE];
  
  for (;;) {
    if (xQueueReceive(serialOutputQueue, buffer, pdMS_TO_TICKS(100)) == pdTRUE) {
      Serial.println(buffer);
    }
    taskYIELD();  // 다른 태스크에 CPU 양보
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // 시리얼 포트가 연결될 때까지 대기
  }
  
  Serial.println(F("초음파 센서 시스템 - FreeRTOS"));
  Serial.println(F("전방/후방/좌측/우측 센서 모니터링"));
  Serial.println();
  
  // 세마포어와 큐 생성
  sonarDataMutex = xSemaphoreCreateMutex();
  if (sonarDataMutex == NULL) {
    Serial.println(F("세마포어 생성 실패"));
    while(1);  // 오류 시 정지
  }
  
  serialOutputQueue = xQueueCreate(SERIAL_QUEUE_LENGTH, SERIAL_ITEM_SIZE);
  if (serialOutputQueue == NULL) {
    Serial.println(F("큐 생성 실패"));
    while(1);  // 오류 시 정지
  }
  
  // 태스크 생성 - 스택 크기 증가
  xTaskCreate(vSonarTask, "SonarTask", 256, NULL, 3, &sonarTaskHandle);
  xTaskCreate(vProcessingTask, "ProcessingTask", 256, NULL, 2, &processingTaskHandle);
  xTaskCreate(vSerialOutputTask, "SerialOutputTask", 256, NULL, 1, &serialOutputTaskHandle);
  
  // 스케줄러 시작
  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS를 사용하면 loop()는 실행되지 않음
}
