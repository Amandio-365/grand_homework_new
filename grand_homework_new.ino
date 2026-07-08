#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>      
#include <Adafruit_SSD1306.h>
#include <Preferences.h>         //Flash相关头文件
#include <esp_sleep.h>           //Deep-sleep对应头文件

#define ENC_A_GPIO      4
#define ENC_B_GPIO      5
#define BUTTON_PIN      13
#define LED_PIN         14
#define SERVO_PIN1      11
#define SERVO_PIN2      10
#define LIGHT_PIN       6
#define I2C_SDA_PIN     8
#define I2C_SCL_PIN     9

#define OLED_I2C_ADDR   0x3C
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define DEBOUNCE_DELAY  200
#define INTERVAL        500
#define PAUSE_MAX       500
#define SERVO_MID_ANGLE 90
#define DEEP_SLEEP_TIME 5000
#define FULL_TIME       120000
#define LED_BLINK_TOTAL (INTERVAL * 2)
#define BUTTON_COOLDOWN 10000
#define DEEP_SLEEP_IDLE_MS   60000   // 30秒无操作即进入Deep-sleep状态

#define NVS_CFG  "pet_cfg"   // 命名空间1：存实时状态
#define NVS_MEM  "pet_mem"   // 命名空间2：存累计统计

// 任务句柄
TaskHandle_t TaskInput_Handle   = NULL;
TaskHandle_t TaskState_Handle   = NULL;
TaskHandle_t TaskServo_Handle   = NULL;
TaskHandle_t TaskPeriph_Handle  = NULL;

// 队列：舵机命令队列，传递动作编号
QueueHandle_t xQueueServoCmd = NULL;

// 二值信号量：事件通知
SemaphoreHandle_t xSemButton     = NULL;  // 按键按下事件
SemaphoreHandle_t xSemMoodTick   = NULL;  // 心情衰减定时器事件
SemaphoreHandle_t xSemMotionTick = NULL;  // 随机动作定时器事件

// 互斥量：全局状态保护
SemaphoreHandle_t xMutexState = NULL;

// 软件定时器
TimerHandle_t xTimerMood   = NULL;  // 30s心情衰减
TimerHandle_t xTimerMotion = NULL;  // 1s随机动作检测

ESP32Encoder encoder;
Servo myServo1, myServo2;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Preferences prefs;

// 全局变量
bool LedLightOn = false;
bool isSleeping = false;
bool isHungry = true;
int mood = 5;
int motion = 0;
int headTouchCount = 0;  //累计摸头次数
int feedCount = 0;       //累计玩耍次数 
long encoderCounter = 0;
long counterStart = 0;

unsigned long LedStartMs = 0;
unsigned long FullStart = 0;
unsigned long LowlightStart = 0;
unsigned long PauseStart = 0;
unsigned long lastInteract = 0;
bool LowlightFlag = true;
bool encoderFlag = false;

RTC_DATA_ATTR unsigned long bootMs = 0;         // 累计运行时间(ms)
RTC_DATA_ATTR bool wasSleeping = false;         // 深睡触发原因：true=低光照睡眠，false=待机无操作

// 中断共享变量
volatile bool wakeupFlag = false;

void saveStateToFlash() 
{
  // 保存状态到nvs_cfg命名空间(包括心情，动作，饱食状态，剩余的饱食秒数)
  prefs.begin(NVS_CFG, false); 
  prefs.putInt("mood", mood);
  prefs.putBool("isHungry", isHungry);
  
  long fullRemainSec = 0;
  if (!isHungry)
  {
    fullRemainSec = (FULL_TIME - (millis() - FullStart)) / 1000;
    if (fullRemainSec < 0) fullRemainSec = 0;
  }
  prefs.putLong("fullRemain", fullRemainSec);
  prefs.putInt("motion", motion);
  prefs.end();  
}

void saveStateToMem()
{
  prefs.begin(NVS_MEM, false);
  prefs.putInt("headCnt", headTouchCount);
  prefs.putInt("feedCnt", feedCount);
  prefs.end();
}

void loadStateFromFlash() 
{
  prefs.begin(NVS_CFG, true);  
  mood = prefs.getInt("mood", 5);      //默认值非常重要，可以帮我垫好读入错误的底
  isHungry = prefs.getBool("isHungry", true);
  long fullRemainSec = prefs.getLong("fullRemain", 0);
  motion = prefs.getInt("motion", 0);
  prefs.end();

  // 根据剩余秒数，反向推算 FullStart 时间戳（适配新的 millis 基准）
  if (!isHungry && fullRemainSec > 0) 
  {
    FullStart = millis() - (FULL_TIME - fullRemainSec * 1000);
  } 
  else 
  {
    FullStart = 0;
  }

  prefs.begin(NVS_MEM, true);
  headTouchCount = prefs.getInt("headCnt", 0);
  feedCount = prefs.getInt("feedCnt", 0);
  prefs.end();
}

void enterDeepSleep(bool isLightSleep) {
  //记录进入休眠时的桌宠睡眠状态
  wasSleeping = isLightSleep;

  //记录本次CPU运行时长
  bootMs += millis();

  //OLED熄灭，舵机判断:(如果在睡觉时休眠，保持舵机位置，否则归中)
  display.clearDisplay();
  display.display();  
  display.ssd1306_command(0xAE);

  if (isLightSleep) 
  {
    myServo1.write(170);
    myServo2.write(170);
  } 
  else 
  {
    ServoBackMid();
  }

  saveStateToFlash();
  Serial.println("[SYSTEM] Entering deep sleep...");
  delay(100);  

  // 5. 设置唤醒源：按键按下（低电平）唤醒
  // 你的按键是上拉输入，按下为低电平，所以第二个参数填0
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);

  // 6. 正式进入深度休眠，芯片几乎完全断电
  esp_deep_sleep_start();
}

// 定时器发出到时间的信号
void vTimerMoodCallback(TimerHandle_t xTimer) {
  xSemaphoreGive(xSemMoodTick);
}

void vTimerMotionCallback(TimerHandle_t xTimer) {
  xSemaphoreGive(xSemMotionTick);
}

int readMoodSafe() 
{
  int val;
  xSemaphoreTake(xMutexState, portMAX_DELAY);
  val = mood;
  xSemaphoreGive(xMutexState);
  return val;
}

// 修改心情值
void writeMoodSafe(int val) 
{
  xSemaphoreTake(xMutexState, portMAX_DELAY);
  mood = val;
  xSemaphoreGive(xMutexState);
}

// 读取睡眠状态
bool readSleepSafe() 
{
  bool val;
  xSemaphoreTake(xMutexState, portMAX_DELAY);
  val = isSleeping;
  xSemaphoreGive(xMutexState);
  return val;
}

// 修改睡眠状态
void writeSleepSafe(bool val) 
{
  xSemaphoreTake(xMutexState, portMAX_DELAY);
  isSleeping = val;
  xSemaphoreGive(xMutexState);
}

// 读取饥饿状态
bool readHungrySafe() 
{
  bool val;
  xSemaphoreTake(xMutexState, portMAX_DELAY);
  val = isHungry;
  xSemaphoreGive(xMutexState);
  return val;
}

// 修改饥饿状态
void writeHungrySafe(bool val) 
{
  xSemaphoreTake(xMutexState, portMAX_DELAY);
  isHungry = val;
  xSemaphoreGive(xMutexState);
}


void ServoBackMid()  //舵机归中
{
  myServo1.write(SERVO_MID_ANGLE);
  myServo2.write(SERVO_MID_ANGLE);
}

void Motion_Play() //动作1---玩耍
{
  for (int i = 0;i < 3;i++)
  {
    myServo1.write(45);
    myServo2.write(45);
    vTaskDelay(pdMS_TO_TICKS(300)); //RTOS自有的Tick“标准”
    myServo1.write(135);
    myServo2.write(135);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  ServoBackMid();
  vTaskDelay(pdMS_TO_TICKS(200));
  motion = 0;
}

void Motion_Wander()  //动作2---闲逛
{
  for (int i = 0;i < 3;i++)
  {
    myServo1.write(30);
    myServo2.write(30);
    vTaskDelay(pdMS_TO_TICKS(600));
    ServoBackMid();
    vTaskDelay(pdMS_TO_TICKS(600));
  }
  vTaskDelay(pdMS_TO_TICKS(400));
  motion = 0;
}

void Motion_Tired()  //动作3---疲惫
{
  for (int i = 0;i < 2;i++)
  {
    myServo1.write(120);
    vTaskDelay(pdMS_TO_TICKS(600));
    myServo1.write(140);
    vTaskDelay(pdMS_TO_TICKS(600));
  }
  ServoBackMid();
  //RTOS优化：将干等Delay()改为vTaskDelay，在未到达600Ticks之前让出CPU给其他任务
  vTaskDelay(pdMS_TO_TICKS(600));     
  motion = 0;
}

void Motion_GoAhead()  //动作4---前进
{
  for (int i = 0;i < 3;i++)
  {
    myServo1.write(20);
    myServo2.write(160);
    vTaskDelay(pdMS_TO_TICKS(600));
    ServoBackMid();
    vTaskDelay(pdMS_TO_TICKS(600));
  }
  vTaskDelay(pdMS_TO_TICKS(400));
  motion = 0;
}
//高优先级：按键事件、编码器喂食检测、光敏睡眠检测
void Task_Input(void *pvParameters) 
{
  (void)pvParameters;

  static unsigned long lastValidPressMs = (unsigned long)-BUTTON_COOLDOWN;

  for (;;) {
    // 1. 等待按键事件（阻塞等待，不占CPU）
    if (digitalRead(BUTTON_PIN) == LOW) 
    {
      vTaskDelay(pdMS_TO_TICKS(50)); // 跳过抖动
      if (digitalRead(BUTTON_PIN) == LOW) 
      {
        bool sleepState = readSleepSafe();
        // 确认按下，执行业务逻辑
        if (!sleepState) 
        {
          unsigned long now = millis();
          if (now - lastValidPressMs < BUTTON_COOLDOWN) {}
          else
          {
            lastInteract = millis();
            headTouchCount++;
            Serial.println("Head Touch!");
            saveStateToMem(); //立即写入Flash

            int curMood = readMoodSafe();
            writeMoodSafe(curMood < 10 ? curMood + 1 : 10);
            // 触发玩耍动作

            // 触发LED闪烁
            xSemaphoreTake(xMutexState, portMAX_DELAY);
            if (curMood < 10) 
            {
              LedLightOn = true;
              LedStartMs = millis();
            }
            xSemaphoreGive(xMutexState);

            int cmd = 1;
            xQueueSend(xQueueServoCmd, &cmd, 0);
            lastValidPressMs = now;
          }
        } 
        else 
        {
          // 睡眠状态下按键唤醒
          wakeupFlag = true;
        }
        // 等待按键松开，防止重复触发
        while (digitalRead(BUTTON_PIN) == LOW) 
        {
          vTaskDelay(pdMS_TO_TICKS(10));
        }
      }
    }

    // 2. 编码器喂食检测
    if (!readSleepSafe()) 
    {
      if (encoderCounter != encoder.getCount()) 
      {
        xSemaphoreTake(xMutexState, portMAX_DELAY);
        if (!encoderFlag) 
        {
          encoderFlag = true;
          counterStart = encoder.getCount();
        }
        encoderCounter = encoder.getCount();
        PauseStart = millis();
        xSemaphoreGive(xMutexState);
      }

      // 停顿超时重置
      if (millis() - PauseStart >= PAUSE_MAX) 
      {
        xSemaphoreTake(xMutexState, portMAX_DELAY);
        counterStart = encoderCounter = encoder.getCount();
        PauseStart = millis();
        encoderFlag = false;
        xSemaphoreGive(xMutexState);
      }

      // 反转一圈触发喂食
      if ((encoderCounter - counterStart <= -40) && readHungrySafe()) {
        writeHungrySafe(false);
        xSemaphoreTake(xMutexState, portMAX_DELAY);
        FullStart = millis();
        counterStart = encoderCounter;
        xSemaphoreGive(xMutexState);
        lastInteract = millis();
        feedCount++;
        saveStateToMem();
        Serial.println("Feeding process successful!");
      }
    }

    // 3. 光敏睡眠检测
    int lightState = digitalRead(LIGHT_PIN);
    xSemaphoreTake(xMutexState, portMAX_DELAY);
    if ((lightState == HIGH) && (LowlightFlag)) 
    {
      LowlightFlag = false;
      LowlightStart = millis();
    } 
    else if (lightState == LOW) 
    {
      LowlightStart = millis();
      LowlightFlag = true;
    }
    xSemaphoreGive(xMutexState);

    // 满足睡眠条件
    if ((millis() - LowlightStart >= DEEP_SLEEP_TIME) && (lightState == HIGH) && !readSleepSafe()) {
      Serial.println("光照不足!我睡觉去了~");
      // 进入睡眠（交给状态任务统一处理）
      writeSleepSafe(true);
      // 停止两个软件定时器
      xTimerStop(xTimerMood, 0);
      xTimerStop(xTimerMotion, 0);
      // 舵机归位睡眠姿态
      myServo1.write(170);
      myServo2.write(170);
      Serial.println("Sleep Mode On!");
    }

    // 唤醒检测
    if (readSleepSafe() && wakeupFlag) {
      wakeupFlag = false;
      writeSleepSafe(false);
      // 恢复定时器
      xTimerStart(xTimerMood, 0);
      xTimerStart(xTimerMotion, 0);
      // 舵机回中
      xSemaphoreTake(xMutexState, portMAX_DELAY);
      LowlightStart = millis();
      xSemaphoreGive(xMutexState);
      Serial.println("Sleep Mode Off!");
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // 小延时，让出CPU
  }
}

void Task_State(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    // 1. 心情衰减定时器事件
    if (xSemaphoreTake(xSemMoodTick, pdMS_TO_TICKS(10)) == pdPASS) 
    {
      if (readHungrySafe() && !readSleepSafe()) 
      {
        int curMood = readMoodSafe();
        int newMood = (curMood - 1 >= 1) ? curMood - 1 : 1;
        writeMoodSafe(newMood);
        Serial.print("Mood down: ");
        Serial.println(newMood);
      }
    }

    // 2. 随机动作定时器事件
    if (xSemaphoreTake(xSemMotionTick, pdMS_TO_TICKS(10)) == pdPASS) 
    {
      if (!readSleepSafe()) 
      {
        // 检查当前是否空闲
        xSemaphoreTake(xMutexState, portMAX_DELAY);
        bool isIdle = (motion == 0);
        xSemaphoreGive(xMutexState);

        if (isIdle) 
        {
          int r = random(30);
          if (r == 0) 
          {
            int cmd;
            if (readMoodSafe() >= 5) 
            {
              cmd = 2; // 闲逛
            } 
            else 
            {
              cmd = 3; // 疲惫
            }
            xQueueSend(xQueueServoCmd, &cmd, 0);
          }
        }
      }
    }

    // 3. 饱食计时
    if (!readSleepSafe() && !readHungrySafe()) 
    {
      xSemaphoreTake(xMutexState, portMAX_DELAY);
      unsigned long fullTime = FullStart;
      xSemaphoreGive(xMutexState);
      
      if (millis() - fullTime > FULL_TIME) 
      {
        writeHungrySafe(true);
        Serial.println("饱腹结束，我又饿了！");
      }
    }

    unsigned long now = millis();

    xSemaphoreTake(xMutexState, portMAX_DELAY);
    bool tempisSleeping = isSleeping;
    unsigned long templastInteract = lastInteract;
    xSemaphoreGive(xMutexState);
    if (now - templastInteract >= DEEP_SLEEP_IDLE_MS)
    {
      if (tempisSleeping)
        enterDeepSleep(true);
      else enterDeepSleep(false);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

//中优先级：舵机处理任务
void Task_Servo(void *pvParameters) {
  (void)pvParameters;
  int cmd = 0;
  for (;;) {
    // 阻塞等待舵机命令，没命令就无限休眠下去
    if (xQueueReceive(xQueueServoCmd, &cmd, portMAX_DELAY) == pdPASS) {
      if (readSleepSafe()) continue; // 睡眠时忽略动作命令

      xSemaphoreTake(xMutexState, portMAX_DELAY);
      motion = cmd;
      xSemaphoreGive(xMutexState);

      switch (cmd) {
        case 1: Motion_Play();    break;
        case 2: Motion_Wander();  break;
        case 3: Motion_Tired();   break;
        case 4: Motion_GoAhead(); break;
      }

      xSemaphoreTake(xMutexState, portMAX_DELAY);
      motion = 0;
      xSemaphoreGive(xMutexState);
    }
  }
}

//低优先级：串口和LED闪烁的任务
void Task_Periph(void *pvParameters) {
  (void)pvParameters;
  for (;;) 
  {
    //LED的闪烁控制
    xSemaphoreTake(xMutexState, portMAX_DELAY);
    bool ledOn = LedLightOn;            //Semaphore安全取用全局变量
    unsigned long ledStart = LedStartMs; //尽量不使用全局变量，降低耦合度
    xSemaphoreGive(xMutexState);

    if (ledOn) 
    {
      unsigned long now = millis();
      if (now - ledStart < LED_BLINK_TOTAL) 
      {
        if (now - ledStart < INTERVAL)
          digitalWrite(LED_PIN, HIGH);
        else
          digitalWrite(LED_PIN, LOW);
      } 
      else 
      {
        digitalWrite(LED_PIN, LOW);
        xSemaphoreTake(xMutexState, portMAX_DELAY);
        LedLightOn = false;
        xSemaphoreGive(xMutexState);
      }
    }

    // 2. 串口命令解析
    if (Serial.available()) 
    {
      String line = Serial.readStringUntil('\n');
      line.trim();
      int spacePos = line.indexOf(' ');
      if (spacePos == -1) 
      {
        Serial.println("格式错误!示例:setmot 1");
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      String cmd = line.substring(0, spacePos);
      String argStr = line.substring(spacePos + 1);
      argStr.trim();

      if (cmd == "setmot" && !readSleepSafe()) 
      {
        int newmot = argStr.toInt();
        if (newmot >= 0 && newmot <= 4) 
        {
          xQueueSend(xQueueServoCmd, &newmot, 0);
          Serial.print("动作已经设置为:");
          Serial.println(newmot);
        } 
        else 
        {
          Serial.println("参数非法!动作范围为0-4!");
        }
      } 
      else if (cmd == "setemo" && !readSleepSafe()) 
      {
        int newemo = argStr.toInt();
        if (newemo >= 1 && newemo <= 10) 
        {
          writeMoodSafe(newemo);
          Serial.print("心情已经设置为:");
          Serial.println(newemo);
        } 
        else 
        {
          Serial.println("参数非法!心情范围为1-10!");
        }
      } 
      else if (cmd == "setful" && !readSleepSafe()) 
      {
        if (argStr == "true") 
        {
          writeHungrySafe(false);
          xSemaphoreTake(xMutexState, portMAX_DELAY);
          FullStart = millis();
          xSemaphoreGive(xMutexState);
          Serial.println("已设置为饱腹状态");
        } 
        else if (argStr == "false") 
        {
          writeHungrySafe(true);
          Serial.println("已设置为饥饿状态");
        } 
        else 
        {
          Serial.println("参数非法!只能设置为true或false!");
        }
      } 
      else if (cmd == "setslp") 
      {
        if (argStr == "true" && !readSleepSafe()) 
        {
          writeSleepSafe(true);
          xTimerStop(xTimerMood, 0);
          xTimerStop(xTimerMotion, 0);
          myServo1.write(170);
          myServo2.write(170);
          Serial.println("设置睡眠状态完成!");
        } 
        else if (argStr == "false" && readSleepSafe()) 
        {
          wakeupFlag = true;
          Serial.println("设置唤醒状态完成!");
        } 
        else 
        {
          Serial.println("参数非法!只能设置为true或false!");
        }
      } 
      else 
      {
        if (!readSleepSafe())
          Serial.println("命令非法!可采用setemo setful setmot和setslp!");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void Task_OLED(void *pvParameters) {
  (void)pvParameters;

  // 初始化I2C与屏幕
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) 
  {
    // 初始化失败就死循环，串口会看不到输出，方便排查
    for(;;) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  vTaskDelay(pdMS_TO_TICKS(500));

  for(;;) 
  {
    //加锁后读取全部的重要数据
    int local_mood;
    bool local_isHungry;
    unsigned long local_fullStart;
    int local_motion;
    bool local_isSleeping;

    xSemaphoreTake(xMutexState, portMAX_DELAY);
    local_mood = mood;
    local_isHungry = isHungry;
    local_fullStart = FullStart;
    local_motion = motion;
    local_isSleeping = isSleeping;
    xSemaphoreGive(xMutexState);
    //释放锁

    display.clearDisplay();

    // 睡眠状态：显示专属界面
    if (local_isSleeping) 
    {
      display.setTextSize(2);
      display.setCursor(22, 20);
      display.print("Sleeping");
      display.setTextSize(1);
      display.setCursor(30, 48);
      display.print("Z z z z...");
    }
    // 正常状态：显示完整信息
    else {
      // 第1行：心情值
      display.setCursor(0, 0);
      display.print("Mood: ");
      display.print(local_mood);
      display.print(" / 10");

      // 第2行：饥饿/饱腹状态
      display.setCursor(0, 16);
      display.print("State: ");
      if (local_isHungry) 
      {
        display.print("Hungry");
      } 
      else 
      {
        display.print("Full");
      }

      // 第3行：饱腹剩余时间
      display.setCursor(0, 32);
      display.print("Remain: ");
      if (!local_isHungry) 
      {
        // 计算剩余秒数
        long remain = (FULL_TIME - (millis() - local_fullStart)) / 1000;
        if (remain < 0) remain = 0;
        display.print(remain);
        display.print("s");
      } 
      else 
      {
        display.print("--");
      }

      // 第4行：动作状态
      display.setCursor(0, 48);
      display.print("Motion: ");
      switch(local_motion) 
      {
        case 0: display.print("NULL");   break;
        case 1: display.print("PLAY");   break;
        case 2: display.print("IDLE"); break;
        case 3: display.print("TIRE");  break;
        case 4: display.print("FOWD");     break;
        default: display.print("???");
      }
    }

    // 真正刷新到屏幕硬件
    display.display();

    // 300ms刷新一次，只阻塞本任务
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (Serial.read() != -1);

  // 1. 初始化GPIO
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LIGHT_PIN, INPUT);
  digitalWrite(LED_PIN, LOW);

  // 2. 创建RTOS内核对象
  xQueueServoCmd   = xQueueCreate(3, sizeof(int));          // 舵机命令队列，最多缓存3条命令
  xSemButton       = xSemaphoreCreateBinary();              // 按键二值信号量
  xSemMoodTick     = xSemaphoreCreateBinary();              // 心情定时器信号量
  xSemMotionTick   = xSemaphoreCreateBinary();              // 动作定时器信号量
  xMutexState      = xSemaphoreCreateMutex();               // 状态互斥量

  // 3. 创建软件定时器
  xTimerMood   = xTimerCreate("MoodTimer",   pdMS_TO_TICKS(30000), pdTRUE, 0, vTimerMoodCallback);
  xTimerMotion = xTimerCreate("MotionTimer", pdMS_TO_TICKS(1000),  pdTRUE, 0, vTimerMotionCallback);

  // 4. 初始化硬件外设
  myServo1.setPeriodHertz(50);
  myServo2.setPeriodHertz(50);
  myServo1.attach(SERVO_PIN1, 500, 2500);
  myServo2.attach(SERVO_PIN2, 500, 2500);
  ServoBackMid();

  encoder.attachHalfQuad(ENC_A_GPIO, ENC_B_GPIO);
  encoder.setCount(0);

  // 5. 创建任务（优先级：输入3 > 状态2 = 舵机2 > 外设1 > OLED显示）
  xTaskCreatePinnedToCore(Task_Input,   "TaskInput",   2048, NULL, 3, &TaskInput_Handle,   0);
  xTaskCreatePinnedToCore(Task_State,   "TaskState",   2048, NULL, 2, &TaskState_Handle,   0);
  xTaskCreatePinnedToCore(Task_Servo,   "TaskServo",   2048, NULL, 2, &TaskServo_Handle,   1);
  xTaskCreatePinnedToCore(Task_Periph,  "TaskPeriph",  3072, NULL, 1, &TaskPeriph_Handle,  1);
  xTaskCreatePinnedToCore(Task_OLED,    "TaskOLED",    3072, NULL, 1, NULL, 1);

  // 6. 启动定时器
  xTimerStart(xTimerMood,   0);
  xTimerStart(xTimerMotion, 0);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  //判断
  if (wakeup_reason != ESP_SLEEP_WAKEUP_UNDEFINED) 
  {
    // 从深睡唤醒 → 读Flash恢复状态
    Serial.println("[SYSTEM] Wake up from deep sleep!");
    loadStateFromFlash();
    bootMs += millis();  // 累计运行时长
  } 
  else 
  {
    Serial.println("[SYSTEM] Cold boot!");
    bootMs = 0;
    headTouchCount = 0;
    feedCount = 0;
  }

  if (wasSleeping) 
  {
    // 低光照睡眠唤醒 → 保持睡眠姿态，不回中
    myServo1.write(170);
    myServo2.write(170);
    wasSleeping = false; // 用完重置标记
    isSleeping = true;
  } 
  else 
  {
    // 待机唤醒/冷启动 → 舵机回中立位
    ServoBackMid();
  }

  lastInteract = millis();
  Serial.println("[SYSTEM] Init done!");

  randomSeed(analogRead(0)); // 初始化随机数种子
  Serial.println("桌宠RTOS系统启动完成!");
  Serial.println(bootMs);
  Serial.println(headTouchCount);
  Serial.println(feedCount);
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(pdMS_TO_TICKS(100));
}
