// ======================================================
// HENES F8
// JOYSTICK + ULTRASONIC SAFETY CONTROL
// Arduino Mega 2560
// ======================================================


// ======================================================
// FRONT ENCODER
// ======================================================

const int ENC_A = 2;
const int ENC_B = 3;

volatile long encoderCount = 0;


// ======================================================
// STEERING DMD
// ======================================================

const int STEER_PWM = 11;
const int STEER_DIR = 6;
const int STEER_BRK = 7;


// ======================================================
// FRONT DRIVE DMD
// ======================================================

const int FRONT_PWM = 12;
const int FRONT_DIR = 8;
const int FRONT_BRK = 22;


// ======================================================
// REAR DRIVE DMD
// ======================================================

const int REAR_PWM = 44;
const int REAR_DIR = 23;
const int REAR_BRK = 24;


// ======================================================
// STEERING POTENTIOMETER
// ======================================================

const int STEER_POT = A15;


// ======================================================
// ULTRASONIC SENSOR
// ======================================================

// 왼쪽
const int US_LEFT_TRIG = 53;
const int US_LEFT_ECHO = 49;

// 중앙
const int US_CENTER_TRIG = 41;
const int US_CENTER_ECHO = 39;

// 오른쪽
const int US_RIGHT_TRIG = 37;
const int US_RIGHT_ECHO = 35;


// ======================================================
// VEHICLE SETTING
// ======================================================

// 현재 실차에서 확인된 주행 출력
const int DRIVE_PWM = 45;

// 조향 출력
const int STEER_PWM_VALUE = 60;

// 조향 한 번 입력 시 구동 시간
const unsigned long STEER_PULSE_MS = 90;


// ======================================================
// ULTRASONIC SAFETY SETTING
// ======================================================

// 50cm 이하부터 위험
const float STOP_DISTANCE_CM = 50.0;

// 한번 정지한 뒤 이 거리 이상이어야 안전하다고 판단
const float CLEAR_DISTANCE_CM = 65.0;

// 동일 센서 3회 연속 검출
const int DANGER_HITS_REQUIRED = 3;

// 3번 연속 안전거리 확인
const int CLEAR_HITS_REQUIRED = 3;


// ======================================================
// DIRECTION
// ======================================================

// 현재 통합 테스트에서 확인된 방향
const bool FRONT_FORWARD_DIR = LOW;
const bool REAR_FORWARD_DIR = LOW;

const bool STEER_LEFT_DIR = LOW;
const bool STEER_RIGHT_DIR = HIGH;


// ======================================================
// COMMUNICATION FAIL-SAFE
// ======================================================

// Python / USB 통신이 끊기면 0.7초 뒤 모든 모터 정지
const unsigned long PC_TIMEOUT_MS = 700;

unsigned long lastPcSignal = 0;


// ======================================================
// STATE
// ======================================================

char driveState = 'S';

bool frontBlocked = false;

// 초음파 E-STOP 이후
// 조이스틱을 중립에 한번 놓게 하기 위한 변수
bool neutralRequired = false;

// A 버튼으로 작동시키는 수동 E-STOP
bool manualEstop = false;


// 초음파 연속 감지 횟수
int leftDangerCount = 0;
int centerDangerCount = 0;
int rightDangerCount = 0;

int clearCount = 0;


// 최근 거리
float leftDistance = 999;
float centerDistance = 999;
float rightDistance = 999;


// 조향 Pulse
bool steeringActive = false;
unsigned long steeringStopTime = 0;


// ======================================================
// ENCODER INTERRUPT
// ======================================================

void encoderISR()
{
  int A = digitalRead(ENC_A);
  int B = digitalRead(ENC_B);

  if (A == B)
  {
    encoderCount++;
  }
  else
  {
    encoderCount--;
  }
}


// ======================================================
// ULTRASONIC DISTANCE
// ======================================================

float readUltrasonic(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  // 약 1m 조금 넘는 범위까지만 기다림
  unsigned long duration =
      pulseIn(echoPin, HIGH, 7000);

  // Echo가 없으면 아주 먼 거리로 취급
  if (duration == 0)
  {
    return 999.0;
  }

  float distance =
      duration * 0.0343 / 2.0;

  return distance;
}


// ======================================================
// DRIVE STOP
// ======================================================

void stopDrive()
{
  analogWrite(FRONT_PWM, 0);
  analogWrite(REAR_PWM, 0);

  digitalWrite(FRONT_BRK, HIGH);
  digitalWrite(REAR_BRK, HIGH);

  driveState = 'S';
}


// ======================================================
// FORWARD
// ======================================================

void driveForward()
{
  // 수동 비상정지
  if (manualEstop)
  {
    stopDrive();
    return;
  }

  // 초음파 장애물
  if (frontBlocked)
  {
    stopDrive();
    return;
  }

  // 장애물 제거 후 조이스틱 중립 확인 전
  if (neutralRequired)
  {
    stopDrive();
    return;
  }

  digitalWrite(
    FRONT_DIR,
    FRONT_FORWARD_DIR
  );

  digitalWrite(
    REAR_DIR,
    REAR_FORWARD_DIR
  );

  digitalWrite(FRONT_BRK, LOW);
  digitalWrite(REAR_BRK, LOW);

  analogWrite(
    FRONT_PWM,
    DRIVE_PWM
  );

  analogWrite(
    REAR_PWM,
    DRIVE_PWM
  );

  driveState = 'F';
}


// ======================================================
// BACKWARD
// ======================================================

void driveBackward()
{
  // A 버튼 비상정지만 후진까지 금지
  if (manualEstop)
  {
    stopDrive();
    return;
  }

  // 초음파는 전방 센서이므로
  // 전방 장애물이 있어도 후진은 허용

  digitalWrite(
    FRONT_DIR,
    !FRONT_FORWARD_DIR
  );

  digitalWrite(
    REAR_DIR,
    !REAR_FORWARD_DIR
  );

  digitalWrite(FRONT_BRK, LOW);
  digitalWrite(REAR_BRK, LOW);

  analogWrite(
    FRONT_PWM,
    DRIVE_PWM
  );

  analogWrite(
    REAR_PWM,
    DRIVE_PWM
  );

  driveState = 'B';
}


// ======================================================
// STEERING STOP
// ======================================================

void stopSteering()
{
  analogWrite(STEER_PWM, 0);

  digitalWrite(
    STEER_BRK,
    HIGH
  );

  steeringActive = false;
}


// ======================================================
// STEERING PULSE
// ======================================================

void startSteering(bool direction)
{
  if (manualEstop)
  {
    stopSteering();
    return;
  }

  digitalWrite(
    STEER_DIR,
    direction
  );

  digitalWrite(
    STEER_BRK,
    LOW
  );

  analogWrite(
    STEER_PWM,
    STEER_PWM_VALUE
  );

  steeringActive = true;

  steeringStopTime =
      millis() + STEER_PULSE_MS;
}


// ======================================================
// MANUAL ESTOP
// ======================================================

void manualEmergencyStop()
{
  manualEstop = true;

  stopDrive();
  stopSteering();

  Serial.println();
  Serial.println("=========================");
  Serial.println("!!! MANUAL E-STOP !!!");
  Serial.println("=========================");
}


// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);


  // --------------------------
  // Encoder
  // --------------------------

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    encoderISR,
    CHANGE
  );


  // --------------------------
  // Steering DMD
  // --------------------------

  pinMode(STEER_PWM, OUTPUT);
  pinMode(STEER_DIR, OUTPUT);
  pinMode(STEER_BRK, OUTPUT);


  // --------------------------
  // Front DMD
  // --------------------------

  pinMode(FRONT_PWM, OUTPUT);
  pinMode(FRONT_DIR, OUTPUT);
  pinMode(FRONT_BRK, OUTPUT);


  // --------------------------
  // Rear DMD
  // --------------------------

  pinMode(REAR_PWM, OUTPUT);
  pinMode(REAR_DIR, OUTPUT);
  pinMode(REAR_BRK, OUTPUT);


  // --------------------------
  // Steering POT
  // --------------------------

  pinMode(STEER_POT, INPUT);


  // --------------------------
  // Ultrasonic
  // --------------------------

  pinMode(
    US_LEFT_TRIG,
    OUTPUT
  );

  pinMode(
    US_LEFT_ECHO,
    INPUT
  );


  pinMode(
    US_CENTER_TRIG,
    OUTPUT
  );

  pinMode(
    US_CENTER_ECHO,
    INPUT
  );


  pinMode(
    US_RIGHT_TRIG,
    OUTPUT
  );

  pinMode(
    US_RIGHT_ECHO,
    INPUT
  );


  // --------------------------
  // Initial STOP
  // --------------------------

  stopDrive();
  stopSteering();

  lastPcSignal = millis();


  Serial.println();
  Serial.println("==============================");
  Serial.println(" F8 JOYSTICK CONTROL READY");
  Serial.println("==============================");

  Serial.println("F : Forward");
  Serial.println("B : Backward");
  Serial.println("S : Drive Stop");

  Serial.println("L : Steering Left");
  Serial.println("R : Steering Right");
  Serial.println("C : Steering Stop");

  Serial.println("X : Manual E-STOP");
  Serial.println("U : Manual E-STOP Release");
  Serial.println("H : PC Heartbeat");

  Serial.println("==============================");
}


// ======================================================
// LOOP
// ======================================================

void loop()
{
  unsigned long now = millis();


  // ====================================================
  // 1. 조향 Pulse 자동 종료
  // ====================================================

  if (
      steeringActive &&
      now >= steeringStopTime
     )
  {
    stopSteering();
  }


  // ====================================================
  // 2. 초음파 측정
  // ====================================================

  static unsigned long lastUltrasonic = 0;

  if (now - lastUltrasonic >= 30)
  {
    lastUltrasonic = now;


    // LEFT
    leftDistance =
        readUltrasonic(
          US_LEFT_TRIG,
          US_LEFT_ECHO
        );

    delay(8);


    // CENTER
    centerDistance =
        readUltrasonic(
          US_CENTER_TRIG,
          US_CENTER_ECHO
        );

    delay(8);


    // RIGHT
    rightDistance =
        readUltrasonic(
          US_RIGHT_TRIG,
          US_RIGHT_ECHO
        );


    // ==================================================
    // 각 센서별 50cm 이하 연속 감지
    // ==================================================

    if (
        leftDistance >= 2 &&
        leftDistance <= STOP_DISTANCE_CM
       )
    {
      leftDangerCount++;
    }
    else
    {
      leftDangerCount = 0;
    }


    if (
        centerDistance >= 2 &&
        centerDistance <= STOP_DISTANCE_CM
       )
    {
      centerDangerCount++;
    }
    else
    {
      centerDangerCount = 0;
    }


    if (
        rightDistance >= 2 &&
        rightDistance <= STOP_DISTANCE_CM
       )
    {
      rightDangerCount++;
    }
    else
    {
      rightDangerCount = 0;
    }


    bool dangerConfirmed =
        leftDangerCount >= DANGER_HITS_REQUIRED ||
        centerDangerCount >= DANGER_HITS_REQUIRED ||
        rightDangerCount >= DANGER_HITS_REQUIRED;


    // ==================================================
    // 장애물 확정
    // ==================================================

    if (dangerConfirmed)
    {
      if (!frontBlocked)
      {
        frontBlocked = true;

        Serial.println();
        Serial.println(
          "!!! ULTRASONIC OBSTACLE DETECTED !!!"
        );
      }


      // 실제 전진 중이면 즉시 정지
      if (driveState == 'F')
      {
        stopDrive();

        // 장애물이 없어져도 바로 재출발 방지
        neutralRequired = true;

        Serial.println(
          ">>> EMERGENCY STOP <<<"
        );
      }
    }


    // ==================================================
    // 장애물이 사라졌는지 확인
    // ==================================================

    bool allClear =
        leftDistance > CLEAR_DISTANCE_CM &&
        centerDistance > CLEAR_DISTANCE_CM &&
        rightDistance > CLEAR_DISTANCE_CM;


    if (allClear)
    {
      clearCount++;

      if (clearCount >= CLEAR_HITS_REQUIRED)
      {
        if (frontBlocked)
        {
          Serial.println(
            "[ULTRASONIC] Path Clear"
          );
        }

        frontBlocked = false;
      }
    }
    else
    {
      clearCount = 0;
    }
  }


  // ====================================================
  // 3. Python / Joystick Serial Command
  // ====================================================

  while (Serial.available() > 0)
  {
    char command = Serial.read();

    lastPcSignal = millis();


    // --------------------------------------------------
    // FORWARD
    // --------------------------------------------------

    if (command == 'F')
    {
      driveForward();
    }


    // --------------------------------------------------
    // BACKWARD
    // --------------------------------------------------

    else if (command == 'B')
    {
      driveBackward();
    }


    // --------------------------------------------------
    // DRIVE STOP / JOYSTICK NEUTRAL
    // --------------------------------------------------

    else if (command == 'S')
    {
      stopDrive();

      // 장애물이 이미 사라진 상태에서
      // 조이스틱 중립을 한번 거치면 재출발 허용
      if (!frontBlocked)
      {
        neutralRequired = false;
      }
    }


    // --------------------------------------------------
    // LEFT
    // --------------------------------------------------

    else if (command == 'L')
    {
      startSteering(
        STEER_LEFT_DIR
      );
    }


    // --------------------------------------------------
    // RIGHT
    // --------------------------------------------------

    else if (command == 'R')
    {
      startSteering(
        STEER_RIGHT_DIR
      );
    }


    // --------------------------------------------------
    // STEERING STOP
    // --------------------------------------------------

    else if (command == 'C')
    {
      stopSteering();
    }


    // --------------------------------------------------
    // MANUAL E-STOP
    // --------------------------------------------------

    else if (command == 'X')
    {
      manualEmergencyStop();
    }


    // --------------------------------------------------
    // E-STOP RELEASE
    // --------------------------------------------------

    else if (command == 'U')
    {
      manualEstop = false;

      stopDrive();
      stopSteering();

      Serial.println(
        "[MANUAL E-STOP RELEASED]"
      );
    }


    // --------------------------------------------------
    // HEARTBEAT
    // --------------------------------------------------

    else if (command == 'H')
    {
      // 아무 동작 없음.
      // PC 연결 확인용
    }
  }


  // ====================================================
  // 4. PC/Python 통신 끊김 FAIL-SAFE
  // ====================================================

  if (
      millis() - lastPcSignal >
      PC_TIMEOUT_MS
     )
  {
    stopDrive();
    stopSteering();
  }


  // ====================================================
  // 5. Monitor
  // ====================================================

  static unsigned long lastPrint = 0;

  if (
      millis() - lastPrint >= 300
     )
  {
    lastPrint = millis();


    noInterrupts();

    long count = encoderCount;

    interrupts();


    Serial.print("[MONITOR] ");

    Serial.print("US=");
    Serial.print(leftDistance);
    Serial.print("/");
    Serial.print(centerDistance);
    Serial.print("/");
    Serial.print(rightDistance);

    Serial.print(" cm");

    Serial.print(" | ENC=");
    Serial.print(count);

    Serial.print(" | POT=");
    Serial.print(
      analogRead(STEER_POT)
    );

    Serial.print(" | BLOCK=");
    Serial.print(frontBlocked);

    Serial.print(" | ESTOP=");
    Serial.println(manualEstop);
  }

}
