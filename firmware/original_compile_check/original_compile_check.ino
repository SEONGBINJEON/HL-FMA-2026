// ======================================================
// HENES F8 통합 주행 테스트
// Arduino Mega
//
// - 앞 구동 모터
// - 뒤 구동 모터
// - 조향 모터
// - 앞 구동 엔코더
// - 조향 전위차계
// ======================================================


// ======================================================
// 1. 앞 구동 엔코더
// ======================================================

const int ENC_A = 2;
const int ENC_B = 3;

volatile long encoderCount = 0;


// ======================================================
// 2. 조향 DMD
// ======================================================

const int STEER_PWM = 11;
const int STEER_DIR = 6;
const int STEER_BRK = 7;


// ======================================================
// 3. 앞 구동 DMD
// ======================================================

const int FRONT_PWM = 12;
const int FRONT_DIR = 8;
const int FRONT_BRK = 22;


// ======================================================
// 4. 뒤 구동 DMD
// ======================================================

const int REAR_PWM = 44;
const int REAR_DIR = 23;
const int REAR_BRK = 24;


// ======================================================
// 5. 조향 전위차계
// ======================================================

const int STEER_SIGNAL = A15;


// ======================================================
// 6. 테스트 출력값
// ======================================================

// 앞/뒤 구동 출력
// 0 ~ 255
// 현재 테스트에서 65가 적당했으므로 유지
//const int DRIVE_PWM = 65;
const int DRIVE_PWM = 45;
// 조향 출력
const int STEER_PWM_VALUE = 60;

// 시리얼 한 번 입력할 때 조향하는 시간
const int STEER_TIME = 200;    // 0.2초


// ======================================================
// 7. 차량 방향 설정
// ======================================================

// 앞 구동은 현재 테스트 기준
// LOW = 전진
// HIGH = 후진
const bool FRONT_FORWARD_DIR = LOW;


// 중요!
// 뒤 구동에서 LOW가 실제 전진이면 그대로 사용.
//
// 만약 F를 눌렀을 때
// 앞바퀴는 전진, 뒷바퀴는 후진한다면
//
// LOW → HIGH로 딱 이것만 변경.
//
const bool REAR_FORWARD_DIR = LOW;


// 조향
// 이전 테스트 기준
const bool STEER_LEFT_DIR  = LOW;
const bool STEER_RIGHT_DIR = HIGH;


// ======================================================
// 8. 현재 상태 표시용
// ======================================================

String driveState = "STOP";
String steerState = "STOP";


// ======================================================
// 엔코더 인터럽트
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
// 앞 + 뒤 구동 정지
// ======================================================

void stopDrive()
{
  // PWM OFF
  analogWrite(FRONT_PWM, 0);
  analogWrite(REAR_PWM, 0);

  // DMD Brake
  digitalWrite(FRONT_BRK, HIGH);
  digitalWrite(REAR_BRK, HIGH);

  driveState = "STOP";
}


// ======================================================
// 전진
// ======================================================

void driveForward()
{
  // 방향 설정
  digitalWrite(FRONT_DIR, FRONT_FORWARD_DIR);
  digitalWrite(REAR_DIR, REAR_FORWARD_DIR);

  // Brake 해제
  digitalWrite(FRONT_BRK, LOW);
  digitalWrite(REAR_BRK, LOW);

  // 앞 / 뒤 같은 PWM
  analogWrite(FRONT_PWM, DRIVE_PWM);
  analogWrite(REAR_PWM, DRIVE_PWM);

  driveState = "FORWARD";
}


// ======================================================
// 후진
// ======================================================

void driveBackward()
{
  // 전진 방향의 반대
  digitalWrite(FRONT_DIR, !FRONT_FORWARD_DIR);
  digitalWrite(REAR_DIR, !REAR_FORWARD_DIR);

  // Brake 해제
  digitalWrite(FRONT_BRK, LOW);
  digitalWrite(REAR_BRK, LOW);

  // 앞 / 뒤 같은 PWM
  analogWrite(FRONT_PWM, DRIVE_PWM);
  analogWrite(REAR_PWM, DRIVE_PWM);

  driveState = "BACKWARD";
}


// ======================================================
// 조향 정지
// ======================================================

void stopSteering()
{
  analogWrite(STEER_PWM, 0);

  digitalWrite(STEER_BRK, HIGH);

  steerState = "STOP";
}


// ======================================================
// 왼쪽 조향
// ======================================================

void steerLeft()
{
  steerState = "LEFT";

  digitalWrite(STEER_DIR, STEER_LEFT_DIR);
  digitalWrite(STEER_BRK, LOW);

  analogWrite(STEER_PWM, STEER_PWM_VALUE);

  // 짧게만 조향
  delay(STEER_TIME);

  stopSteering();
}


// ======================================================
// 오른쪽 조향
// ======================================================

void steerRight()
{
  steerState = "RIGHT";

  digitalWrite(STEER_DIR, STEER_RIGHT_DIR);
  digitalWrite(STEER_BRK, LOW);

  analogWrite(STEER_PWM, STEER_PWM_VALUE);

  // 짧게만 조향
  delay(STEER_TIME);

  stopSteering();
}


// ======================================================
// 전체 긴급 정지
// ======================================================

void emergencyStop()
{
  stopDrive();
  stopSteering();

  Serial.println();
  Serial.println("================================");
  Serial.println("!!! EMERGENCY STOP !!!");
  Serial.println("모든 모터 정지");
  Serial.println("================================");
}


// ======================================================
// 엔코더 초기화
// ======================================================

void resetEncoder()
{
  noInterrupts();

  encoderCount = 0;

  interrupts();

  Serial.println("[ENCODER RESET] Count = 0");
}


// ======================================================
// 현재 상태 출력
// ======================================================

void printStatus()
{
  noInterrupts();

  long count = encoderCount;

  interrupts();

  int steeringValue = analogRead(STEER_SIGNAL);


  Serial.println();
  Serial.println("--------------- STATUS ---------------");

  Serial.print("DRIVE      : ");
  Serial.println(driveState);

  Serial.print("STEERING   : ");
  Serial.println(steerState);

  Serial.print("ENCODER    : ");
  Serial.println(count);

  Serial.print("ENC A / B  : ");
  Serial.print(digitalRead(ENC_A));
  Serial.print(" / ");
  Serial.println(digitalRead(ENC_B));

  Serial.print("STEER POT  : ");
  Serial.println(steeringValue);

  Serial.println("--------------------------------------");
}


// ======================================================
// 사용 가능한 명령 출력
// ======================================================

void printMenu()
{
  Serial.println();
  Serial.println("=========================================");
  Serial.println("      HENES F8 DRIVE TEST");
  Serial.println("=========================================");
  Serial.println();
  Serial.println("[DRIVE]");
  Serial.println("F : Forward  (앞 + 뒤 전진)");
  Serial.println("B : Backward (앞 + 뒤 후진)");
  Serial.println("S : Stop     (앞 + 뒤 정지)");
  Serial.println();
  Serial.println("[STEERING]");
  Serial.println("L : Left     (왼쪽 조향)");
  Serial.println("R : Right    (오른쪽 조향)");
  Serial.println("C : Center/Steering STOP");
  Serial.println();
  Serial.println("[ETC]");
  Serial.println("E : Encoder Count Reset");
  Serial.println("P : Print Status");
  Serial.println("Z : EMERGENCY STOP");
  Serial.println();
  Serial.println("=========================================");
}


// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);


  // ----------------------------------------------------
  // 엔코더
  // ----------------------------------------------------

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    encoderISR,
    CHANGE
  );


  // ----------------------------------------------------
  // 조향 DMD
  // ----------------------------------------------------

  pinMode(STEER_PWM, OUTPUT);
  pinMode(STEER_DIR, OUTPUT);
  pinMode(STEER_BRK, OUTPUT);


  // ----------------------------------------------------
  // 앞 구동 DMD
  // ----------------------------------------------------

  pinMode(FRONT_PWM, OUTPUT);
  pinMode(FRONT_DIR, OUTPUT);
  pinMode(FRONT_BRK, OUTPUT);


  // ----------------------------------------------------
  // 뒤 구동 DMD
  // ----------------------------------------------------

  pinMode(REAR_PWM, OUTPUT);
  pinMode(REAR_DIR, OUTPUT);
  pinMode(REAR_BRK, OUTPUT);


  // ----------------------------------------------------
  // 조향 전위차계
  // ----------------------------------------------------

  pinMode(STEER_SIGNAL, INPUT);


  // ----------------------------------------------------
  // 부팅 시 모든 모터 정지
  // ----------------------------------------------------

  stopDrive();
  stopSteering();


  delay(500);

  printMenu();
}


// ======================================================
// LOOP
// ======================================================

void loop()
{
  // ====================================================
  // 시리얼 명령
  // ====================================================

  if (Serial.available() > 0)
  {
    char command = Serial.read();


    // --------------------------------------------------
    // 전진
    // --------------------------------------------------

    if (command == 'F' || command == 'f')
    {
      driveForward();

      Serial.println();
      Serial.println("[DRIVE] >>> FORWARD");
    }


    // --------------------------------------------------
    // 후진
    // --------------------------------------------------

    else if (command == 'B' || command == 'b')
    {
      driveBackward();

      Serial.println();
      Serial.println("[DRIVE] <<< BACKWARD");
    }


    // --------------------------------------------------
    // 주행 정지
    // --------------------------------------------------

    else if (command == 'S' || command == 's')
    {
      stopDrive();

      Serial.println();
      Serial.println("[DRIVE] STOP");
    }


    // --------------------------------------------------
    // 왼쪽 조향
    // --------------------------------------------------

    else if (command == 'L' || command == 'l')
    {
      Serial.println();
      Serial.println("[STEERING] LEFT");

      steerLeft();

      Serial.print("Steering Position = ");
      Serial.println(analogRead(STEER_SIGNAL));
    }


    // --------------------------------------------------
    // 오른쪽 조향
    // --------------------------------------------------

    else if (command == 'R' || command == 'r')
    {
      Serial.println();
      Serial.println("[STEERING] RIGHT");

      steerRight();

      Serial.print("Steering Position = ");
      Serial.println(analogRead(STEER_SIGNAL));
    }


    // --------------------------------------------------
    // 조향 정지
    // --------------------------------------------------

    else if (command == 'C' || command == 'c')
    {
      stopSteering();

      Serial.println();
      Serial.println("[STEERING] STOP");
    }


    // --------------------------------------------------
    // 엔코더 초기화
    // --------------------------------------------------

    else if (command == 'E' || command == 'e')
    {
      resetEncoder();
    }


    // --------------------------------------------------
    // 현재 상태
    // --------------------------------------------------

    else if (command == 'P' || command == 'p')
    {
      printStatus();
    }


    // --------------------------------------------------
    // 긴급 정지
    // --------------------------------------------------

    else if (command == 'Z' || command == 'z')
    {
      emergencyStop();
    }
  }


  // ====================================================
  // 0.5초마다 현재 센서값 자동 출력
  // ====================================================

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 500)
  {
    lastPrint = millis();


    noInterrupts();

    long count = encoderCount;

    interrupts();


    int steeringValue = analogRead(STEER_SIGNAL);


    Serial.print("[MONITOR] ");

    Serial.print("Drive=");
    Serial.print(driveState);

    Serial.print(" | Encoder=");
    Serial.print(count);

    Serial.print(" | Steering=");
    Serial.println(steeringValue);
  }
}
