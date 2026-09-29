# 수동 제어 v1 구현 계약

2026-09-05. 구현 중인 별도 스케치/PC 도구의 공통 규격. 원본 펌웨어 보존. 최초 배포는 모터 출력이 항상0인 BENCH 모드이며, 주행 검증 완료를 뜻하지 않는다.

## 프레임

-115200 baud,8N1. 명시적으로 식별한 Mega2560에만 접속.
-바이트 배열: `F1 A5 TYPE LENGTH PAYLOAD CRC_LO CRC_HI`. CRC16-CCITT-FALSE(poly0x1021,init0xFFFF,xorout0,no reflection), TYPE부터 PAYLOAD끝까지 계산.
-PC→MCU: TYPE=1,LENGTH=12. PAYLOAD=`<IIBbbB`(little endian): boot_id(uint32),request_id(uint32),action(uint8),drive(int8),steer(int8),deadman(uint8).
-MCU→PC: TYPE=129,LENGTH=26. PAYLOAD=`<IIIiHBBbbbbH`: boot_id,request_id,millis,encoder_count(int32),steering_ADC(uint16),state(uint8),reason(uint8),requested_drive(int8),requested_steer(int8),actual_drive(int8),actual_steer(int8),flags(uint16).
-drive/steer는-1/0/+1만. drive+1은 기존 코드의전진,steer-1은 기존 코드의왼쪽(실제 방향 미측정). deadman은0/1만. 기존 F/B/L/R 문자 프로토콜로 구동하지 않는다.
-actions:0=DISARM,1=ARM,2=CONTROL. state:0=DISARMED,1=ARMED(논리상태, BENCH에서는 출력0).
-reason:0=BOOT,1=STOP,2=TIMEOUT,3=BAD_FRAME,4=STALE,5=INTERLOCK,6=ARM_REJECTED,7=REVERSAL,8=STEER_LIMIT,9=READY,10=ACTIVE,11=NEUTRAL.
-flags bit0=BENCH(실제 출력 잠금),bit1=DRIVE_ENABLED,bit2=STEER_ENABLED,bit3=INTERLOCK_CONFIGURED,bit4=INTERLOCK_OK,bit5=BOOT_ID_VALID. 그 외 비트0.

## MCU 동작

-50ms마다 요청/상태 프레임을 보낸다. 한 번에 한 요청만 유효하며 같은요청 재전송은 최초발행시각을 갱신하지 않는다. 요청은150ms 뒤 만료한다. 응답을1회 수락하면 소비하고 다음발행까지 재사용하지 않는다. 새 요청ID는 부호없는32비트 증가,0은 쓰지 않고 wrap이면정지/잠금. `millis()`차이는 uint32뺄셈으로 wrap안전.
-명령 출력 허용 기한은 요청 발행시각+150ms이다. 늦은 응답으로 수신시각부터150ms를 새로 부여하지 않는다. 만료 확인을 RX/출력 적용보다 먼저 한다. 프로세스/USB가 끊기면 마지막 발행시각 기준150ms 안에 논리출력0/DISARMED로 전환한다. 이는 실제 제동 거리 보장이 아니다.
-boot_id는 재부팅 전 대기 프레임과 구분되도록 EEPROM의 검증된 단조 부팅 카운터를 사용한다. 원본은 EEPROM 미사용. 처음 전부FF인 예약영역은 초기화 가능. magic/counter/inverse 검사·쓰기 후 검증 실패,0/wrap은 fail closed. EEPROM은 부팅 때만 갱신하며 바이트0~11예약을 문서화한다. 미정의/손상 데이터를 임의 초기화하고 주행 가능 상태로 만들지 않는다.
-유효CRC의DISARM은boot/request와무관하게항상정지하며긴급DISARM은요청당1회응답규칙의예외다. 그외모든명령은현재boot/request·미만료·미소비·범위검증필수. 모든DISARM경로에서이전중립확인·방향이력·ARM대기를폐기해새중립/ARM을요구한다. 오류/과부하/부분프레임만료/의미없는바이트는DISARM/출력0/요청폐기. 최대프레임32바이트,파서입력버퍼/루프당처리량유한,부분프레임30ms만료. 수신홍수도출력타이머를막지않는다.
-DISARM 이후 CONTROL(0,0,deadman0)로 중립 확인 후 별도 ARM(0,0,deadman0)만 허용. ARM은 출력0이며, 이후의 새 CONTROL과deadman1이어야 비영점요청 가능. deadman해제는DISARM. 중립확인은500ms 이내만 유효하고 부팅/오류/만료 시 폐기.
-ARM 직후 아직deadman1을수락하지않았으면CONTROL(0,0,0)로출력0의준비상태를유지할수있다(매번유효요청응답필수). 한번deadman1을수락한뒤0이되면DISARM. 비영점+deadman0은항상거부. ARMED에서CONTROL(0,0,deadman1)은정지유지가능. 반대부호의구동/조향전환은중간0을거쳐도같은ARM세션에서는DISARM시켜새ARM을요구한다(물리정지를보증하는타이머를추측하지않는다).
-중립확인 ACK는state0/reasonNEUTRAL(11),ARM성공ACK는state1/reasonREADY(9). PC는중립ACK를확인한뒤다음요청에ARM을보낸다.
-모터 출력 기본 잠금: `ACTUATION_ENABLED=false`. 요청을 계산·보고하되 모든PWM0/BRK HIGH. 실제출력허용을 켜려면 확인된 E-stop/전원허용 입력 구성과 실측 검증 필요. 임의입력핀 배정 금지, 기본-1. 조향ADC한계/증가방향 미설정이면 STEER_ENABLED=false. 774/350/590/830을 한계로 사용하지 않는다.
-구동 출력 한계는 기존 코드PWM45를 상한 후보로 보존(실속도미검증). 조향PWM60상한 후보. 조향200ms타이머는ARM세션의첫비영점조향수락때시작하며후속동일방향/중립패킷으로재시작·연장하지않는다.200ms도달시정지/DISARM/새중립확인·ARM필요. bench에서도논리타이머검사. 실제조향은설정된ADC범위와증가방향검사통과시만.
-원본GPIO:ENC_A2/ENC_B3,STEER PWM11/DIR6/BRK7/ADC A15,FRONT12/8/22,REAR44/23/24. 원본 forward LOW,steer left LOW. 부팅 시 BRK HIGH를 출력핀모드 전 미리 설정하고 PWM0. 엔코더ISR 유지,센서읽기는 원시값. Serial.availableForWrite로상태송신이 제어루프를막지 않게 한다. AVR watchdog은 별도검토 전 사용하지 않는다.

## PC 동작

-새 한국어 Tkinter 조작창, W/S/A/D, Shift를 누른 동안 조작 허용, 별도‘조작 준비’버튼,큰‘정지’버튼/Escape. BENCH 표시를 눈에 띄게 하고 ‘실제 모터 출력 잠금’과 원시센서값을 표시한다.
-구동/조향키·Shift를 놓은 상태에서만명시적ARM. ARM 성공후새 키입력만 사용. 버튼 클릭/앱시작/연결복구/포커스복귀로 자동ARM하지 않는다. Shift는수식키여서키반복의 영향을줄인다. 모든입력해제/포커스상실/닫기/예외/수신만료에서중립+DISARM,실패하면MCU만료에 맡긴다.
-입력처리와응답송신은Tk메인루프 한곳에서 실행. 마지막키를 별도송신스레드가 무한갱신하지 않는다. 루프가100ms 이상멎으면유효요청이와도DISARM. 비차단serial(read timeout0,write_timeout0),부분쓰기/대기TX가 있으면fail closed.
-호스트는 동일요청에1회만응답. 중복/오래된/잘못된상태를 정상요청으로 재사용하지 않는다. boot변경/수신150ms초과/CRC오류/역행request는중립+DISARM. 미리쌓인 상태 프레임들은최신1개만검토,루프복귀시묵은키송신금지. 수신기반latency/ACK는진단값이며물리정지검증아님.
-Arduino고정USB VID2341/PID0042/serial95431353338351A01190/by-id와열기직후정체성확인,프로젝트다른프로그램과flock배타잠금. 기존펌웨어미인식이면명령을보내지 않음(새상태프레임수신 전 송신금지). 포트를열면Mega가리셋될수있으므로현재모터OFF확인뒤에만실접속. 포트는사용자가명시한 `--connect`에서만열며기본은하드웨어없는demo.
-수신/송신원문·host시각·UI이벤트·실행소스해시를새data폴더에저장. 자격증명접근 없음.

## 검증

C++코어를호스트에서실제실행하는검사로만료/묵은응답/중복/재ARM/CRC/홍수/부분프레임/시각wrap/boot불일치/반전/조향상한/출력잠금을확인. PC는입력상실/메인루프멈춤/오래된요청/리셋/부분쓰기를가상전송으로검사. C++코어와PythonPC프레임을상호검증한다. Mega컴파일은논리검사를대신하지않으며벤치업로드후실제PWM/BRK·USB단절을확인해야한다.

최신물리상태: 사용자는21:29이후모터구동전원과차량배터리OFF를확인했고, 버튼을누르면주행/풀면OFF라고설명을정정했다. 이전E-stop설명과불일치하므로비상정지배선/버튼동작을확인하기전실제구동잠금을풀지않는다. BENCH프로토콜개발은진행가능하다.

## 2026-09-05 수신 경계 및 PC 시간 제한 보완

- 만료·인터록·조향 제한은 즉시 DISARM하고 제어 상태를 폐기한다. 그때 이미 일부 받은 프레임은 원래 30ms 부분 수신 기한 안에서 끝까지만 파싱해 버린다. 이 프레임은 중립 증명·ARM·출력 기한을 만들지 못하며, 유효한 명시적 DISARM만 처리할 수 있다. 별도 잘못된 바이트/CRC 오류의 진단은 그대로 유지한다.
- PC는 송신 직전에 이전 tick부터의 경과 시간과 현재 tick 안의 처리 시간을 함께 확인한다. 이전 tick 이후 대기 90ms와 현재 로그 처리 90ms처럼 합산 100ms를 넘긴 경우 CONTROL을 DISARM으로 바꾼다. 두 지연을 각자 100ms 이하라고 허용하지 않는다.
