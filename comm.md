# 로봇 A(자율주행) ↔ 로봇 B(매니퓰레이터) 시리얼 플래그 통신 프로토콜
> 현재 사용하지 않음. 2026.07 기준 절대 이를 참조하지 말것.

## 0. 목적
- 로봇 A(자율주행 컴퓨터)가 워크스테이션 도착/작업 지시를 전송한다.
- 로봇 B(매니퓰레이터 컴퓨터)가 작업 수행 후 완료/실패 결과를 회신한다.
- 플래그 기반의 단순·명확한 상태 동기화를 목표로 한다.

---

## 1. 통신 계층(Transport)
### 1.1 시리얼
- 물리 채널: UART(USB-Serial 포함)
- Baudrate: 115200 (기본)
- Timeout: 0.1s (기본)
- 메시지 단위: 라인(Line) 단위
- Line ending: `\n` (기본, 송신 시 자동 부착)

### 1.2 ROS2 ↔ Serial 브릿지 토픽 (serial_flag_bridge.py)
- 송신(TX) 토픽: `serial_tx` (std_msgs/String)
- 수신(RX) 토픽: `serial_rx` (std_msgs/String)

> 로봇 A에서 `serial_tx`로 publish → 시리얼로 전송 → 로봇 B의 `serial_rx`로 수신  
> 로봇 B에서 `serial_tx`로 publish → 시리얼로 전송 → 로봇 A의 `serial_rx`로 수신

---

## 2. 메시지 포맷(Format)
### 2.1 기본 형식 (CSV-like)
- 구분자: `,`
- 공백: 사용하지 않음
- 전체 형태:
  - `TYPE,WS,TASK`

### 2.2 필드 정의
- `TYPE` : 메시지 유형
  - `ARRIVED` : (A→B) 워크스테이션 도착 및 작업 요청
  - `DONE`    : (B→A) 작업 성공 완료
- `WS` : 워크스테이션 ID
  - 예: `WS1`, `WS2`
- `TASK` : 수행 작업
  - `PICK3`  : 물건 3개 집어서 적재
  - `PLACE3` : 적재함에서 물건 3개 내려놓기
---

## 3. 메시지 종류 및 의미
### 3.1 A → B (자율주행 → 매니퓰레이터)
- `ARRIVED,WSx,PICK3`
  - 의미: 로봇 A가 WSx에 도착했으니, 로봇 B는 물건 3개를 집어 적재를 수행한다.
- `ARRIVED,WSx,PLACE3`
  - 의미: 로봇 A가 WSx에 도착했으니, 로봇 B는 물건 3개를 내려놓는 하역을 수행한다.

### 3.2 B → A (매니퓰레이터 → 자율주행)
- `DONE,WSx,PICK3`
  - 의미: WSx에서 PICK3 작업 성공 완료
- `DONE,WSx,PLACE3`
  - 의미: WSx에서 PLACE3 작업 성공 완료

---

## 4. 상태 머신 동작 규칙
### 4.1 로봇 A(자율주행) 규칙
1) Nav2로 WSx 도착 감지
2) 수행할 작업이 PICK이면:
   - `serial_tx`에 `ARRIVED,WSx,PICK3` publish
3) 수행할 작업이 PLACE이면:
   - `serial_tx`에 `ARRIVED,WSx,PLACE3` publish
4) 이후 `serial_rx`에서 아래 중 하나를 대기
   - `DONE,WSx,<TASK>` 수신 → 다음 워크스테이션으로 이동
   - `FAIL,WSx,<TASK>,<REASON>` 수신 → 정책에 따라 재시도/정지/스킵

### 4.2 로봇 B(매니퓰레이터) 규칙
1) `serial_rx`에서 `ARRIVED,WSx,TASK` 수신
2) TASK에 맞는 시퀀스 실행
   - PICK3: 물체 3개 집기 → 적재함 적재
   - PLACE3: 적재함에서 3개 꺼내기 → 워크스테이션 내려놓기
3) 성공 시:
   - `serial_tx`에 `DONE,WSx,TASK` publish
4) 실패 시:
   - `serial_tx`에 `FAIL,WSx,TASK,REASON` publish

---

## 5. 시나리오 매핑(요구사항 그대로)
### 5.1 기본 루프
- (1) WS1 도착(빈 상태) → PICK3
  - A→B: `ARRIVED,WS1,PICK3`
  - B→A: `DONE,WS1,PICK3`
- (3) WS2 이동 → PLACE3
  - A→B: `ARRIVED,WS2,PLACE3`
  - B→A: `DONE,WS2,PLACE3`
- (5) 동일 WS2에서 다시 PICK3
  - A→B: `ARRIVED,WS2,PICK3`
  - B→A: `DONE,WS2,PICK3`
- 이후 다음 워크스테이션으로 반복

---

## 6. 예시(Examples)
### 6.1 A가 WS1 도착 후 집기 요청
- 송신(A → serial_tx):
  - `ARRIVED,WS1,PICK3`

### 6.2 B가 집기 완료 후 완료 회신
- 송신(B → serial_tx):
  - `DONE,WS1,PICK3`

### 6.3 B가 물체 미검출로 실패 회신
- 송신(B → serial_tx):
  - `FAIL,WS1,PICK3,NO_OBJECT`

---

## 7. 운영 규칙(권장)
- A는 `DONE` 수신 전까지 다음 워크스페이스로 이동하지 않는다.
- B는 동시에 두 작업을 수행하지 않는다(단일 명령 처리).

---

## 8. ROS CLI 테스트
- A에서 B로 명령 전송(시리얼 송신):
  - `ros2 topic pub /serial_tx std_msgs/msg/String "{data: 'ARRIVED,WS1,PICK3'}"`
- 수신 확인:
  - `ros2 topic echo /serial_rx`
