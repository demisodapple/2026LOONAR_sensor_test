# MKRZERO 센서 실험

PC USB → MKRZERO → LIS3MDL / MLX90614 / MAX31865 + PT1000 → USB CSV(0.5초 간격).
상위 Teensy 프로젝트와 별개의 PlatformIO 프로젝트다. VS Code에서 이 폴더를 Open Folder로 열면 된다.

## 배선

USB와 배터리를 뺀 상태에서 배선한다. 사진의 부품면에서 USB를 위로 두면 D11/D12는 오른쪽 열, D0/D1은 왼쪽 열에 있다.
숫자는 커넥터 순번이 아니라 보드에 인쇄된 디지털 핀 번호다. VCC는 3.3V이며 VIN과 다르다.

| 센서 단자 | MKRZERO 핀 | 역할 |
| --- | --- | --- |
| LIS3MDL SDA | 11 / SDA | 자기장 전용 데이터선 (Wire / SERCOM2) |
| LIS3MDL SCL | 12 / SCL | 자기장 전용 클록선 |
| MLX90614 SDA | 0 | 적외선 전용 데이터선 (irWire / SERCOM3) |
| MLX90614 SCL | 1 | 적외선 전용 클록선 |
| MAX31865 SDI / MOSI | 8 / MOSI | 보드 → 센서 데이터 |
| MAX31865 SDO / MISO | 10 / MISO | 센서 → 보드 데이터 |
| MAX31865 CLK / SCK | 9 / SCK | SPI 클록 |
| MAX31865 CS | 7 | 통신할 센서 선택 |
| 각 센서 GND | GND | 공통 전압 기준 |
| 3.3V 동작 가능 모듈의 전원 입력 | VCC / 3.3V | 센서 전원 |

MKRZERO 신호는 3.3V다. 5V 신호/5V 풀업을 D11/D12 등에 연결하지 않는다.
MLX90614는 3V형/5V형이 있으므로 센서 또는 모듈의 정확한 모델을 확인한다.
5V 전용이면 그 모듈에 맞는 전원과 양방향 I2C 전압 변환기가 필요하다. 보드 사진만으로 센서 전원 규격은 확인할 수 없다.
모듈에 VIN과 3Vo가 함께 있다면 3Vo는 출력일 수 있으므로 전원 입력 핀을 모듈 자료에서 확인한다.

LIS3MDL은 I2C 모드로 연결한다. CS를 HIGH로 유지해야 하는 모듈은 CS를 3.3V에 연결한다.
SDO/SA1은 해당 모듈의 주소 선택용으로 GND 또는 3.3V에 고정한다. 코드는 0x1C와 0x1E를 모두 시도한다.
MLX90614 주소는 기본값 0x5A를 사용한다. 두 센서는 별도 하드웨어 I2C 버스를 사용한다.
D0/D1은 코드가 추가 I2C 핀으로 설정하므로 이 실험에서 다른 용도로 사용하지 않는다.

풀업은 데이터/클록 선을 통신하지 않을 때 HIGH로 유지하는 저항이다.
모듈에 이미 3.3V 풀업이 있으면 중복 추가하지 않는다. 없으면 다음처럼 각각 연결한다.

```text
VCC(3.3V) ─ 4.7 kΩ ─ SDA(D11)
VCC(3.3V) ─ 4.7 kΩ ─ SCL(D12)
VCC(3.3V) ─ 4.7 kΩ ─ SDA(D0)
VCC(3.3V) ─ 4.7 kΩ ─ SCL(D1)
GND ─ 모든 센서 GND (풀업과 별도의 공통 접지)
```

MAX31865의 RTD 단자에는 PT1000을 연결하며 MKRZERO 핀에 PT1000을 직접 연결하지 않는다.
3선식 PT1000은 같은 저항 끝에 연결된 두 선과 반대 끝의 한 선으로 구성된다.
전원 OFF에서 멀티미터 저항으로 두 선 사이가 거의 0Ω인 쌍을 찾는다.
그 쌍은 모듈의 3선식 결선도에 지정된 같은 쪽 단자에 연결하고, 점퍼도 3선식으로 설정한다.
단자와 점퍼 배치는 모듈마다 다르므로 모듈 사진/모델 없이 특정 위치를 단정하지 않는다.
코드 설정은 PT1000, 기준 저항 4630Ω, MAX31865_3WIRE이다. 실제 모듈 기준 저항이 다르면 src/main.cpp의 RTD_RREF를 바꾼다.
PT100이면 RTD_R0도 100Ω으로 바꿔야 한다. 기준 저항이 틀리면 통신이 성공해도 온도가 틀린다.

## 빌드와 업로드

저장소 루트 PowerShell에서 실행한다. COMx는 실제 장치 포트로 바꾼다.

```powershell
$pioExe = "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"
& $pioExe run -d mkrzero_sensor_test -e all_sensors
& $pioExe device list
& $pioExe run -d mkrzero_sensor_test -e all_sensors -t upload --upload-port COMx
& $pioExe device monitor -d mkrzero_sensor_test -e all_sensors -p COMx -b 115200
```

먼저 mag_only → ir_only → rtd_only → all_sensors 순서로 -e 뒤 환경 이름을 바꿔 업로드한다.
단독 시험할 때는 다른 센서의 전원과 신호선을 분리한다. 비활성 센서도 물리적으로 버스를 붙잡을 수 있다.
업로드 전 시리얼 모니터를 닫는다. 포트가 안 잡히면 RESET을 빠르게 두 번 눌러 부트로더로 진입한 뒤 장치 목록을 다시 확인한다.
이 폴더에서 직접 실행할 때는 -d mkrzero_sensor_test를 생략한다.

## 출력과 확인

| 출력/검사 | 의미 또는 정상 조건 |
| --- | --- |
| 센서 READY | 초기 통신 확인. MAX31865는 쓰기/읽기 검증 포함 |
| mag_valid / ir_valid / rtd_valid | 해당 측정 읽기 및 기본 검사의 성공=1, 실패/비활성=0 |
| mag_x/y/z_uT, mag_norm_uT | 자기장 세 축과 벡터 크기, 단위 마이크로테슬라, 보정 없는 원본 |
| ir_ambient_C / ir_object_C | 센서 자체 온도 / 비접촉 대상 온도(섭씨) |
| rtd_raw / rtd_ohm / rtd_C | 원시 변환값 / 계산 저항 / 접촉 온도 |
| rtd_fault=0x00 | MAX31865가 보고한 고장 비트 없음 |
| nan | 측정 실패 또는 해당 시험에서 비활성 |
| 검은 탐침=GND, 빨간 탐침=VCC | 약 3.3V |
| 검은 탐침=GND, 빨간 탐침=SDA/SCL | 통신하지 않을 때 각각 약 3.3V |

전압은 VCC와 각 센서 전원 단자에서 각각 확인한다. I2C not idle은 한 선 이상 LOW라는 뜻이며 배선/전원/풀업부터 확인한다.
SDA/SCL은 센서별로 분리하고 서로 연결하지 않는다. 각 버스에 풀업 한 쌍이 필요하다.
전원과 GND는 공유한다. 버스 분리는 전원 잡음이나 펌웨어 호출 정지까지 격리하지는 않는다.
코드는 누락 센서를 5초마다 재시도한다. 시리얼로 r을 보내면 초기화를 재시도하고 h를 보내면 CSV 헤더를 다시 출력한다.
SAMD 통신 호출 자체가 정지할 경우 자동 재시도가 실행되지 않을 수 있으므로 물리적 고장을 소프트웨어로 해결했다고 판단하지 않는다.
로그의 # 시작 줄은 진단 메시지이며 나머지는 CSV다. 초기 헤더를 놓쳤으면 h를 전송한다.
READY/valid는 통신 확인이며 정확도 검증은 별도다. 자기장 보정 계수는 기존 Teensy에서 가져오지 않는다.

공식 자료:
- https://docs.arduino.cc/resources/pinouts/ABX00012-full-pinout.pdf
- https://docs.platformio.org/en/latest/boards/atmelsam/mkrzero.html
- https://github.com/arduino/ArduinoCore-samd/blob/master/variants/mkrzero/variant.cpp
