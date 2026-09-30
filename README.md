# LOONAR Teensy 4.1 센서 시험 · 자기장 보정

이 폴더는 **Teensy 4.1 + LIS3MDL + MLX90614 + MAX31865/PT1000** 시험용이다. 세 센서의 원본 값을 USB CSV로 내보낸다. LIS3MDL 배선은 현재 코드 기준 `Wire2` SDA 25/SCL 24, MLX90614는 `Wire1` SDA 17/SCL 16, MAX31865는 SPI CS 10이다. 실제 보드·전압·핀 배치를 전원 인가 전에 확인한다.

기본 `include/mag_calibration_params.h`는 `UNVERIFIED`, `kEnabled=false`다. **원본 XYZ와 원본 크기는 항상 기록**하고, 보정 계수가 없으면 보정 열은 `nan`, `mag_cal_valid=0`으로 기록한다. `mag_cal_science_ready=0`은 빵판용 시험 계수 또는 계수 없음, `1`은 설치 상태에서 독립 검증과 외부 크기 기준까지 통과한 계수라는 뜻이다. `mag_valid=1`은 센서 읽기 성공이지 주변 자기 간섭이 없다는 증명은 아니다.

## 1. 펌웨어 빌드와 USB 연결

PowerShell에서 이 폴더를 연 뒤:

이 PC에서는 `pio`/`python`이 PATH에 잡혀 있지 않아, 아래처럼 설치된 실행 파일을 직접 호출한다.

```powershell
& 'C:\Users\subin\.platformio\penv\Scripts\platformio.exe' run
& 'C:\Users\subin\.platformio\penv\Scripts\platformio.exe' run -t upload
& 'C:\Users\subin\.platformio\penv\Scripts\platformio.exe' device list
```

업로드는 기존 Teensy 펌웨어를 교체한다. 포트가 안 보이거나 `# LIS3MDL BUS STATE ... BOTH_STUCK_LOW`가 나오면 보정보다 **배선·전원·I²C 상태**를 먼저 해결한다. CSV에 `mag_valid=0`/`nan`만 나오면 보정 피팅을 진행하지 않는다.

## 2. 노트북에서 측정 파일 만들기

이 PC에서는 PlatformIO의 Python에 `pyserial`(USB 로깅)이, Codex Python에 `numpy`(피팅)가 이미 있다. 아래 두 경로를 각각 사용한다. 다른 PC에서는 일반 Python에 `requirements.txt`를 설치하면 된다.

```powershell
& 'C:\Users\subin\.platformio\penv\Scripts\python.exe' tools\capture_serial.py --port COM4 --session bench_cal_01 --phase bench --state motors_off --note "breadboard, same location, laptop 2 m" --seconds 180
```

`COM4`는 `pio device list`에서 찾은 실제 포트로 바꾼다. 세션마다 **새 이름**을 사용한다. 결과는 `logs/sessions/bench_cal_01.csv`(측정값), `.log`(진단 메시지), `.json`(환경·상태 기록)으로 나뉜다. 이전 세션에 덧붙이지 않는다. `--seconds`를 빼면 Ctrl+C로 종료할 수 있다.

노트북에서 처음부터 정확한 거리를 정할 수는 없다. 센서를 고정한 채 노트북을 약 1 m, 2 m로 옮겨 측정해 기준값 변화가 줄어드는지 확인한다. USB 선을 늘려 노트북을 멀리 두고 I²C 점퍼선은 짧게 유지한다.

| 세션 | 어떻게 측정하나 | 목적 |
| --- | --- | --- |
| `bench_fixed_01` | 센서와 주변 물체를 고정, 30–60초 | 기본 흔들림·간섭 확인 |
| `bench_cal_01` | 같은 위치에서 센서와 **센서 주변에 고정된 부품을 함께** 롤·피치·요우 등 다양한 3D 자세로 천천히 회전, 2–3분 | 보정 계수 계산 |
| `bench_val_01` | 별도 3D 회전을 다시 수행 | 독립 검증 |
| `bench_laptop_1m/2m` | 센서는 고정, 노트북 거리만 변경 | 노트북 영향 확인 |

현재 CSV는 약 **2 Hz**다. 피팅기는 자기장 측정의 **80% 이상 유효**, 유효한 점 **120개 이상**, 여러 3D 방향의 분포를 요구한다. 요우만 반복하거나 한쪽 방향에 점이 몰리면 거부한다. 회전 중 센서의 공간 위치가 크게 이동하거나 주변 물체가 바뀌면 그 세션은 보정용으로 쓰지 않는다. `--note`에 설치 상태·장소·높이를 구체적으로 적는다.

## 3. PC에서 보정법 시험

```powershell
& 'C:\Users\subin\.platformio\penv\Scripts\python.exe' tools\capture_serial.py --port COM4 --session bench_val_01 --phase validation --state motors_off --note "separate 3D rotation" --seconds 180
& 'C:\Users\subin\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' tools\fit_magnetometer.py --calibration logs\sessions\bench_cal_01.csv --validation logs\sessions\bench_val_01.csv --output-dir logs\sessions\bench_fit
```

피팅기는 이상치 후보를 제외하는 반복 타원체 피팅을 수행한다. 출력의 `calibration.json`에는 오프셋, 3×3 보정 행렬, 유효 데이터 비율, 3D 방향 분포, 독립 세션 오차가 기록된다. `train_preview.csv`·`validation_preview.csv`에서 **보정 전후 크기의 흔들림**을 비교한다. 실제 자기장 크기를 별도로 모르면 결과 상태는 `relative_only_not_for_science`다. 타원체 모양만으로 절대 µT 크기를 알아낼 수 없기 때문이다.

빵판 보정 결과를 **Teensy에서도 시험**하려면 독립 검증이 통과한 뒤 아래 명령으로 시험용 헤더를 만든다. 펌웨어를 재빌드·업로드하면 보정 열에 값이 나오지만 `# MAG_CAL id=BENCH_ONLY-...`, `mag_cal_science_ready=0`이므로 탐사/PCA의 확정 계수로 사용하지 않는다.

```powershell
& 'C:\Users\subin\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' tools\fit_magnetometer.py --calibration logs\sessions\bench_cal_01.csv --validation logs\sessions\bench_val_01.csv --output-dir logs\sessions\bench_fit --bench-header include\mag_calibration_params.h
& 'C:\Users\subin\.platformio\penv\Scripts\platformio.exe' run
& 'C:\Users\subin\.platformio\penv\Scripts\platformio.exe' run -t upload
```

## 4. 로버에 설치한 뒤

빵판에서 얻은 **계수는 로버 계수로 재사용하지 않는다.** 센서 위치·브래킷·배선이 확정된 상태에서 구동부를 끈 별도 3D 회전 세션 두 개를 `--phase installed --state motors_off`로 수집한다. 모터 정지/작동, 리액션 휠, 전원 상태 변화는 별도 **간섭 시험** 세션으로 기록한다. 움직이는 모터의 자기장은 고정 보정 행렬만으로 해결되지 않는다.

실제로 설치한 센서를 충분한 3D 방향으로 움직일 수 없다면, 현재 피팅기로 완전한 3축 탑재 보정 계수를 만들 수 없다. 그 상태에서 요우 데이터만 억지로 피팅하지 말고 센서 분리 거리와 배선을 개선하고, 고정 자세·구동 상태별 차이를 측정해 한계를 명시한다.

독립적으로 확인한 **그 장소의 자기장 크기(µT)**, 설치 상태의 *보정 세션과 검증 세션*, 충분한 3D 분포가 모두 있을 때만 `--reference-ut 값 --header include\mag_calibration_params.h`를 추가해 **과학 측정용 Teensy 계수**를 내보낸다. 외부 기준이 없는 빵판 헤더는 오직 `BENCH_ONLY` 시험용이다. 계수를 바꾼 뒤에는 빌드와 업로드를 다시 한다. 과학 측정 중 자동으로 타원체를 다시 학습시키면 실제 공간 자기 이상을 보정으로 지울 수 있으므로 사용하지 않는다.

원본·보정값을 모두 저장하고, PCA를 학습할 때 사용한 보정 버전과 Teensy의 `# MAG_CAL id=...`가 같아야 한다. 이 시험 펌웨어는 물리 로버에서 아직 검증되지 않았다.
