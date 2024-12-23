# AGV_team

`AGV_team`는 ROS2를 기반으로 한 AGV(Automated Guided Vehicle) 제어 패키지를 포함하는 디렉토리입니다. 

AGV를 제어하는 라즈베리파이4에서 실행합니다.

## 소개

`AGV_team`은 ROS2를 사용하여 AGV의 이동 및 제어를 담당하는 패키지입니다. 이 패키지는 AGV의 이동 방향과 속도 제어, 센서 데이터 처리, Arduino와의 통신 등을 포함한 다양한 기능을 제공합니다. 또한, Arduino를 사용한 모터 제어 코드를 포함하여 하드웨어와의 연동을 지원합니다.

## 구성 요소

- **agv_control_py**: 주요 ROS2 패키지로 AGV의 제어 로직을 담당합니다.
- **arduino**: Arduino를 사용한 모터 제어 코드와 펌웨어를 포함합니다.
  - **encoder_motor_controll**: 인코더와 모터 제어를 위한 코드.
  - **final**: 최종 펌웨어 코드.

## 필수 조건

- **운영체제**: Ubuntu 20.04 이상
- **ROS2 버전**: Foxy Fitzroy 이상
- **Python 버전**: Python 3.8 이상
- **기타 의존성**:
  - `colcon` 빌드 도구
  - Arduino IDE (펌웨어 업로드를 위해 필요)
  - 기타 ROS2 패키지 및 라이브러리

## 설치

### 0. 레포지토리 클론

먼저, GitHub 레포지토리를 클론합니다.

```
git clone https://github.com/poweq/KG_2_Project.git
```
### 1. ROS2 워크스페이스 생성(선택)

먼저, ROS2 워크스페이스를 설정합니다. 최상위 디렉토리에 `ros2_ws` 폴더를 생성하고, 그 안에 `src` 폴더를 만듭니다.

```
mkdir -p ~/ros2_ws/src
```

### 2. 디렉토리 구조 설정

클론한 레포지토리 내부의 `AGV_team` 디렉토리에서 `ros2_ws` 폴더를 우분투의 상위 폴더로 복사합니다.

```
cd KG_2_Project/AGV_team
cp -r AGV_team/src ~/ros2_ws/src
```
### 3. 워크스페이스 빌드
ROS2 워크스페이스를 빌드합니다.

```
cd ~/ros2_ws
colcon build
```
### 4. 환경 설정
빌드가 완료되면, 환경 설정 파일을 소스합니다.

```
source ~/ros2_ws/install/setup.bash
```

### 5. AGV 노드 실행
agv_control_py 패키지의 제어 노드를 실행합니다.
(agv_contorl노드는 오타가 맞지만 실행에는 문제 없습니다)
```
ros2 run agv_control_py agv_contorl
```

### 6. 동작 예시 
실행시 ROS_team의 ps2 컨트롤러의 신호를 대기합니다

### 7. 추가 설정
라즈베리파이와 아두이는 현재 ttyAMA1에 연결되어 있습니다.
(tx : gpio 0, rx : gpio 1)
이 연결도를 바꾸고 싶다면
```
AGV_team\ros2_ws\src\agv_control_py\agv_control_py\agv_controller_node.py


self.serial_port = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
```

코드를 수정하면됩니다.
