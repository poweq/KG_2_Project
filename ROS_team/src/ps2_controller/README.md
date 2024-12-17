# ROS_team

`ROS_team`는 ROS2를 기반으로 한 PS2 컨트롤러 인터페이스 패키지를 포함하는 디렉토리입니다. 

ROS2가 설치된 리눅스 PC에서 구동 가능합니다.

## 소개

`ROS_team`은 PS2 게임 컨트롤러를 사용하여 로봇을 직관적으로 제어할 수 있도록 하는 ROS2 패키지를 제공합니다. 

## 구성 요소

- **ps2_controller**: PS2 컨트롤러 인터페이스를 제공하는 ROS2 패키지.
  - **src**: 패키지의 소스 코드가 포함된 디렉토리.
    - `ps2_controller.cpp`: PS2 컨트롤러의 입력을 처리하는 노드 구현 파일.
    - `ps2_controller_node.cpp`: PS2 컨트롤러 노드의 주요 로직을 포함.(중요)
  - **include/ps2_controller**: 헤더 파일이 포함된 디렉토리.
    - `ps2_controller.h`: PS2 컨트롤러 노드의 헤더 파일.
    - `ps2_controller_node.hpp`: PS2 컨트롤러 노드의 클래스 정의 파일.
  - **CMakeLists.txt**: 패키지의 빌드 설정 파일.
  - **package.xml**: 패키지의 메타데이터 및 의존성을 정의하는 파일.
- **test.py**: 패키지의 기능을 테스트하기 위한 스크립트.

## 필수 조건

- **운영체제**: Ubuntu 20.04 이상
- **ROS2 버전**: Foxy Fitzroy 이상
- **Python 버전**: Python 3.8 이상
- **기타 의존성**:
  - `colcon` 빌드 도구
  - PS2 컨트롤러 및 USB 어댑터
  - 필요한 ROS2 패키지 및 라이브러리

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

클론한 레포지토리 내부의 `ROS_team` 디렉토리에서 `ros2_ws` 폴더를 우분투의 상위 폴더로 복사합니다.

```
cd KG_2_Project/ROS_team
cp -r src ~/ros2_ws/src
```

### 3. 패키지 빌드
생성한 워크스페이스 디렉토리로 이동하여 빌드합니다.

```
cd ~/ros2_ws
colcon build --packages-select ps2_controller
````

### 4. 환경 설정
빌드가 완료되면, 환경 설정 파일을 소스합니다.

```
source ~/ros2_ws/install/setup.bash
```
### 5.빌드 및 실행
ps2_controller 패키지의 컨트롤러 노드를 실행합니다.

```
ros2 run ps2_controller ps2_controller_node
```
이 명령어는 PS2 컨트롤러의 입력을 처리하여 ROS2 메시지로 변환하는 노드를 시작합니다.

### 6. 동작 설명
PS2 컨트롤러의 왼쪽 스틱은 전진 후진을 담당합니다.
오른쪽 스틱은 회전을 담당합니다.

왼쪽 스틱의 입력이 없고 오른쪽스틱만 조종하면 제자리에서 회전합니다.