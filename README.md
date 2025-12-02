# 🤖 AfterSchool - 교내 방범 로봇 시스템

> 자율 주행 로봇을 활용한 스마트 교내 보안 시스템

[![Spring Boot](https://img.shields.io/badge/Spring%20Boot-3.5.5-brightgreen.svg)](https://spring.io/projects/spring-boot)
[![React](https://img.shields.io/badge/React-19.1-blue.svg)](https://reactjs.org/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Java](https://img.shields.io/badge/Java-21-orange.svg)](https://www.oracle.com/java/)
[![TypeScript](https://img.shields.io/badge/TypeScript-4.9-blue.svg)](https://www.typescriptlang.org/)

## 📋 목차

- [프로젝트 소개](#-프로젝트-소개)
- [주요 기능](#-주요-기능)
- [기술 스택](#-기술-스택)
- [시스템 아키텍처](#-시스템-아키텍처)
- [프로젝트 구조](#-프로젝트-구조)
- [시작하기](#-시작하기)
- [환경 변수 설정](#-환경-변수-설정)
- [사용 방법](#-사용-방법)
- [API 문서](#-api-문서)
- [라이센스](#-라이센스)

## 🎯 프로젝트 소개

AfterSchool은 **TurtleBot3 기반 자율 주행 로봇**을 활용하여 교내 보안을 강화하는 스마트 방범 시스템입니다. 
로봇이 지정된 경로를 자율적으로 순찰하며, AI 기반 침입자 탐지 시스템을 통해 실시간으로 보안 이벤트를 감지하고 관리자에게 알림을 전송합니다.

### 📌 프로젝트 배경

- 야간 및 휴일 교내 보안 공백 최소화
- 효율적인 보안 인력 배치 및 운영
- AI 기술을 활용한 자동화된 침입자 탐지
- 실시간 모니터링 및 증거 자료 확보

## ✨ 주요 기능

### 🚀 핵심 기능

#### 1. **자율 순찰 (Autonomous Patrol)**
- 사전 정의된 경로를 따라 자율 주행
- 장애물 회피 및 동적 경로 재설정
- 배터리 관리 및 자동 충전 복귀

#### 2. **AI 기반 침입자 탐지 (Person Detection)**
- YOLOv8 딥러닝 모델을 활용한 실시간 사람 탐지
- 신뢰도 기반 침입자 식별
- 탐지 시 자동 증거 사진 캡처 및 S3 업로드

#### 3. **실시간 모니터링 대시보드**
- WebSocket 기반 실시간 로봇 위치 추적
- 로봇 상태 정보 (배터리, 속도, FSM 상태) 실시간 표시
- 이벤트 알림 및 탐지 이력 관리

#### 4. **원격 제어 시스템**
- MQTT 프로토콜을 통한 로봇 원격 제어
- 순찰 시작/중지, 특정 위치로 이동 명령
- 긴급 정지 및 시스템 상태 모니터링

#### 5. **증거 자료 관리**
- AWS S3를 활용한 탐지 이미지 저장
- 이벤트별 증거 파일 자동 분류
- 관리자 대시보드에서 증거 자료 조회 및 다운로드

## 🛠 기술 스택

### Frontend
- **React** 19.1.1 - UI 프레임워크
- **TypeScript** 4.9.5 - 타입 안정성
- **React Router** 6.30.1 - SPA 라우팅
- **STOMP.js** 7.2.0 - WebSocket 통신
- **SockJS** 1.6.1 - WebSocket 폴백

### Backend
- **Spring Boot** 3.5.5 - 애플리케이션 프레임워크
- **Java** 21 LTS - 프로그래밍 언어
- **Spring Security** + JWT - 인증/인가
- **Spring Data JPA** - ORM
- **Spring Integration MQTT** - MQTT 통신
- **Spring WebSocket** - 실시간 통신

### Database
- **MySQL** 8.4 - 주 데이터베이스
- **Redis** 7 - 세션 관리 및 캐싱

### Robot & AI
- **ROS2 Humble** - 로봇 운영체제
- **Gazebo** - 3D 로봇 시뮬레이터
- **YOLOv8** - 객체 탐지 딥러닝 모델
- **OpenCV** - 이미지 처리
- **Navigation2** - 자율 주행 네비게이션

### Cloud & DevOps
- **AWS S3** - 이미지 저장소
- **Docker** + **Docker Compose** - 컨테이너화
- **Jenkins** - CI/CD 파이프라인
- **Nginx** - 웹 서버 및 리버스 프록시

### Communication
- **MQTT** (Eclipse Mosquitto) - 로봇-서버 간 통신
- **WebSocket** (STOMP) - 서버-클라이언트 간 실시간 통신

## 🏗 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                         Client Layer                             │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  React Frontend (TypeScript)                              │  │
│  │  - Dashboard, Robot Control, Event Management             │  │
│  └────────────────┬─────────────────────────────────────────┘  │
└───────────────────┼─────────────────────────────────────────────┘
                    │ WebSocket (STOMP) / REST API
┌───────────────────┼─────────────────────────────────────────────┐
│                   │         Backend Layer                        │
│  ┌────────────────▼─────────────────────────────────────────┐  │
│  │  Spring Boot Application (Java 21)                        │  │
│  │  ├─ REST API Controllers                                  │  │
│  │  ├─ WebSocket Handlers                                    │  │
│  │  ├─ MQTT Message Handlers                                 │  │
│  │  ├─ JWT Authentication & Authorization                    │  │
│  │  └─ S3 File Management                                    │  │
│  └────────┬─────────────────────┬───────────────────────────┘  │
└───────────┼─────────────────────┼───────────────────────────────┘
            │                     │
    ┌───────▼─────────┐   ┌──────▼──────────┐
    │  MySQL Database │   │  Redis Cache    │
    │  - Users        │   │  - Sessions     │
    │  - Robots       │   │  - JWT Blacklist│
    │  - Events       │   └─────────────────┘
    │  - Logs         │
    └─────────────────┘
            │                     │ MQTT Protocol
┌───────────┼─────────────────────┼───────────────────────────────┐
│           │                     │    Robot Layer                 │
│  ┌────────▼─────────────────────▼───────────────────────────┐  │
│  │  MQTT Broker (Eclipse Mosquitto)                          │  │
│  │  - Topic: from_robot/* (Status, Detection)                │  │
│  │  - Topic: to_robot/*   (Commands)                         │  │
│  └────────────────────┬───────────────────────────────────────┘  │
│                       │                                           │
│  ┌────────────────────▼───────────────────────────────────────┐  │
│  │  ROS2 Nodes (Python)                                       │  │
│  │  ├─ MQTT-ROS Bridge                                        │  │
│  │  ├─ Person Detection (YOLOv8)                              │  │
│  │  ├─ FSM State Machine (Patrol/Track)                      │  │
│  │  └─ Navigation Controller                                  │  │
│  └────────────────────┬───────────────────────────────────────┘  │
│                       │                                           │
│  ┌────────────────────▼───────────────────────────────────────┐  │
│  │  TurtleBot3 Waffle (Gazebo Simulation)                    │  │
│  │  - TB1: Zone A Patrol                                      │  │
│  │  - TB2: Zone B Patrol                                      │  │
│  └────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────┘
                       │
            ┌──────────▼──────────┐
            │     AWS S3          │
            │  Evidence Storage   │
            └─────────────────────┘
```

## 📁 프로젝트 구조

```
AfterSchool/
├── backend/                    # Spring Boot Backend
│   ├── src/
│   │   ├── main/
│   │   │   ├── java/com/ssafy/afterschool/
│   │   │   │   ├── domain/          # 도메인 계층
│   │   │   │   │   ├── auth/        # 인증/인가
│   │   │   │   │   ├── user/        # 사용자 관리
│   │   │   │   │   ├── robot/       # 로봇 관리
│   │   │   │   │   ├── event/       # 이벤트 관리
│   │   │   │   │   └── map/         # 맵 관리
│   │   │   │   ├── global/          # 공통 설정
│   │   │   │   │   ├── config/      # Spring 설정
│   │   │   │   │   ├── security/    # 보안 설정
│   │   │   │   │   ├── mqtt/        # MQTT 설정
│   │   │   │   │   ├── websocket/   # WebSocket 설정
│   │   │   │   │   └── s3/          # AWS S3 설정
│   │   │   │   └── AfterSchoolApplication.java
│   │   │   └── resources/
│   │   │       └── application.yml  # 애플리케이션 설정
│   │   └── test/                    # 테스트 코드
│   ├── build.gradle                 # Gradle 빌드 설정
│   ├── Dockerfile                   # Docker 이미지 빌드
│   ├── compose.yaml                 # Docker Compose 설정
│   └── schema.sql                   # 데이터베이스 스키마
│
├── frontend/                   # React Frontend
│   └── afterschool/
│       ├── public/
│       ├── src/
│       │   ├── components/          # 재사용 컴포넌트
│       │   │   ├── Admin/          # 관리자 컴포넌트
│       │   │   ├── Robot/          # 로봇 관련 컴포넌트
│       │   │   ├── Event/          # 이벤트 컴포넌트
│       │   │   ├── Patrol/         # 순찰 컴포넌트
│       │   │   └── Map/            # 맵 컴포넌트
│       │   ├── pages/              # 페이지 컴포넌트
│       │   ├── services/           # API 서비스
│       │   ├── hooks/              # Custom Hooks
│       │   ├── types/              # TypeScript 타입 정의
│       │   └── App.tsx
│       ├── package.json
│       └── tsconfig.json
│
├── robot/                      # ROS2 Robot System
│   ├── ros2_ws/                    # ROS2 Workspace
│   │   └── src/
│   │       ├── ai_detection/       # AI 탐지 패키지
│   │       │   └── person_detection/
│   │       │       ├── person_detector.py    # YOLOv8 탐지
│   │       │       ├── detect_fsm_tb1.py     # TB1 FSM
│   │       │       └── detect_fsm_tb2.py     # TB2 FSM
│   │       ├── mqtt_ros_bridge/    # MQTT-ROS2 브리지
│   │       │   ├── mqtt_ros_bridge.py        # 상태 발행
│   │       │   └── mqtt_cmd_to_ros.py        # 명령 수신
│   │       └── coverage_planner/   # 경로 계획
│   ├── simulation/                 # Gazebo 시뮬레이션
│   │   └── worlds/
│   └── maps/                       # 맵 파일
│
├── nginx.conf                  # Nginx 설정
├── Jenkinsfile                 # CI/CD 파이프라인
├── compose.yaml                # 전체 시스템 Docker Compose
└── README.md

```

## 🚀 시작하기

### 사전 요구사항

#### Backend & Frontend
- **Java** 21 이상
- **Node.js** 18.x 이상
- **Docker** & **Docker Compose**
- **MySQL** 8.4
- **Redis** 7

#### Robot (선택사항)
- **ROS2 Humble** (Ubuntu 22.04 권장)
- **Gazebo** 시뮬레이터
- **Python** 3.10+

### 1. 저장소 클론

```bash
git clone https://github.com/your-username/AfterSchool.git
cd AfterSchool
```

#### Backend 환경 변수 파일 생성

`backend/.env` 파일을 생성하고 필요한 환경 변수를 설정하세요.

**필수 환경 변수:**
- 데이터베이스 연결 정보 (DB_HOST, DB_PORT, DB_NAME, DB_USERNAME, DB_PASSWORD)
- Redis 연결 정보 (REDIS_HOST, REDIS_PORT, REDIS_PASSWORD)
- JWT 시크릿 키 (JWT_SECRET - 최소 256bit)
- MQTT 브로커 정보 (MQTT_BROKER_URL, MQTT_USERNAME, MQTT_PASSWORD)
- AWS S3 자격증명 (AWS_REGION, AWS_S3_BUCKET, AWS_ACCESS_KEY_ID, AWS_SECRET_ACCESS_KEY)


#### Frontend 환경 변수 파일 생성

`frontend/afterschool/.env` 파일을 생성하세요:


### 3. Docker Compose로 실행 (권장)

```bash
# Backend + Database + Redis 실행
cd backend
docker compose up -d

# 로그 확인
docker compose logs -f
```

### 4. 수동 실행

#### Backend 실행

```bash
cd backend
./gradlew clean build
./gradlew bootRun
```

#### Frontend 실행

```bash
cd frontend/afterschool
npm install
npm start
```

#### Robot Simulation 실행

```bash
# ROS2 환경 설정
source /opt/ros/humble/setup.bash
cd robot/ros2_ws
colcon build
source install/setup.bash

# Gazebo 시뮬레이션 실행
ros2 launch person_detection spawn_robots_only.launch.py

# MQTT 브리지 실행
ros2 run mqtt_ros_bridge mqtt_ros_bridge

# 탐지 노드 실행
ros2 run person_detection detect_fsm_tb1
ros2 run person_detection detect_fsm_tb2
```

## 🔐 환경 변수 설정

### 환경 변수 확인 방법
모든 민감 정보는 `${ENV_VARIABLE}` 형식으로 환경 변수에서 읽어옵니다.

### 주요 환경 변수 카테고리

1. **데이터베이스 설정**: `DB_*` 접두사
2. **Redis 설정**: `REDIS_*` 접두사  
3. **JWT 설정**: `JWT_*` 접두사
4. **MQTT 설정**: `MQTT_*` 접두사
5. **AWS 설정**: `AWS_*` 접두사
6. **서버 설정**: `SERVER_*` 접두사

### 환경 변수 설정 가이드

1. `backend/.env` 파일 생성
2. `application.yml`에서 `${VARIABLE_NAME}` 형식으로 참조되는 변수 확인
3. 각 환경에 맞는 실제 값 설정

## 📖 사용 방법

### 1. 웹 대시보드 접속

브라우저에서 `http://localhost:3000` 접속

### 2. 로그인

기본 관리자 계정:
- **사용자명**: `admin`
- **비밀번호**: 초기 설정 시 생성된 비밀번호

### 3. 로봇 순찰 시작

1. **로봇 목록** 페이지에서 로봇 선택
2. **순찰 시작** 버튼 클릭
3. 실시간 로봇 위치 및 상태 모니터링

### 4. 이벤트 확인

1. **이벤트** 페이지에서 탐지 이력 확인
2. 이벤트 클릭 시 상세 정보 및 증거 사진 확인
3. 필터링 및 검색 기능 활용

### 5. 관리자 기능

- **사용자 관리**: 권한별 사용자 추가/수정/삭제
- **시스템 로그**: 명령 실행 이력 확인
- **대시보드**: 통계 및 시스템 상태 모니터링

## 📚 API 문서

백엔드 서버 실행 후 Swagger UI에서 상세한 API 문서를 확인할 수 있습니다:

#### 인증 API
- 로그인, 로그아웃, 토큰 갱신

#### 로봇 관리 API
- 로봇 목록 조회, 상태 조회, 순찰 제어, 이동 명령

#### 이벤트 관리 API
- 탐지 이벤트 조회, 상세 정보, 증거 파일 다운로드

#### 관리자 API
- 사용자 관리, 시스템 로그 조회

## 🧪 테스트

### Backend 테스트

```bash
cd backend
./gradlew test
```

### Frontend 테스트

```bash
cd frontend/afterschool
npm test
```

## 🔧 트러블슈팅

### Frontend 관련 이슈

#### 1. 지도(PGM 파일) 화면 출력 문제

**문제 상황:**
- ROS2 환경에서 생성된 PGM 파일이 일반적인 이미지 태그(`<img>`)로 화면에 출력되지 않음
- PGM 파일은 바이너리 형식이라 브라우저에서 직접 렌더링 불가

**원인:**
- PGM 파일은 ROS2 맵 생성 도구에서 생성된 바이너리 형식의 이미지 파일
- 브라우저의 기본 이미지 렌더러가 PGM 형식을 지원하지 않음

**해결 방법:**
1. PGM 파일을 AWS S3에 저장
2. 프론트엔드에서 S3 버킷과 키를 사용하여 파일 다운로드
3. Canvas API를 사용하여 PGM 파일을 파싱하고 화면에 렌더링

```typescript
// 예시: Canvas를 사용한 PGM 파일 렌더링
const loadPgmMap = async (s3Bucket: string, s3Key: string) => {
  const response = await fetch(`/api/maps/download?bucket=${s3Bucket}&key=${s3Key}`);
  const blob = await response.blob();
  const imageBitmap = await createImageBitmap(blob);
  
  const canvas = document.getElementById('map-canvas') as HTMLCanvasElement;
  const ctx = canvas.getContext('2d');
  ctx?.drawImage(imageBitmap, 0, 0);
};
```

#### 2. WebSocket 로봇 위치 반응형 레이아웃 문제

**문제 상황:**
- 지도 너비가 2100px인데 화면 너비가 1440px로 작음
- 로봇 위치가 화면 밖에 표시되어 보이지 않음
- 반응형 레이아웃이 로봇 위치 마커에 적용되지 않음

**원인:**
- 로봇 위치 좌표가 실제 지도 픽셀 좌표로 전달됨
- 화면 크기와 지도 크기의 비율이 고려되지 않음
- CSS만으로는 동적 좌표 조정 불가

**해결 방법:**
1. 지도 파일의 실제 너비와 화면에 표시되는 너비의 비율 계산
2. WebSocket으로 받은 로봇 위치 좌표에 비율을 적용하여 스케일링
3. 컴포넌트별로 다른 비율 적용 (지도 컴포넌트, 순찰 모드 등)

```typescript
// 예시: 반응형 로봇 위치 계산
const calculateResponsivePosition = (
  robotX: number, 
  robotY: number, 
  mapWidth: number, 
  containerWidth: number
) => {
  const scaleRatio = containerWidth / mapWidth;
  return {
    x: robotX * scaleRatio,
    y: robotY * scaleRatio
  };
};

// WebSocket 메시지 수신 시
websocketService.onRobotPositionUpdate((data) => {
  const scaledPosition = calculateResponsivePosition(
    data.position.pixel_x,
    data.position.pixel_y,
    MAP_CONFIG.width,
    mapContainerRef.current?.clientWidth || 1440
  );
  setRobotPosition(scaledPosition);
});
```

#### 3. WebSocket 연결 끊김 문제

**문제 상황:**
- 일정 시간 후 WebSocket 연결이 자동으로 끊어짐
- 로봇 위치 업데이트가 중단됨

**해결 방법:**
- 자동 재연결 로직 구현
- Heartbeat 메커니즘 추가
- 연결 상태 모니터링 및 사용자 알림

```typescript
// WebSocket 재연결 로직
private handleReconnect() {
  if (this.reconnectAttempts < this.maxReconnectAttempts) {
    setTimeout(() => {
      this.connect(this.callbacks);
    }, this.reconnectInterval);
  }
}
```

### Backend & Infrastructure 관련 이슈

#### 4. Docker Compose 실행 오류

```bash
# 기존 컨테이너 정리
docker compose down -v

# 이미지 재빌드
docker compose up --build -d
```

#### 5. 데이터베이스 연결 오류

- `.env` 파일의 데이터베이스 자격증명 확인
- MySQL 컨테이너 상태 확인: `docker compose ps`
- 로그 확인: `docker compose logs mysql`

#### 6. MQTT 연결 오류

- MQTT Broker가 실행 중인지 확인
- 방화벽 설정 확인 (1883 포트)
- 환경 변수 `MQTT_BROKER_URL` 확인

#### 7. S3 업로드 오류

- AWS 자격증명 확인
- S3 버킷 권한 확인 (PutObject, GetObject)
- IAM 정책 확인

## 🤝 기여하기

기여는 언제나 환영합니다!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 라이센스

이 프로젝트는 MIT 라이센스를 따릅니다. 자세한 내용은 `LICENSE` 파일을 참조하세요.

## 👥 팀원

- **Backend Developer** - Spring Boot, MQTT, WebSocket, Docker, Jenkins, AWS
- **Frontend Developer** - React, TypeScript
- **Robot Engineer** - ROS2, Gazebo, AI Detection

