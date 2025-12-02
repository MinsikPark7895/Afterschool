# 🏗️ 멀티 스테이지 빌드로 최적화된 프로덕션 이미지 생성
FROM eclipse-temurin:21-jdk AS builder

WORKDIR /app

# 📦 의존성 파일들을 먼저 복사 (캐시 최적화)
COPY build.gradle settings.gradle ./
COPY gradle gradle
COPY gradlew ./

# 🔧 gradlew 실행 권한 부여
RUN chmod +x ./gradlew

# 📥 의존성 다운로드 (의존성이 변경되지 않으면 캐시 재사용)
RUN ./gradlew dependencies --no-daemon

# 📝 소스 코드 복사 및 빌드
COPY src src
RUN ./gradlew build -x test --no-daemon

# 🚀 런타임 스테이지
FROM eclipse-temurin:21-jre-alpine

WORKDIR /app

# 📋 빌더 스테이지에서 빌드된 JAR 파일 복사
COPY --from=builder /app/build/libs/afterschool-0.0.1-SNAPSHOT.jar app.jar

EXPOSE 8080

# ⚡ Spring Boot 애플리케이션 실행
ENTRYPOINT ["java", "-jar", "app.jar"]