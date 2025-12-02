.PHONY: up down build restart logs clean test swagger

# 기본 실행 (빌드 + 백그라운드 실행)
up:
	docker compose up --build -d

# 컨테이너 중지 및 제거
down:
	docker compose down

# 강제 재빌드 후 실행
build:
	docker compose up --build -d --force-recreate

# 재시작
restart: down up

# 백엔드 로그 확인
logs:
	docker compose logs -f backend

# 모든 로그 확인
logs-all:
	docker compose logs -f

# 실행 상태 확인
status:
	docker compose ps

# 정리 (컨테이너, 볼륨, 이미지 모두 제거)
clean:
	docker compose down -v --rmi all

# 테스트 실행 (로컬)
test:
	./gradlew test

# Swagger UI 열기 (브라우저)
swagger:
	@echo "Opening Swagger UI..."
	@echo "URL: http://localhost:8080/api/swagger-ui/index.html"
	@powershell -Command "Start-Process 'http://localhost:8080/api/swagger-ui/index.html'"

# 도움말
help:
	@echo "Available commands:"
	@echo "  make up      - Start all services (build + detached)"
	@echo "  make down    - Stop and remove all containers"
	@echo "  make build   - Force rebuild and start"
	@echo "  make restart - Stop and start again"
	@echo "  make logs    - Show backend logs"
	@echo "  make status  - Show container status"
	@echo "  make clean   - Remove all containers, volumes, images"
	@echo "  make test    - Run tests locally"
	@echo "  make swagger - Open Swagger UI in browser"
	@echo "  make help    - Show this help"