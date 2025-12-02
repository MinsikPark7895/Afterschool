pipeline {
    agent any

    tools {
        nodejs 'NodeJS'
    }

    environment {
        // 📋 이미지 및 컨테이너 설정
        BACKEND_IMAGE = 'afterschool-backend'
        BACKEND_CONTAINER = 'afterschool-backend'

        // 🌐 서버 정보 (환경 변수에서 가져오거나 기본값 사용)
        SERVER_HOST = "${env.SERVER_HOST ?: 'localhost'}"
        SERVER_USER = "${env.SERVER_USER ?: 'ubuntu'}"

        // 🔢 빌드 버전 설정
        BUILD_VERSION = "${BUILD_NUMBER}"
        IMAGE_TAG = "${BUILD_NUMBER}"
    }

    stages {
        stage('🚀 시작') {
            steps {
                echo '=================================================='
                echo '         AfterSchool CI/CD 파이프라인 시작         '
                echo '=================================================='

                checkout scm

                echo "🌐 배포 대상 서버: ${env.SERVER_HOST}"
            }
        }

        stage('🏗️ 빌드') {
            parallel {
                stage('🔧 백엔드 빌드') {
                    steps {
                        echo ''
                        echo '======= 백엔드 빌드 시작 ======='

                        dir('backend') {
                            echo '🔨 Gradle 빌드 실행...'
                            sh 'chmod +x ./gradlew'
                            sh './gradlew clean build -x test --console=plain --quiet'
                            echo '✅ Spring Boot 빌드 완료'
                        }

                        echo '======= 백엔드 빌드 완료 ======='
                        echo ''
                    }
                }

                stage('🎨 프론트엔드 빌드') {
                    steps {
                        echo ''
                        echo '======= 프론트엔드 빌드 시작 ======='

                        dir('frontend/afterschool') {
                            echo '📦 npm 의존성 설치...'
                            sh 'npm ci --silent'
                            echo '✅ 의존성 설치 완료'

                            echo '⚛️ React 앱 빌드...'
                            sh 'npm run build 2>/dev/null || npm run build'
                            echo '✅ React 빌드 완료'
                        }

                        echo '======= 프론트엔드 빌드 완료 ======='
                        echo ''
                    }
                }
            }
        }

        stage('🧪 테스트') {
            steps {
                echo ''
                echo '======= 테스트 실행 ======='

                dir('backend') {
                    echo '🧪 백엔드 단위 테스트 실행...'
                    sh './gradlew test --console=plain --quiet'
                    echo '✅ 테스트 완료'
                }

                echo '======= 테스트 완료 ======='
                echo ''
            }
            post {
                always {
                    dir('backend') {
                        junit testResults: 'build/test-results/test/*.xml', allowEmptyResults: true
                    }
                }
            }
        }

        stage('🚀 배포') {
            steps {
                echo ''
                echo '=================================================='
                echo '                    배포 시작                     '
                echo '=================================================='

                // 🔐 Secret File을 사용하여 환경 변수 관리
                withCredentials([file(credentialsId: 'afterschool-env-file', variable: 'ENV_FILE')]) {
                    sshagent(['server-pem-key']) {
                        // 백엔드 배포
                        echo '🔧 백엔드 애플리케이션 배포 중...'
                        sh """
                            echo '📦 프로젝트 아카이브 생성...'
                            git archive --format=tar.gz HEAD > project-${BUILD_NUMBER}.tar.gz

                            echo '🚀 프로젝트 아카이브 및 환경 설정 파일 서버 전송...'
                            scp -o StrictHostKeyChecking=no -q project-${BUILD_NUMBER}.tar.gz ${env.SERVER_USER}@${env.SERVER_HOST}:/tmp/
                            scp -o StrictHostKeyChecking=no -q \$ENV_FILE ${env.SERVER_USER}@${env.SERVER_HOST}:/tmp/.env

                            echo '🔄 서버에서 프로젝트 배포...'
                            ssh -o StrictHostKeyChecking=no ${env.SERVER_USER}@${env.SERVER_HOST} "
                                cd /opt

                                # 프로젝트 디렉토리명 (환경 변수 또는 기본값)
                                PROJECT_DIR=\${PROJECT_DIR:-S13P21A504}

                                # 기존 Docker 컨테이너 중지
                                if [ -d \$PROJECT_DIR ]; then
                                    echo '🛑 기존 컨테이너 중지 중...'
                                    cd \$PROJECT_DIR/backend
                                    docker compose down --remove-orphans --timeout 30 2>/dev/null || true
                                    cd /opt
                                    
                                    # 기존 프로젝트 백업
                                    sudo rm -rf \${PROJECT_DIR}_backup
                                    sudo mv \$PROJECT_DIR \${PROJECT_DIR}_backup
                                fi

                                # 새 프로젝트 배포
                                echo '📂 새 프로젝트 배포 중...'
                                sudo mkdir -p \$PROJECT_DIR
                                cd \$PROJECT_DIR
                                sudo tar -xzf /tmp/project-${BUILD_NUMBER}.tar.gz
                                sudo chown -R ${env.SERVER_USER}:${env.SERVER_USER} .
                                sudo chmod +x backend/gradlew

                                # 🔐 Jenkins Credentials에서 가져온 환경 설정 파일 배치
                                echo '🔐 환경 설정 파일 배치 중...'
                                cp /tmp/.env .env
                                cp /tmp/.env backend/.env
                                echo '✅ 환경 설정 파일 배치 완료'

                                # Docker Compose로 전체 스택 재시작
                                echo '🐳 Docker 컨테이너 시작 중...'
                                cd backend
                                docker compose up -d --build --force-recreate

                                # 컨테이너 시작 확인
                                echo '⏳ 컨테이너 시작 대기 중...'
                                sleep 15
                                
                                echo '📊 컨테이너 상태 확인:'
                                docker compose ps
                                
                                echo '📋 애플리케이션 로그 확인 (최근 10줄):'
                                docker compose logs --tail=10 || true

                                # 임시 파일 정리
                                rm -f /tmp/project-${BUILD_NUMBER}.tar.gz /tmp/.env
                                echo '✅ 백엔드 및 DB 스택 배포 완료'
                            "
                        """

                        // 프론트엔드 배포
                        echo '🎨 프론트엔드 웹사이트 배포 중...'
                        sh """
                            cd frontend/afterschool
                            tar -czf build.tar.gz build/
                            scp -o StrictHostKeyChecking=no -q build.tar.gz ${env.SERVER_USER}@${env.SERVER_HOST}:/tmp/
                            scp -o StrictHostKeyChecking=no -q ../../nginx.conf ${env.SERVER_USER}@${env.SERVER_HOST}:/tmp/

                            ssh -o StrictHostKeyChecking=no ${env.SERVER_USER}@${env.SERVER_HOST} "
                                cd /tmp
                                sudo rm -rf /var/www/html/*
                                sudo tar -xzf build.tar.gz
                                sudo cp -r build/* /var/www/html/
                                sudo chown -R www-data:www-data /var/www/html/
                                sudo cp nginx.conf /etc/nginx/sites-available/default
                                sudo nginx -t && sudo systemctl reload nginx 2>/dev/null || sudo service nginx reload 2>/dev/null || true
                                rm -f build.tar.gz nginx.conf
                                echo '✅ 프론트엔드 및 nginx 설정 배포 완료'
                            "
                        """
                    }
                }

                echo ''
                echo '=================================================='
                echo '🎉 배포 성공!'
                echo "🌐 웹사이트: http://${env.SERVER_HOST}"
                echo '🔐 환경 설정이 Jenkins Secret File로 안전하게 관리됩니다!'
                echo '=================================================='
            }
        }
    }

    post {
        success {
            echo ''
            echo '🎉 CI/CD 파이프라인 성공!'
            updateGitlabCommitStatus name: 'build', state: 'success'
        }
        failure {
            echo ''
            echo '❌ CI/CD 파이프라인 실패!'
            echo '🔍 위 로그를 확인하여 문제를 해결하세요.'
            updateGitlabCommitStatus name: 'build', state: 'failed'
        }
        always {
            cleanWs()
        }
    }
}
