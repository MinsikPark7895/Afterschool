import { MapActivateResponse, MapCreateRequest, MapCreateResponse, MapDetailResponse, MapListResponse, MapSaveRequest } from "../types/map";


const BASE_URL = process.env.REACT_APP_API_URL || '/api';

export const mapService = {
    // S3 파일 존재 여부 테스트용 (임시)
    async testS3Files(): Promise<void> {
        console.log("🧪 S3 파일 존재 여부 테스트 시작...");
        
        // 1. 백엔드 S3 설정 정보 확인
        try {
            console.log("🔧 백엔드 S3 설정 확인 중...");
            const configResponse = await fetch(`${BASE_URL}/actuator/info`, {
                headers: {
                    'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
                }
            });
            
            if (configResponse.ok) {
                const configData = await configResponse.json();
                console.log("⚙️ 백엔드 설정 정보:", configData);
            } else {
                console.log("⚠️ 백엔드 설정 정보를 가져올 수 없음");
            }
        } catch (error) {
            console.log("⚠️ 백엔드 설정 확인 실패:", error);
        }
        
        // 2. S3 파일 테스트
        const testUrls = [
            `${BASE_URL}/maps/pgm`,
            `${BASE_URL}/maps/yaml`
        ];
        
        for (const url of testUrls) {
            try {
                console.log(`📡 테스트 중: ${url}`);
                
                // HEAD 요청으로 빠른 확인
                const headResponse = await fetch(url, {
                    method: 'HEAD',
                    headers: {
                        'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
                    }
                });
                
                console.log(`${headResponse.ok ? '✅' : '❌'} HEAD ${url}: ${headResponse.status} ${headResponse.statusText}`);
                
                // 실패시 GET 요청으로 상세 오류 확인
                if (!headResponse.ok) {
                    const errorResponse = await fetch(url, {
                        method: 'GET',
                        headers: {
                            'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
                        }
                    });
                    
                    const errorText = await errorResponse.text();
                    console.log(`📄 GET ${url} 오류 응답:`, errorText);
                    
                    // JSON 파싱 시도
                    try {
                        const errorJson = JSON.parse(errorText);
                        console.log(`🔍 상세 오류 분석:`, {
                            status: errorJson.status,
                            message: errorJson.message,
                            timestamp: errorJson.timestamp
                        });
                    } catch (e) {
                        console.log(`📝 오류 응답이 JSON이 아님:`, errorText.substring(0, 200));
                    }
                }
                
            } catch (error) {
                console.log(`❌ ${url}: 네트워크 연결 오류`, error);
            }
        }
        
        console.log("🔍 백엔드 개발자에게 확인 요청할 사항:");
        console.log("   1. 서버 로그에서 S3Service 오류 메시지 확인");
        console.log("   2. 환경변수 AWS_S3_BUCKET, AWS_REGION 값 확인");
        console.log("   3. AWS 자격증명 설정 확인 (profile 또는 IAM role)");
        console.log("   4. S3 버킷 권한 정책 확인");
    },

    // 맵 목록 조회 (페이지네이션 + 필터 추가)
    async getMaps(isActive: boolean = true, page: number = 0, size: number = 10): Promise<MapListResponse> {
        const response = await fetch(`${BASE_URL}/map?isActive=${isActive}&page=${page}&size=${size}`, {
            method: 'GET',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
            }
        });
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }

        return await response.json();
    },

    // 맵 생성 시작
    async createMap(mapData: MapCreateRequest): Promise<MapCreateResponse> {
        const response = await fetch(`${BASE_URL}/maps/create-map`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
            },
            body: JSON.stringify(mapData)
        });

        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }

        return await response.json();
    },

    // 맵 저장 완료
    async saveMap(mapData: MapSaveRequest): Promise<any> {
        const response = await fetch(`${BASE_URL}/maps/save-map`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
            },
            body: JSON.stringify(mapData)
        });

        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }

        return await response.json();
    },

    // 맵 상세 조회
    async getMapDetail(mapId: string): Promise<MapDetailResponse> {
        const response = await fetch(`${BASE_URL}/maps/${mapId}`, {
            method: 'GET',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
            }
        });

        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }

        return await response.json();
    },

    // 맵 활성화
    async activateMap(mapId: string): Promise<MapActivateResponse> {
        const response = await fetch(`${BASE_URL}/maps/${mapId}/activate`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
            }
        });

        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }

        return await response.json();
    },

    // PGM 맵 이미지 파일 다운로드 (Blob 방식)
    async downloadPgmFile(): Promise<Blob> {
        try {
            const response = await fetch(`${BASE_URL}/maps/pgm`, {
                method: 'GET',
                headers: {
                    'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
                },
                // 타임아웃 설정으로 무한 대기 방지
                signal: AbortSignal.timeout(30000) // 30초 타임아웃
            });

            if (!response.ok) {
                if (response.status === 404) {
                    throw new Error('맵 파일을 찾을 수 없습니다.');
                } else if (response.status === 503) {
                    throw new Error('S3 연결 오류가 발생했습니다.');
                }
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            return await response.blob();
        } catch (error) {
            if (error instanceof Error && error.name === 'TimeoutError') {
                throw new Error('맵 파일 다운로드 시간이 초과되었습니다.');
            }
            throw error;
        }
    },

    // PGM 맵 이미지 URL 가져오기 (백엔드 S3 키 방식에 맞춤)
    async getPgmImageUrl(): Promise<string> {
        try {
            console.log('백엔드 /maps/pgm API 호출 시작...');
            console.log('🔗 BASE_URL:', BASE_URL);
            console.log('🔗 전체 URL:', `${BASE_URL}/maps/pgm`);
            console.log('🔑 토큰:', localStorage.getItem('accessToken') ? '토큰 있음' : '토큰 없음');
            
            // 백엔드에서 S3 키 "maps/map.pgm"로 파일을 다운로드하여 반환
            const response = await fetch(`${BASE_URL}/maps/pgm`, {
                method: 'GET',
                headers: {
                    'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
                },
                signal: AbortSignal.timeout(30000)
            });

            if (response.ok) {
                console.log('PGM 파일 다운로드 성공, Blob URL 생성...');
                // 백엔드에서 ByteArrayResource로 반환하므로 Blob으로 받아서 URL 생성
                const blob = await response.blob();
                const imageUrl = URL.createObjectURL(blob);
                console.log('PGM Blob URL 생성 완료:', imageUrl);
                return imageUrl;
            } else {
                console.error('❌ PGM API 응답 오류:', response.status, response.statusText);
                console.error('❌ 요청 URL:', `${BASE_URL}/maps/pgm`);
                
                // 응답 본문도 확인해보기
                try {
                    const errorText = await response.text();
                    console.error('❌ PGM 응답 본문:', errorText);
                    
                    // JSON 파싱 시도
                    try {
                        const errorJson = JSON.parse(errorText);
                        console.error('🔍 PGM 백엔드 오류 상세:', {
                            status: errorJson.status,
                            message: errorJson.message,
                            data: errorJson.data
                        });
                    } catch (jsonError) {
                        console.error('📄 PGM 응답이 JSON이 아님:', errorText);
                    }
                } catch (e) {
                    console.error('❌ PGM 응답 본문 읽기 실패:', e);
                }
                
                throw new Error(`PGM API 오류: ${response.status} - ${response.statusText}`);
            }

        } catch (error) {
            console.error('PGM 파일 로드 실패:', error);
            // 에러 시 폴백 - 빈 이미지나 기본 이미지 사용
            throw new Error(`PGM 파일을 불러올 수 없습니다: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
    },

    // YAML 맵 설정 파일 다운로드 (백엔드 S3 키 방식에 맞춤)
    async downloadYamlFile(): Promise<string> {
        try {
            console.log('백엔드 /maps/yaml API 호출 시작...');
            console.log('🔗 BASE_URL:', BASE_URL);
            console.log('🔗 전체 URL:', `${BASE_URL}/maps/yaml`);
            console.log('🔑 토큰:', localStorage.getItem('accessToken') ? '토큰 있음' : '토큰 없음');
            
            // 백엔드에서 S3 키 "maps/map.yaml"로 파일을 다운로드하여 반환
            const response = await fetch(`${BASE_URL}/maps/yaml`, {
                method: 'GET',
                headers: {
                    'Authorization': `Bearer ${localStorage.getItem('accessToken')}`
                },
                signal: AbortSignal.timeout(30000)
            });

            if (!response.ok) {
                console.error('❌ YAML API 응답 오류:', response.status, response.statusText);
                console.error('❌ 요청 URL:', `${BASE_URL}/maps/yaml`);
                console.error('❌ 요청 헤더:', response.headers);
                
                // 응답 본문도 확인해보기
                try {
                    const errorText = await response.text();
                    console.error('❌ 응답 본문:', errorText);
                    
                    // JSON 파싱 시도
                    try {
                        const errorJson = JSON.parse(errorText);
                        console.error('🔍 백엔드 오류 상세:', {
                            status: errorJson.status,
                            message: errorJson.message,
                            data: errorJson.data
                        });
                    } catch (jsonError) {
                        console.error('📄 응답이 JSON이 아님:', errorText);
                    }
                } catch (e) {
                    console.error('❌ 응답 본문 읽기 실패:', e);
                }
                
                if (response.status === 404) {
                    throw new Error('맵 설정 파일을 찾을 수 없습니다.');
                } else if (response.status === 500) {
                    throw new Error('서버 내부 오류가 발생했습니다. S3 연결을 확인해주세요.');
                } else if (response.status === 503) {
                    throw new Error('S3 연결 오류가 발생했습니다.');
                }
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const yamlContent = await response.text();
            console.log('YAML 파일 다운로드 성공, 길이:', yamlContent.length);
            return yamlContent;
            
        } catch (error) {
            console.error('YAML 파일 다운로드 실패:', error);
            if (error instanceof Error && error.name === 'TimeoutError') {
                throw new Error('맵 설정 파일 다운로드 시간이 초과되었습니다.');
            }
            throw error;
        }
    }
};