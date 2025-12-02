import React, { useState, useEffect } from "react";
import Header from "../components/Header/Header";
import MapCard from "../components/Map/MapCard";

// 보안을 고려한 프론트엔드 전용 타입 정의
interface MapData {
  id: number;
  mapId: string;
  mapName: string;
  coveragePercentage: number;
  isActive: boolean;
  lastUpdated: string;
}

// 백엔드 응답 타입 (보안을 고려하여 최소한의 정보만 포함)
interface MapApiResponse {
  status: string;
  message: string;
  data: {
    maps: Array<{
      id: number;
      mapId: string;
      mapName: string;
      coveragePercentage: number;
      isActive: boolean;
      lastUpdated: string;
    }>;
    pagination: {
      currentPage: number;
      totalPages: number;
      totalItems: number;
    };
  };
}

export default function MapPage() {
  const [maps, setMaps] = useState<MapData[]>([]);

  useEffect(() => {
    // 백엔드 API 호출
    fetchMapData();
  }, []);

  const fetchMapData = async () => {
    try {
      const response = await fetch("/api/maps");
      const apiData: MapApiResponse = await response.json();

      // 백엔드 데이터를 MapData 형태로 변환
      const mappedData: MapData[] = apiData.data.maps.map((item) => ({
        id: item.id,
        mapId: item.mapId,
        mapName: item.mapName,
        coveragePercentage: item.coveragePercentage,
        isActive: item.isActive,
        lastUpdated: item.lastUpdated,
      }));
      setMaps(mappedData);
    } catch (error) {
      console.error("Failed to fetch maps: ", error);
    }
  };

  const handleMapClick = (mapId: number) => {
    // 지도 클릭 시 실행될 로직
    console.log("Map clicked:", mapId);
    // TODO: 상세 페이지로 이동하거나 모달 표시
  };

  return (
    <div>
      <Header />
      {/* 메인 컨텐츠 */}
      <main>
        <h1>맵 페이지</h1>
        <div className="map-container">
          <div className="map-grid">
            {maps.map((map) => (
              <MapCard
                key={map.id}
                id={map.id}
                mapId={map.mapId}
                mapName={map.mapName}
                coveragePercentage={map.coveragePercentage}
                isActive={map.isActive}
                onClick={() => handleMapClick(map.id)}
              />
            ))}
          </div>
        </div>
      </main>
    </div>
  );
}
