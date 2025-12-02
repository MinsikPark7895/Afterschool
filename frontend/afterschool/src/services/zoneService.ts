import { ZoneListResponse } from "../types/zone";


const BASE_URL = process.env.REACT_APP_API_URL || '/api';

export const zoneService = {
    // 구역 목록 조회
    async getZones(mapId?: string, assignedRobotId?: string): Promise<ZoneListResponse> {
        let url = `${BASE_URL}/zones`;
        const params = new URLSearchParams();

        if (mapId) params.append('mapId', mapId);
        if (assignedRobotId) params.append('assignedRobotId', assignedRobotId);

        if (params.toString()) {
            url += `?${params.toString()}`;
        }

        const response = await fetch(url, {
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
    }

};


