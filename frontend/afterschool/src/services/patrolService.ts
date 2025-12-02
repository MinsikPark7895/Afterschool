import { PatrolDetail, PatrolRecord } from "../types/patrol";

export const patrolService = {
    // 순찰 기록 목록 조회
    async getPatrolRecords(): Promise<PatrolRecord[]> {
        const token = localStorage.getItem('accessToken');
        // 이 API는 나중에 수정해야 함
        const response = await fetch('/api/admin/patrol-records', {
            method: 'GET',
            headers: {
                'Authorization': `Bearer ${token}`,
                'Content-Type': 'application/json'
            },
        });

        if(!response.ok) {
            throw new Error('순찰 기록을 불러오는데 실패했습니다.');
        }

        return response.json();
    },

    // 순찰 상세 정보 조회
    async getPatrolDetail(id: number): Promise<PatrolDetail> {
        const token = localStorage.getItem('accessToken');

        const response = await fetch(`/api/admin/patrol-records/${id}`, {
            method: 'GET',
            headers: {
                'Authorization': `Bearer ${token}`,
                'Content-Type': 'application/json',
            },
        });

        if(!response.ok) {
            throw new Error('순찰 상세 정보를 불러오는데 실패했습니다.');
        }

        return response.json();
    }
};