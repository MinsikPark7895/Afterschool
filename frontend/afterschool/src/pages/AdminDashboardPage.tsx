import { useEffect, useState } from "react";
import Header from "../components/Header/Header";
import PatrolRecordCard from "../components/Admin/PatrolRecordCard";
import "./AdminDashboardPage.css";



interface PatrolRecord {
    id: number;
    date: string;
    status: 'completed' | 'in_progress' | 'failed';
    duration: number;
    robotId: string;
    zones: string[];
}

export default function AdminDashboardPage() {
    const [patrolRecords, setPatrolRecords] = useState<PatrolRecord[]>([]);
    // const [loading, setLoading] = useState(false); // 현재 미사용

    // 더미 데이터(백엔드 연동 전까지 사용)
    useEffect(() => {
        const dummyData: PatrolRecord[] = [
            {
                id: 1,
                date: '2025-01-01',
                status: 'completed',
                duration: 45,
                robotId: 'ROBOT-001',
                zones: ['zone1', 'zone2'],
            },
            {
                id: 2,
                date: '2025-01-02',
                status: 'in_progress',
                duration: 30,
                robotId: 'ROBOT-001',
                zones: ['zone1', 'zone2'],
            },
            {
                id: 3,
                date: '2025-01-03',
                status: 'failed',
                duration: 15,
                robotId: 'ROBOT-002',
                zones: ['zone1', 'zone2'],
            }
        ];
        setPatrolRecords(dummyData);
    }, []);

    return (
        <div className="admin-dashboard-page">
            {/* 상단 헤더 */}
            <div className="admin-header">
                <h1>순찰 기록 관리</h1>
            </div>

            {/* 네비게이션 바 */}
            <Header/>

            {/* 메인 콘텐츠 */}
            <div className="admin-dashboard-content">
                <div className="patrol-records-card">
                    <h2>순찰 기록</h2>
                    <div className="patrol-divider"></div>
                    <div className="patrol-records-list">
                        {patrolRecords.map((record) => (
                            <PatrolRecordCard
                                key={record.id}
                                record={record}
                                onClick={() => {
                                    // 순찰 상세 정보 모달 열기
                                    console.log('순찰 기록 클릭:', record);
                                }}
                            />
                        ))}
                    </div>
                </div>
            </div>
        </div>
    )
}

