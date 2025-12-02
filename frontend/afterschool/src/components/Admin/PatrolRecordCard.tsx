

interface PatrolRecord {
    id: number;
    date: string;
    status: 'completed' | 'in_progress' | 'failed';
    duration: number;
    robotId: string;
    zones: string[];
}

interface PatrolRecordCardProps {
    record: PatrolRecord;
    onClick: () => void;
}

export default function PatrolRecordCard({ record, onClick}: PatrolRecordCardProps) {
    return (
        <div className="patrol-record-card" onClick={onClick}>
            <span className="patrol-date">{record.date}</span>
        </div>
    );
}



