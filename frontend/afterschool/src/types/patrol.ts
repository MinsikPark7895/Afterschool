export interface PatrolRecord {
    id: number;
    date: string;
    status: 'completed' | 'in_progress' | 'failed';
    duration: number;
    robotId: string;
    zones: string[];
    startTime?: string;
    endTime?: string;
    issues?: string[];
}

export interface PatrolDetail {
    id: number;
    date: string;
    status: 'completed' | 'in_progress' | 'failed';
    duration: number;
    robotId: string;
    zones: string[];
    route: {
        x: number;
        y: number;
        timestamp: string;
    }[];
    events: {
        type: 'start' | 'end' | 'issue' | 'zone_enter' | 'zone_exit';
        timestamp: string;
        description?: string;
    }[];
}


