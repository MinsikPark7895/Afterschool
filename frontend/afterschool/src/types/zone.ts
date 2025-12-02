export interface Zone {
    id: number;
    zoneId: string;
    zoneName: string;
    mapId: number;
    mapName: string;
    boundaryCoordinates: Array<{
        x: number;
        y: number;
    }>;
    waypoints: Array<{
        x: number;
        y: number;
    }>;
    assignedRobotId: string;
    assignedRobotName: string;
    priorityLevel: number;
    createdAt: string;
    updatedAt: string;
}

export interface ZoneListResponse {
    status: "SUCCESS" | "ERROR";
    message: string;
    data: Zone[];
}


