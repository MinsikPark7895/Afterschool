export interface Robot {
  id: number;
  robotId: string;
  robotName: string;
  robotType: string;
  assignedZone: string;
  isActive: boolean;
  currentStatus: CurrentStatus;
  createdAt: string;
  updatedAt: string;
}

export interface CurrentStatus {
  state: string;
  position: Position;
  batteryLevel: number;
  lastUpdated: string;
}

export interface Position {
  x: number;
  y: number;
  z: number;
  orientation?: {
    x: number;
    y: number;
    z: number;
    w: number;
  };
  currentZone?: string;
}

export interface Battery {
  level: number;
  chargingStatus: string;
}

export interface Hardware {
  lidar: string;
  camera: string;
  imu: string;
  wheels: string;
  motors: string;
}

export interface Performance {
  cpuUsage: number;
  memoryUsage: number;
  fps: number;
}

export interface Health {
  overallStatus: string;
  hardware: Hardware;
  performance: Performance;
}

export interface DetailedStatus {
  state: string;
  position: Position;
  battery: Battery;
  health: Health;
  lastUpdated: string;
}

export interface RobotDetail extends Omit<Robot, "currentStatus"> {
  detailedStatus: DetailedStatus;
}

export interface RobotListResponse {
  status: string;
  message: string;
  data: Robot[];
}

export interface RobotDetailResponse {
  status: string;
  message: string;
  data: RobotDetail;
}

export interface Waypoint {
  x: number;
  y: number;
  order: number;
}

export interface PatrolStartRequest {
  patrolZone: string;
  patrolMode: "auto" | "manual";
  waypoints: Waypoint[];
  patrolSpeed: number;
  loopCount: number; // -1은 무한 반복
  detectionEnabled: boolean;
  startImmediately: boolean;
  patrolDuration: number; // 초 단위
}

export interface PatrolStopRequest {
  stopMode: "immediate" | "grateful";
  returnPosition: {
    x: number;
    y: number;
  };
  reason: string;
}

export interface MoveToRequest {
  targetPosition: Position;
  speed: number;
  precision: "high" | "medium" | "low";
  avoidObstacles: boolean;
  maxPlanningTime: number;
}

export interface EmergencyStopRequest {
  stopAllRobots: boolean;
  affectedRobots?: string[];
  stopMode: "immediate";
  disableMotors: boolean;
  reason: string;
  description: string;
}

export interface CommandResponse {
  status: string;
  message: string;
  data: {
    command: string;
    robotId: string;
    commandType: string;
    status: string;
    sentAt: string;
    expectedResponseTime?: number;
  };
}
