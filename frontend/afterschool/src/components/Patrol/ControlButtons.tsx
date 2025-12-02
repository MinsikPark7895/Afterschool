import React from "react";

interface ControlButtonsProps {
    onStartPatrol?: () => void;
    onManualControl?: () => void;
    isPatrolActive?: boolean;
    isManualMode?: boolean;
}

const ControlButtons: React.FC<ControlButtonsProps> = ({
    onStartPatrol,
    onManualControl,
    isPatrolActive = false,
    isManualMode = false,
}) => {
    return (
        <div className="control-buttons">
            <button
                className={`control-btn ${isPatrolActive ? 'active' : ''}`}
                onClick={onStartPatrol}
            >
                {isPatrolActive ? '순찰 중지' : '순찰 시작'}
            </button>

            <button
                className={`control-button manual-button ${isManualMode ? 'active' : ''}`}
                onClick={onManualControl}
            >
                {isManualMode ? '수동 모드 해제' : '수동 조작'}
            </button>
        </div>
    );
};

export default ControlButtons;


