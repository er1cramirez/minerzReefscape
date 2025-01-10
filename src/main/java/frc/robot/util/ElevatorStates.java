package frc.robot.util;

public enum ElevatorStates {
    GROUND,
    L1,
    L2,
    L3,
    TOP;

    public double getHeight() {
        switch (this) {
            case GROUND:
                return 0.0;
            case L1:
                return 1.0;
            case L2:
                return 2.0;
            case L3:
                return 3.0;
            case TOP:
                return 4.0;
            default:
                return 0.0;
        }
    }
}
