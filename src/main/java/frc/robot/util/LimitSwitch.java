package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitch {
    enum Direction {
        FORWARD, REVERSE
    }

    private Direction direction;

    private DigitalInput limitSwitch;

    public LimitSwitch(int port, Direction direction) {
        this.direction = direction;
        limitSwitch = new DigitalInput(port);
    }

    public boolean isTriggered() {
        return limitSwitch.get();
    }

    public boolean isTriggered(Direction direction) {
        return isTriggered() && this.direction == direction;
    }

    
        
    
}
