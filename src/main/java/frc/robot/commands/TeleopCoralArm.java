package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGrabberArm;

public class TeleopCoralArm extends Command {
    private final CoralGrabberArm arm;
    private final DoubleSupplier speedSupplier;
    private static final double DEADBAND = 0.1;
    
    public TeleopCoralArm(CoralGrabberArm arm, DoubleSupplier speedSupplier) {
        this.arm = arm;
        this.speedSupplier = speedSupplier;
        addRequirements(arm);
    }
    
    @Override
    public void initialize() {
        // arm.disablePositionControl();
    }
    
    @Override
    public void execute() {
        double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
        arm.setSpeed(speed);
    }
    
    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}