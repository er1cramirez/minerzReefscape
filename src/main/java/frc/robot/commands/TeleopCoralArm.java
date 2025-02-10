package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllersConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralGrabberArm;
import frc.robot.util.InputProcessor;

public class TeleopCoralArm extends Command {
    private final CoralGrabberArm arm;
    private final DoubleSupplier speedSupplier;
    private final SlewRateLimiter speedLimiter;

    
    public TeleopCoralArm(CoralGrabberArm arm, DoubleSupplier speedSupplier) {
        this.arm = arm;
        this.speedSupplier = speedSupplier;
        this.speedLimiter = new SlewRateLimiter(
            CoralConstants.SLEW_RATE
        );
        addRequirements(arm);
    }
    
    @Override
    public void initialize() {
        // arm.disablePositionControl();
    }
    
    @Override
    public void execute() {
        double speed = InputProcessor.processInput(
            speedSupplier.getAsDouble(),
            ControllersConstants.mechanismControllerDeadband,
            speedLimiter,
            CoralConstants.MAX_SPEED
        );
        arm.setSpeed(speed);
    }
    
    @Override
    public void end(boolean interrupted) {
        // arm.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}