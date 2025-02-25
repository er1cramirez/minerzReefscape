package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ControllersConstants;
import frc.robot.subsystems.AlgaeGrabberArm;
// import frc.robot.subsystems.CoralGrabberArm;
import frc.robot.util.InputProcessor;

public class TeleopAlgaeArm extends Command {
    private final AlgaeGrabberArm arm;
    private final DoubleSupplier speedSupplier;
    private final SlewRateLimiter speedLimiter;
    
    public TeleopAlgaeArm(AlgaeGrabberArm arm, DoubleSupplier speedSupplier) {
        this.arm = arm;
        this.speedSupplier = speedSupplier;
        this.speedLimiter = new SlewRateLimiter(
            AlgaeConstants.SLEW_RATE
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
            AlgaeConstants.MAX_SPEED
        );
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