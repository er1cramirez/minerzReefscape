package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllersConstants;
import frc.robot.subsystems.Climber;
import frc.robot.util.InputProcessor;

public class TeleopClimber extends Command {
    private final Climber climber;
    private final DoubleSupplier speedSupplier;
    private final SlewRateLimiter speedLimiter;
    

    
    public TeleopClimber(Climber climber, DoubleSupplier speedSupplier) {
        this.climber = climber;
        this.speedSupplier = speedSupplier;
        this.speedLimiter = new SlewRateLimiter(
            ClimberConstants.SLEW_RATE
        );
        addRequirements(climber);
    }
    
    @Override
    public void execute() {
        double speed = InputProcessor.processInput(
            speedSupplier.getAsDouble(),
            ControllersConstants.chassisControllerDeadband,
            speedLimiter,
            ClimberConstants.MAX_SPEED
        );
        climber.setSpeed(speed);
    }
    
    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
