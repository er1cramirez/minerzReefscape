package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllersConstants;
import frc.robot.Constants.SimpleElevatorConstants;
import frc.robot.subsystems.SimpleElevator;
import frc.robot.util.InputProcessor;

public class TeleopSimpleElevator extends Command {
    private final SimpleElevator elevator;
    private final DoubleSupplier speedSupplier;
    private final SlewRateLimiter speedLimiter;

    public TeleopSimpleElevator(SimpleElevator elevator, DoubleSupplier speedSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
        this.speedLimiter = new SlewRateLimiter(
            SimpleElevatorConstants.SLEW_RATE);
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double speed = InputProcessor.processInput(
            speedSupplier.getAsDouble(),
            ControllersConstants.mechanismControllerDeadband,
            speedLimiter,
            SimpleElevatorConstants.MAX_SPEED
        );
        elevator.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}