package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SimpleElevator;

public class TeleopSimpleElevator extends Command {
    private final SimpleElevator elevator;
    private final DoubleSupplier speedSupplier;
    private final SlewRateLimiter speedLimiter;

    private static final double SLEW_RATE = 1.0;
    private static final double DEADBAND = 0.1;
    private static final double MAX_SPEED = 0.7;

    public TeleopSimpleElevator(SimpleElevator elevator, DoubleSupplier speedSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
        this.speedLimiter = new SlewRateLimiter(SLEW_RATE);
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double input = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
        double speed = speedLimiter.calculate(input * MAX_SPEED);
        elevator.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}