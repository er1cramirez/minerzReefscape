package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class TeleopElevator extends Command {
    private final Elevator elevator;
    private final DoubleSupplier speedSupplier;
    private final SlewRateLimiter speedLimiter;

    public TeleopElevator(Elevator elevator, DoubleSupplier speedSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
        this.speedLimiter = new SlewRateLimiter(0.5);
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double speed = speedLimiter.calculate(
            MathUtil.applyDeadband(speedSupplier.getAsDouble(), 0.1)
        );
        elevator.setSpeed(speed); 
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator();
    }
}
