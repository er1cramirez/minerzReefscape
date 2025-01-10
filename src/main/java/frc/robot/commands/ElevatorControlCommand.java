package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.util.ElevatorStates;

public class ElevatorControlCommand extends Command{
    private final Elevator elevator;
    private final TrapezoidProfile profile;
    private final TrapezoidProfile.State targetState;
    private TrapezoidProfile.State currentState;
    private static final double dt = 0.02;

    // Position threshold for considering the movement complete
    private static final double POSITION_THRESHOLD = 0.05; // meters

    public ElevatorControlCommand(Elevator elevator, ElevatorStates targetPosition) {
        this.elevator = elevator;
        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.MAX_VELOCITY,
                Constants.ElevatorConstants.MAX_ACCELERATION
            )
        );
        this.targetState = new TrapezoidProfile.State(targetPosition.getHeight(), 0);
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        currentState = new TrapezoidProfile.State(
            elevator.getCurrentPosition(),
            elevator.getCurrentVelocity()
        );
    }

    @Override
    public void execute() {
        currentState = profile.calculate(dt, currentState, targetState);
        elevator.setPosition(currentState.position);
    }

    @Override
    public boolean isFinished() {
        boolean atTarget = Math.abs(elevator.getCurrentPosition() - targetState.position) < POSITION_THRESHOLD;
        boolean stoppedMoving = Math.abs(elevator.getCurrentVelocity()) < 0.01; // meters per second
    
        return atTarget && stoppedMoving;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator();
    }
}
