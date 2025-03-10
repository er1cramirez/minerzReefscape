package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGrabberArm;

public class ArmPositionCommand extends Command {
    private final CoralGrabberArm arm;
    private final TrapezoidProfile profile;
    private final TrapezoidProfile.State targetState;
    private TrapezoidProfile.State currentState;
    private static final double dt = 0.02; // 20ms loop time

    // Constants for arm motion
    private static final double MAX_VELOCITY = 2.0; // radians per second
    private static final double MAX_ACCELERATION = 4.0; // radians per second squared
    
    // Position threshold for considering the movement complete
    private static final double POSITION_THRESHOLD = 0.05; // radians

    public ArmPositionCommand(CoralGrabberArm arm, Rotation2d targetPosition) {
        this.arm = arm;
        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION)
        );
        this.targetState = new TrapezoidProfile.State(targetPosition.getRadians(), 0);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        currentState = new TrapezoidProfile.State(
            arm.getArmPosition(), 
            0.0 // We don't have velocity tracking, so assume starting velocity of 0
        );
    }

    @Override
    public void execute() {
        currentState = profile.calculate(dt, currentState, targetState);
        arm.setArmPosition(new Rotation2d(currentState.position));
    }

    @Override
    public boolean isFinished() {
        boolean atTarget = Math.abs(arm.getArmPosition() - targetState.position) < POSITION_THRESHOLD;
        return atTarget;
    }

    @Override
    public void end(boolean interrupted) {
        // No need to stop as the motor will hold position via closed loop control
    }
}
