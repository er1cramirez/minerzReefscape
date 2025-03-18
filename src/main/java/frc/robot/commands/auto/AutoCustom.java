package frc.robot.commands.auto;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.CoralGrabber;
import frc.robot.subsystems.CoralGrabberArm;
import frc.robot.subsystems.SimpleElevator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class AutoCustom extends SequentialCommandGroup {
    
    public AutoCustom(SwerveDrivetrain swerve, CoralGrabberArm coralArm, CoralGrabber coralGrabber, SimpleElevator elevator) {
        // Define poses for trajectories
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d());
        Pose2d forwardPose = new Pose2d(1, 0, new Rotation2d());  // 1 meter forward
        Pose2d diagonalPose = new Pose2d(0.5, -1, new Rotation2d()); // Diagonal back and right
        Pose2d rightPose = new Pose2d(0.5, -2, new Rotation2d());  // More to the right
        
        // Create trajectories
        Trajectory forwardTrajectory = createTrajectory(swerve, startPose, forwardPose);
        Trajectory diagonalTrajectory = createTrajectory(swerve, forwardPose, diagonalPose);
        Trajectory rightTrajectory = createTrajectory(swerve, diagonalPose, rightPose);
        
        addCommands(
            // 1. Reset heading, lift coral arm and absorb
            new InstantCommand(() -> swerve.resetRobotHeading()),
            Commands.startEnd(
                () -> coralArm.setSpeed(0.15),
                () -> coralArm.stop(),
                coralArm
            ).withTimeout(0.5),
            
            Commands.startEnd(
                () -> coralGrabber.grab(),
                () -> coralGrabber.stop(),
                coralGrabber
            ).withTimeout(0.3),
            
            // 2. Move forward
            new SwerveTrajectoryCommand(
                "Move Forward",
                swerve,
                forwardTrajectory,
                new Rotation2d()
            ),
            
            // 3. Raise elevator to limit
            Commands.startEnd(
                () -> elevator.setSpeed(0.8),  // Using high power to raise
                () -> elevator.stop(),
                elevator
            ).withTimeout(2.0),
            
            // 4. Move diagonally (back and right)
            new SwerveTrajectoryCommand(
                "Move Diagonally",
                swerve,
                diagonalTrajectory,
                new Rotation2d()
            ),
            
            // 5. Wait 2 seconds
            Commands.waitSeconds(2),
            
            // 6. Release coral
            Commands.startEnd(
                () -> coralGrabber.release(),
                () -> coralGrabber.stop(),
                coralGrabber
            ).withTimeout(0.3),
            
            // 7. Move to the right
            new SwerveTrajectoryCommand(
                "Move Right",
                swerve,
                rightTrajectory,
                new Rotation2d()
            )
        );
    }
    
    private Trajectory createTrajectory(SwerveDrivetrain swerve, Pose2d start, Pose2d end) {
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.MAX_SPEED,
            AutoConstants.MAX_ACCELERATION
        ).setKinematics(swerve.getKinematics());
        
        return TrajectoryGenerator.generateTrajectory(
            start,
            List.of(),
            end,
            config
        );
    }
}
