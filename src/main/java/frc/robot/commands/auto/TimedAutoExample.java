package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SimpleElevator;
import frc.robot.subsystems.CoralGrabber;
import frc.robot.subsystems.CoralGrabberArm;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class TimedAutoExample extends SequentialCommandGroup {
    public TimedAutoExample(
            SwerveDrivetrain swerve,
            SimpleElevator elevator,
            CoralGrabber coralGrabber,
            CoralGrabberArm coralArm,
            AlgaeGrabber algaeGrabber) {

        // Create trajectories
        Trajectory moveToScorePosition = createSimpleTrajectory(swerve, 
            new Pose2d(0, 0, new Rotation2d()), 
            new Pose2d(2, 0, new Rotation2d()));

        Trajectory moveToAlgaePosition = createSimpleTrajectory(swerve,
            new Pose2d(2, 0, new Rotation2d()),
            new Pose2d(1, 2, new Rotation2d()));

        addCommands(
            // 1. Move to scoring position
            new SwerveTrajectoryCommand(
                "Move To Score",
                swerve,
                moveToScorePosition,
                new Rotation2d()
            ),

            // 2. Raise elevator
            Commands.startEnd(
                () -> elevator.setSpeed(0.5),
                () -> elevator.stop(),
                elevator
            ).withTimeout(1.0),

            // 3. Position coral arm
            Commands.startEnd(
                () -> coralArm.setSpeed(0.3),
                () -> coralArm.stop(),
                coralArm
            ).withTimeout(0.5),

            // 4. Release coral
            Commands.startEnd(
                () -> coralGrabber.release(),
                () -> coralGrabber.stop(),
                coralGrabber
            ).withTimeout(0.3),

            // 5. Lower elevator and arm together
            Commands.parallel(
                Commands.startEnd(
                    () -> elevator.setSpeed(-0.3),
                    () -> elevator.stop(),
                    elevator
                ),
                Commands.startEnd(
                    () -> coralArm.setSpeed(-0.3),
                    () -> coralArm.stop(),
                    coralArm
                )
            ).withTimeout(0.8),

            // 6. Move to algae position
            new SwerveTrajectoryCommand(
                "Move To Algae",
                swerve,
                moveToAlgaePosition,
                new Rotation2d()
            ),

            // 7. Collection sequence
            Commands.sequence(
                // Position arm and elevator
                Commands.parallel(
                    Commands.startEnd(
                        () -> elevator.setSpeed(0.4),
                        () -> elevator.stop(),
                        elevator
                    ),
                    Commands.startEnd(
                        () -> coralArm.setSpeed(0.4),
                        () -> coralArm.stop(),
                        coralArm
                    )
                ).withTimeout(0.7),

                // Grab algae
                Commands.startEnd(
                    () -> algaeGrabber.grab(),
                    () -> algaeGrabber.stop(),
                    algaeGrabber
                ).withTimeout(0.5),

                // Pull down
                Commands.parallel(
                    Commands.startEnd(
                        () -> elevator.setSpeed(-0.3),
                        () -> elevator.stop(),
                        elevator
                    ),
                    Commands.startEnd(
                        () -> coralArm.setSpeed(-0.3),
                        () -> coralArm.stop(),
                        coralArm
                    )
                ).withTimeout(0.6)
            )
        );
    }

    private Trajectory createSimpleTrajectory(SwerveDrivetrain swerve, Pose2d start, Pose2d end) {
        return createTrajectory(swerve, start, end);
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