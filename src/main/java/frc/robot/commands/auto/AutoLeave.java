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
//import frc.robot.subsystems.SimpleElevator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class AutoLeave extends SequentialCommandGroup {

    private final Pose2d startPose = new Pose2d(0, 0, new Rotation2d());
    private final Pose2d goalPose = new Pose2d(0.5,0, new Rotation2d());

    // Trajectory trajectory = createTrajectory(swerve, startPose, goalPose);
    public AutoLeave(SwerveDrivetrain swerve, CoralGrabberArm coralArm, CoralGrabber coralGrabber) {
        Trajectory trajectory = createTrajectory(swerve, startPose, goalPose);  

        addCommands(
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
            // Commands.wait(1);
            new SwerveTrajectoryCommand(
                "Test Auto",
                swerve,
                trajectory,
                new Rotation2d()
            ),
            Commands.waitSeconds(2)
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
