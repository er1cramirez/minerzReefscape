package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;

public class LeaveAuto extends SequentialCommandGroup {
    private final SwerveDriveSubsystem swerveDrive;

    private final Pose2d start = new Pose2d(0, 0, new Rotation2d(0));
    private final Pose2d end = new Pose2d(1, 1, new Rotation2d(0)); 

    public LeaveAuto(SwerveDriveSubsystem swerveDrive) {
        this.swerveDrive = swerveDrive;
        // Create trajectory configurations
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.SwerveDriveConstants.kinematics);

        Trajectory ouTrajectory = createTrajectory(config, start, end);

        // Create the controller commands for each trajectory
        var thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0,
            Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveCommand = createSwerveCommand(ouTrajectory, thetaController);

        // Build the complete command sequence
        addCommands(
            // Reset odometry to starting pose
            new InstantCommand(() -> swerveDrive.resetOdometry(ouTrajectory.getInitialPose())),
            swerveCommand
        );
    }

    private SwerveControllerCommand createSwerveCommand(
            Trajectory trajectory, 
            ProfiledPIDController thetaController) {
        
        return new SwerveControllerCommand(
            trajectory,
            swerveDrive::getPose,
            Constants.SwerveDriveConstants.kinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            swerveDrive::setModuleStates,
            swerveDrive
        );
    }

    private Trajectory createTrajectory(TrajectoryConfig config, Pose2d start, Pose2d end) {
        return TrajectoryGenerator.generateTrajectory(
            List.of(start, end),
            config
        );
    }
}
