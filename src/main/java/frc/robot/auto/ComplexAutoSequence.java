package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;S
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.constants.Constants;
import java.util.List;

public class ComplexAutoSequence extends SequentialCommandGroup {
    private final SwerveDriveSubsystem swerveDrive;
    // private final ShooterSubsystem m_shooter;
    // private final IntakeSubsystem m_intake;

    public ComplexAutoSequence(
            SwerveDriveSubsystem driveSubsystem/*,
            ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem*/) {
        
        swerveDrive = driveSubsystem;
        // m_shooter = shooterSubsystem;
        // m_intake = intakeSubsystem;

        // Create trajectory configurations
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.SwerveDriveConstants.kinematics);

        // Create individual trajectories
        Trajectory trajectory1 = createTrajectory1(config);
        Trajectory trajectory2 = createTrajectory2(config);
        Trajectory trajectory3 = createTrajectory3(config);

        // Create the controller commands for each trajectory
        var thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0,
            Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveCommand1 = createSwerveCommand(trajectory1, thetaController);
        SwerveControllerCommand swerveCommand2 = createSwerveCommand(trajectory2, thetaController);
        SwerveControllerCommand swerveCommand3 = createSwerveCommand(trajectory3, thetaController);

        // Build the complete command sequence
        addCommands(
            // Reset odometry to starting pose
            new InstantCommand(() -> swerveDrive.resetOdometry(trajectory1.getInitialPose())),
            
            // First movement
            swerveCommand1,
            /* 
            // Shoot
            new InstantCommand(() -> m_shooter.prepare()),
            new WaitCommand(1.0), // Wait for shooter to reach speed
            new InstantCommand(() -> m_shooter.shoot()),
            new WaitCommand(0.5),
            new InstantCommand(() -> m_shooter.stop()),
            */
            // Second movement
            swerveCommand2,
            /* 
            // Intake
            new InstantCommand(() -> m_intake.start()),
            new WaitCommand(1.0),
            new InstantCommand(() -> m_intake.stop()),
            */
            // Final movement
            swerveCommand3
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

    private Trajectory createTrajectory1(TrajectoryConfig config) {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1, 1)
            ),
            new Pose2d(2, 2, new Rotation2d(0)),
            config
        );
    }

    private Trajectory createTrajectory2(TrajectoryConfig config) {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 2, new Rotation2d(0)),
            List.of(
                new Translation2d(3, 1)
            ),
            new Pose2d(4, 0, new Rotation2d(0)),
            config
        );
    }

    private Trajectory createTrajectory3(TrajectoryConfig config) {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(4, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(3, -1)
            ),
            new Pose2d(2, -2, new Rotation2d(0)),
            config
        );
    }
}