package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class LeaveAuto extends SequentialCommandGroup {
    public LeaveAuto(SwerveDrivetrain swerve) {
        // Configure trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.MAX_SPEED,
            AutoConstants.MAX_ACCELERATION
        ).setKinematics(swerve.getKinematics());

        // Create PID controllers
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        
        // Create profiled PID controller for rotation
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared)
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create trajectory
        var trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d()),
            List.of(), // Interior waypoints
            new Pose2d(2, 0, new Rotation2d()),
            config
        );

        // Create swerve command with correct constructor
        var swerveCommand = new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            swerve.getKinematics(),
            xController,
            yController,
            thetaController,
            () -> new Rotation2d(),  // Desired rotation
            (states) -> swerve.setModuleStates(states),
            swerve
        );

        addCommands(
            new InstantCommand(() -> swerve.resetPose(trajectory.getInitialPose())),
            swerveCommand
        );
    }
}
