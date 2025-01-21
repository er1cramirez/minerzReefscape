package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
// import frc.robot.telemetry.SwerveTelemetry;

public class SwerveTrajectoryCommand extends SwerveControllerCommand {
    private final SwerveDrivetrain swerve;
    private final Trajectory trajectory;
    private final String trajectoryName;
    
    public SwerveTrajectoryCommand(
            String name,
            SwerveDrivetrain swerve,
            Trajectory trajectory,
            Rotation2d targetRotation) {  // Remove swerveTelemetry parameter
        super(
            trajectory,
            swerve::getPose,
            swerve.getKinematics(),
            createXController(),
            createYController(),
            createThetaController(),
            () -> targetRotation,
            swerve::setModuleStates,
            swerve
        );
        
        this.swerve = swerve;
        this.trajectory = trajectory;
        this.trajectoryName = name;
    }

    @Override
    public void initialize() {
        swerve.resetPose(trajectory.getInitialPose());
        swerve.addTrajectory(trajectoryName, trajectory);
        swerve.setActiveTrajectory(trajectoryName);
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setActiveTrajectory("");
        super.end(interrupted);
    }

    // Add getter for trajectory
    protected Trajectory getTrajectory() {
        return trajectory;
    }

    private static PIDController createXController() {
        return new PIDController(AutoConstants.kPXController, 0, 0);
    }
    
    private static PIDController createYController() {
        return new PIDController(AutoConstants.kPYController, 0, 0);
    }
    
    private static ProfiledPIDController createThetaController() {
        var controller = new ProfiledPIDController(
            AutoConstants.kPThetaController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared)
        );
        controller.enableContinuousInput(-Math.PI, Math.PI);
        return controller;
    }
    
    
}