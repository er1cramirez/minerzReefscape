package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;

/**
 * TeleopDrive implements manual control of the swerve drive system using an Xbox controller.
 * 
 * <p>Features:
 * <ul>
 *   <li>Smooth acceleration using slew rate limiters</li>
 *   <li>Deadband application for precise control</li>
 *   <li>Support for both field-oriented and robot-oriented control</li>
 *   <li>Speed limiting based on configuration constants</li>
 * </ul>
 */
public class TeleopDrive extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final XboxController controller;
    
    // Input processing
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter rotLimiter;

    /**
     * Creates a new TeleopDrive command.
     *
     * @param swerveDriveSubsystem The subsystem used by this command
     * @param controller The Xbox controller used for driver input
     */
    public TeleopDrive(SwerveDriveSubsystem swerveDriveSubsystem, XboxController controller) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.controller = controller;
        
        // Initialize slew rate limiters with configured rates
        this.xLimiter = new SlewRateLimiter(Constants.SwerveDriveConstants.kTranslationalSlew);
        this.yLimiter = new SlewRateLimiter(Constants.SwerveDriveConstants.kTranslationalSlew);
        this.rotLimiter = new SlewRateLimiter(Constants.SwerveDriveConstants.kRotationalSlew);
        
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void execute() {
        // Get and process joystick inputs
        double xSpeed = processInput(
            controller.getLeftX(),
            Constants.ControllersConstants.chassisControllerDeadband,
            xLimiter,
            Constants.SwerveDriveConstants.kMaxSpeed
        );
        
        double ySpeed = processInput(
            controller.getLeftY(),
            Constants.ControllersConstants.chassisControllerDeadband,
            yLimiter,
            Constants.SwerveDriveConstants.kMaxSpeed
        );
        
        double rotSpeed = processInput(
            controller.getRightX(),
            Constants.ControllersConstants.chassisControllerDeadband,
            rotLimiter,
            Constants.SwerveDriveConstants.kMaxAngularSpeed
        );

        // Create and apply chassis speeds
        ChassisSpeeds chassisSpeeds = swerveDriveSubsystem.isFieldOriented()
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerveDriveSubsystem.getGyroAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        
        swerveDriveSubsystem.drive(chassisSpeeds);
    }

    /**
     * Processes a raw input value by applying deadband, slew rate limiting, and scaling.
     *
     * @param input Raw input value from controller
     * @param deadband Deadband value to apply
     * @param limiter SlewRateLimiter for smooth acceleration
     * @param maxSpeed Maximum speed to scale to
     * @return Processed input value
     */
    private double processInput(double input, double deadband, SlewRateLimiter limiter, double maxSpeed) {
        return limiter.calculate(MathUtil.applyDeadband(input, deadband)) * maxSpeed;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}