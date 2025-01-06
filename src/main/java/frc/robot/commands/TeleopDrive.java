package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain.CalibrationState;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;

/**
 * TeleopDrive implements manual control of the swerve drive system using provided velocity inputs.
 * This command processes raw inputs into smooth, controlled chassis movements.
 * 
 * <p>Features:
 * <ul>
 *   <li>Smooth acceleration using slew rate limiters</li>
 *   <li>Deadband application for precise control</li>
 *   <li>Support for both field-oriented and robot-oriented control</li>
 *   <li>Speed limiting based on configuration constants</li>
 *   <li>Input supplier flexibility for different control sources</li>
 * </ul>
 */
public class TeleopDrive extends Command {
    private static final double STOPPING_THRESHOLD = 0.01;

    // Subsystem
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    
    // Input suppliers
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    
    // Input processing
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter rotLimiter;

    /**
     * Creates a new TeleopDrive command.
     *
     * @param swerveDriveSubsystem The swerve drive subsystem
     * @param translationXSupplier Supplier for forward/backward movement (-1 to 1)
     * @param translationYSupplier Supplier for left/right movement (-1 to 1)
     * @param rotationSupplier Supplier for rotational movement (-1 to 1)
     */
    public TeleopDrive(
            SwerveDriveSubsystem swerveDriveSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        
        // Initialize slew rate limiters
        this.xLimiter = new SlewRateLimiter(Constants.SwerveDriveConstants.kTranslationalSlew);
        this.yLimiter = new SlewRateLimiter(Constants.SwerveDriveConstants.kTranslationalSlew);
        this.rotLimiter = new SlewRateLimiter(Constants.SwerveDriveConstants.kRotationalSlew);
        
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void execute() {
        // Don't process drive commands during calibration
        if (swerveDriveSubsystem.getSystemCalibrationState() == CalibrationState.IN_PROGRESS) {
            return;
        }
        
        // Get raw inputs from suppliers
        double xSpeed = translationXSupplier.getAsDouble();
        double ySpeed = translationYSupplier.getAsDouble();
        double rotSpeed = rotationSupplier.getAsDouble();
        
        // Process inputs with deadband and slew rate limiting
        xSpeed = processInput(
            xSpeed,
            Constants.ControllersConstants.chassisControllerDeadband,
            xLimiter,
            Constants.SwerveDriveConstants.kMaxSpeed
        );
        
        ySpeed = processInput(
            ySpeed,
            Constants.ControllersConstants.chassisControllerDeadband,
            yLimiter,
            Constants.SwerveDriveConstants.kMaxSpeed
        );
        
        rotSpeed = processInput(
            rotSpeed,
            Constants.ControllersConstants.chassisControllerDeadband,
            rotLimiter,
            Constants.SwerveDriveConstants.kMaxAngularSpeed
        );

        // Check if inputs are below stopping threshold
        if (Math.abs(xSpeed) < STOPPING_THRESHOLD && 
            Math.abs(ySpeed) < STOPPING_THRESHOLD && 
            Math.abs(rotSpeed) < STOPPING_THRESHOLD) {
            swerveDriveSubsystem.drive(new ChassisSpeeds());
            return;
        }

        // Create chassis speeds based on orientation mode
        ChassisSpeeds chassisSpeeds = swerveDriveSubsystem.isFieldOriented()
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rotSpeed,
                swerveDriveSubsystem.getGyroAngle()
            )
            : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        
        swerveDriveSubsystem.drive(chassisSpeeds);
    }

    /**
     * Processes a raw input value by applying deadband, slew rate limiting, and scaling.
     *
     * @param input Raw input value (-1 to 1)
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