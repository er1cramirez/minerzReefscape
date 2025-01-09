package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class TeleopDrive extends Command{
    // Subsystem
    private final SwerveDrivetrain swerve;
    
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
     * @param swerve The swerve drive subsystem
     * @param translationXSupplier Supplier for forward/backward movement (-1 to 1)
     * @param translationYSupplier Supplier for left/right movement (-1 to 1)
     * @param rotationSupplier Supplier for rotational movement (-1 to 1)
     */
    public TeleopDrive(
            SwerveDrivetrain swerve,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        
        this.swerve = swerve;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        
        // Initialize slew rate limiters
        this.xLimiter = new SlewRateLimiter(Constants.SwerveConstants.translationalSlew);
        this.yLimiter = new SlewRateLimiter(Constants.SwerveConstants.translationalSlew);
        this.rotLimiter = new SlewRateLimiter(Constants.SwerveConstants.rotationalSlew);
        
        addRequirements(swerve);
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
    public void execute() {
        // Process inputs
        double xSpeed = processInput(translationXSupplier.getAsDouble(), Constants.ControllersConstants.chassisControllerDeadband, xLimiter, Constants.SwerveConstants.maxSpeed);
        double ySpeed = processInput(translationYSupplier.getAsDouble(), Constants.ControllersConstants.chassisControllerDeadband, yLimiter, Constants.SwerveConstants.maxSpeed);
        double rot = processInput(rotationSupplier.getAsDouble(), Constants.ControllersConstants.chassisControllerDeadband, rotLimiter, Constants.SwerveConstants.maxAngularSpeed);

        // Calculate chassis speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, swerve.getRobotHeading());

        // Drive the swerve drive
        swerve.drive(speeds, swerve.isFieldRelative());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(), true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
