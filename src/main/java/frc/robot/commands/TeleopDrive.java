package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.InputProcessor;

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
        this.xLimiter = new SlewRateLimiter(
            Constants.SwerveConstants.translationalSlew);
        this.yLimiter = new SlewRateLimiter(
            Constants.SwerveConstants.translationalSlew);
        this.rotLimiter = new SlewRateLimiter(
            Constants.SwerveConstants.rotationalSlew);
        addRequirements(swerve);
    }
    @Override
    public void execute() {
        // Process inputs
        double xSpeed = InputProcessor.processInput(
            translationXSupplier.getAsDouble(), 
            Constants.ControllersConstants.chassisControllerDeadband, 
            xLimiter, Constants.SwerveConstants.maxSpeed);
        double ySpeed = InputProcessor.processInput(
            translationYSupplier.getAsDouble(), 
            Constants.ControllersConstants.chassisControllerDeadband, 
            yLimiter, Constants.SwerveConstants.maxSpeed);
        double rot = InputProcessor.processInput(
            rotationSupplier.getAsDouble(), 
            Constants.ControllersConstants.chassisControllerDeadband, 
            rotLimiter, Constants.SwerveConstants.maxSpeed);

        // Calculate chassis speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, swerve.getRobotHeading());

        // Drive the swerve drive
        swerve.drive(speeds, swerve.isFieldRelative());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(), false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
