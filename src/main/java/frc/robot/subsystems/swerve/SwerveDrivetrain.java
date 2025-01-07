package frc.robot.subsystems.swerve;

import java.util.Arrays;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrivetrain extends SubsystemBase{

    // Constants

    // Hardware
    private final SwerveModule[] swerveModules;
    private final AHRS gyro;

    // Kinematics and odometry
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    // Configuration

    // State variables

    // Constructor

    /**
     * Constructs a SwerveDrivetrain object.
     */
    public SwerveDrivetrain(){
        // Initialize hardware
        swerveModules = initializeSwerveModules();
        gyro = new AHRS(/*USB */AHRS.NavXComType.kUSB1);
        // Initialize kinematics and odometry
        kinematics = Constants.SwerveConstants.kinematics;
        odometry = new SwerveDriveOdometry(kinematics, getRobotHeading(), getModulePositions());
    }

    // Methods
    private SwerveModule[] initializeSwerveModules() {
        return new SwerveModule[] {
            new SwerveModule(Constants.SwerveConstants.frontLeftModuleConstants),
            new SwerveModule(Constants.SwerveConstants.frontRightModuleConstants),
            new SwerveModule(Constants.SwerveConstants.backLeftModuleConstants),
            new SwerveModule(Constants.SwerveConstants.backRightModuleConstants)
        };
    }

    private Rotation2d getRobotHeading(){
        // CCW is positive
        return Rotation2d.fromDegrees(gyro.getFusedHeading());
    }

    private void resetGyro(boolean isRedAlliance){
        // Reset the gyro

    }

    private SwerveModulePosition[] getModulePositions(){
        return Arrays.stream(swerveModules)
            .map(module -> module.getModulePosition())
            .toArray(SwerveModulePosition[]::new);
    }

    /**
     * Drives the robot using the given x, y, and rotation values.
     * 
     * @param x The x value
     * @param y The y value
     * @param rotation The rotation value
     */
    public void drive(double x, double y, double rotation){
        // Calculate the module states
    }

    /**
     * Updates the module states.
     */
    public void updateModuleStates(){
        // Update the module states
    }

    /**
     * Updates the module states to the target states.
     */
    public void updateModuleStatesToTarget(){
        // Update the module states to the target states
    }

    /**
     * Updates the module states to the current states.
     */
    public void updateModuleStatesToCurrent(){
        // Update the module states to the current states
    }

    /**
     * Sets the target states of the modules.
     * 
     * @param targetStates The target states
     */
    public void setTargetStates(SwerveModuleState[] targetStates){
        // Set the target states of the modules
    }

    /**
     * Sets the target states of the modules to the current states.
     */
    public void setTargetStatesToCurrent(){
        // Set the target states of the modules to the current states
    }

    /**
     * Sets the target states of the modules to the given states.
     * 
     * @param states The states to set the target states to
     */
    public void setTargetStatesTo(SwerveModuleState[] states){
        // Set the target states of the modules to the given states
    }

    /**
     * Periodic method.
     */
    @Override
    public void periodic(){
        // This method will be called once per scheduler run
    }

    // Telemetry initialization

    /**
     * Initializes the telemetry.
     */
    @Override
    public void initSendable(SendableBuilder builder){
        // Initialize the telemetry
    }
    
}
