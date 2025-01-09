package frc.robot.subsystems.swerve;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.telemetry.SwerveDrivetrainTelemetry;

public class SwerveDrivetrain extends SubsystemBase{
    // Hardware
    private final SwerveModule[] swerveModules;
    private final AHRS gyro;
    
    // Kinematics & Odometry
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    
    // Telemetry
    private final SwerveDrivetrainTelemetry telemetry;
    
    // State
    // private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    private boolean isFieldRelative = true;

    /**
     * Constructs a SwerveDrivetrain object.
     */
    public SwerveDrivetrain(){
        // Initialize hardware
        swerveModules = new SwerveModule[] {
            new SwerveModule(Constants.SwerveConstants.frontLeftModuleConstants),
            new SwerveModule(Constants.SwerveConstants.frontRightModuleConstants),
            new SwerveModule(Constants.SwerveConstants.backLeftModuleConstants),
            new SwerveModule(Constants.SwerveConstants.backRightModuleConstants)
        };
        gyro = new AHRS(/*USB */AHRS.NavXComType.kUSB1);
        // Initialize kinematics and odometry
        kinematics = Constants.SwerveConstants.kinematics;
        odometry = new SwerveDriveOdometry(
            kinematics, 
            getRobotHeading(), 
            getModulePositions());
        // Initialize telemetry
        telemetry = new SwerveDrivetrainTelemetry(this);
        SmartDashboard.putData("SwerveDrivetrain", telemetry);
        resetRobotHeading(true);
    }

    // Methods
    /**
     * Drive the robot with given chassis speeds
     */
    public void drive(ChassisSpeeds speeds, boolean isFieldRelative) {
        if (isFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRobotHeading());
        }
        
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
        // currentSpeeds = speeds;
    }


    /**
     * Set individual module states directly
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxSpeed);
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(states[i]);
        }
    }

    public void stop() {
        drive(new ChassisSpeeds(), isFieldRelative);
    }

    // get Yaw from gyro
    public Rotation2d getRobotHeading(){
        // CCW is positive
        return Rotation2d.fromDegrees(gyro.getFusedHeading());
    }

    // Reset the robot heading(yaw)
    private void resetRobotHeading(boolean isRedAlliance){
        // Reset the gyro
    } 

    
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getRobotHeading(), getModulePositions(), pose);
    }

    

    public SwerveModuleState[] getCurrentStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getCurrentState();
        }
        return states;
    }

    public SwerveModuleState[] getTargetStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getTargetState();
        }
        return states;
    }

    private SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < swerveModules.length; i++) {
            positions[i] = swerveModules[i].getModulePosition();
        }
        return positions;
    }

    public boolean isFieldRelative() {
        return isFieldRelative;
    }

    public void setFieldRelative(boolean isFieldRelative) {
        this.isFieldRelative = isFieldRelative;
    }

    public void setLockMode(){
        // Set the swerve modules to lock mode(x pattern)
    }

    public void testEachModule() {
        final double TEST_SPEED = 0.3;
        final double TEST_DURATION = 1.0;
        Timer timer = new Timer();
    
        for (int i = 0; i < swerveModules.length; i++) {
            // Test drive motor
            timer.reset();
            timer.start();
            while (timer.get() < TEST_DURATION) {
                SwerveModuleState testState = new SwerveModuleState(TEST_SPEED, new Rotation2d());
                swerveModules[i].setDesiredState(testState);
                
                if (Math.abs(swerveModules[i].getCurrentState().speedMetersPerSecond) < 0.1) {
                    // Logger.getInstance().recordOutput("TestFailed/Module" + i + "/Drive", true);
                }
            }
            
            // Test turning motor
            timer.reset();
            while (timer.get() < TEST_DURATION) {
                SwerveModuleState testState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(90));
                swerveModules[i].setDesiredState(testState);
                
                if (Math.abs(swerveModules[i].getCurrentState().angle.getDegrees() - 90) > 5) {
                    // Logger.getInstance().recordOutput("TestFailed/Module" + i + "/Turn", true);
                }
            }
            
            // Stop module
            swerveModules[i].setDesiredState(new SwerveModuleState());
        }
    }
    /**
     * Periodic method.
     */
    @Override
    public void periodic(){
        // This method will be called once per scheduler run
    }

    /**
     * Initializes the telemetry.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        telemetry.initTelemetry(builder);
    }
}
