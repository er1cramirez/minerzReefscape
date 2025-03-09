package frc.robot.subsystems.swerve;

import java.util.HashMap;
import java.util.Map;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
// import frc.robot.telemetry.SwerveDrivetrainTelemetry;
import frc.robot.telemetry.SwerveTelemetry;
import frc.robot.util.DriveMode;

public class SwerveDrivetrain extends SubsystemBase{
    // Hardware
    private final TalonSwerveModule[] swerveModules;
    private final AHRS gyro;
    
    // Kinematics & Odometry
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private Pose2d pose;

    // Telemetry
    private final SwerveTelemetry telemetry;
    private final Map<String, Trajectory> trajectories = new HashMap<>();
    private String activeTrajectoryName = "";
    
    // State
    // private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    private boolean isFieldRelative = false;
    private DriveMode driveMode = DriveMode.NORMAL;

    /**
     * Constructs a SwerveDrivetrain object.
     */
    public SwerveDrivetrain(){
        // Initialize hardware
        swerveModules = new TalonSwerveModule[] {
            new TalonSwerveModule(Constants.SwerveConstants.frontLeftModuleConstants),
            new TalonSwerveModule(Constants.SwerveConstants.frontRightModuleConstants),
            new TalonSwerveModule(Constants.SwerveConstants.backLeftModuleConstants),
            new TalonSwerveModule(Constants.SwerveConstants.backRightModuleConstants)
        };
        gyro = new AHRS(/*USB */AHRS.NavXComType.kMXP_SPI);
        
        // Initialize kinematics and odometry
        kinematics = Constants.SwerveConstants.kinematics;
        resetRobotHeading();
        // Initialize odometry with initial position
        this.odometry = new SwerveDriveOdometry(
            kinematics,
            getRobotHeading(),
            getModulePositions(),
            new Pose2d()
        );
        
        pose = new Pose2d();
        // Initialize telemetry
        telemetry = new SwerveTelemetry(this);
        SmartDashboard.putData("Swerve Drive", telemetry);
        
    }

    // Methods
    /**
     * Drive the robot with given chassis speeds
     */
    public void drive(ChassisSpeeds speeds) {
        // if (isFieldRelative) {
        //     speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRobotHeading());
        // }
        // speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRobotHeading());
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
        drive(new ChassisSpeeds());
    }

    // get Yaw from gyro
    public Rotation2d getRobotHeading(){
        // CCW is positive
        return Rotation2d.fromDegrees(
            gyro.getAngle());
        // return new Rotation2d();
    }

    // Reset the robot heading(yaw)
    private void resetRobotHeading(){
        // Reset the gyro with the appropriate offset based on alliance
        gyro.reset();
    } 

    
    public Pose2d getPose() {
        return pose;
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(
            getRobotHeading(), 
            getModulePositions(), 
            pose);
        this.pose = pose;
    }

    /**
     * Gets the swerve drive kinematics.
     * @return The kinematics
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
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

    // Trajectory visualization methods
    public void addTrajectory(String name, Trajectory trajectory) {
        trajectories.put(name, trajectory);
    }

    public Map<String, Trajectory> getTrajectories(){
        return trajectories;
    }
    public void setActiveTrajectory(String name) {
        activeTrajectoryName = name;
    }

    public String getActiveTrajectoryName() {
        return activeTrajectoryName;
    }
    /**
     * Periodic method.
     */
    @Override
    public void periodic(){
        // Update odometry
        pose = odometry.update(
            getRobotHeading(),
            getModulePositions()
        );
        telemetry.updateTelemetry();
    }

    /**
     * Initializes the telemetry.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        telemetry.initTelemetry(builder);
    }

    public void setDriveMode(DriveMode mode) {
        this.driveMode = mode;
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }
}
