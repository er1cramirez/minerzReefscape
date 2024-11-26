package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SwerveDriveSubsystem extends SubsystemBase {
    // Modules
    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    // Kinematics
    private final double chassisWidth = Units.inchesToMeters(24);
    private final double chassisLength = Units.inchesToMeters(24);
    // The locations for the modules must be relative to the center of the robot. Positive x values represent moving toward the front of the robot whereas positive y values represent moving toward the left of the robot.
    private final Translation2d frontLeftLocation = new Translation2d(chassisWidth / 2, chassisLength / 2);
    private final Translation2d frontRightLocation = new Translation2d(chassisWidth / 2, -chassisLength / 2);
    private final Translation2d backLeftLocation = new Translation2d(-chassisWidth / 2, chassisLength / 2);
    private final Translation2d backRightLocation = new Translation2d(-chassisWidth / 2, -chassisLength / 2);

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    
    CommandXboxController driverController;

    // For debugging purposes
    private final StructArrayPublisher<SwerveModuleState> currentStatesPublisher;
    private final StructArrayPublisher<SwerveModuleState> targetStatesPublisher;
    SwerveModuleState[] currentStates;
    SwerveModuleState[] targetStates;

    public SwerveDriveSubsystem(CommandXboxController driverController) {
        frontLeftModule = new SwerveModule(1, 2, 3);
        frontRightModule = new SwerveModule(4, 5, 6);
        backLeftModule = new SwerveModule(7, 8, 9);
        backRightModule = new SwerveModule(10, 11, 12);

        this.driverController = driverController;

        currentStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveCurrentStates", SwerveModuleState.struct).publish();
        targetStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveTargetStates", SwerveModuleState.struct).publish();
    }

    @Override
    public void periodic() {

        setDesiredStates(getStatesFromSpeeds(getSpeedsFromController()));
        // This method will be called once per scheduler run
        currentStates = new SwerveModuleState[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };

        targetStates = new SwerveModuleState[] {
            frontLeftModule.getTargetState(),
            frontRightModule.getTargetState(),
            backLeftModule.getTargetState(),
            backRightModule.getTargetState()
        };

        currentStatesPublisher.set(currentStates);
        targetStatesPublisher.set(targetStates);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setDesiredStates(SwerveModuleState[] desiredStates) {
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getStatesFromSpeeds(ChassisSpeeds speeds) {
        return swerveKinematics.toSwerveModuleStates(speeds);
    }

    public ChassisSpeeds getSpeedsFromController() {
        double forward = -driverController.getLeftY();
        double strafe = driverController.getLeftX();
        double rotation = driverController.getRightX();

        return new ChassisSpeeds(forward, strafe, rotation);
    }

}