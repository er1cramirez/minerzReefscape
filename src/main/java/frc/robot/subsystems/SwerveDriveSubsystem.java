package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;

public class SwerveDriveSubsystem extends SubsystemBase {
    // Modules
    private Mk4iSwerveModule frontLeftModule;
    private Mk4iSwerveModule frontRightModule;
    private Mk4iSwerveModule backLeftModule;
    private Mk4iSwerveModule backRightModule;

    // Kinematics
    private final SwerveDriveKinematics swerveDriveKinematics = Constants.SwerveDriveConstants.kinematics;

    CommandXboxController driverController;

    // For debugging purposes
    private final StructArrayPublisher<SwerveModuleState> currentStatesPublisher;
    private final StructArrayPublisher<SwerveModuleState> targetStatesPublisher;
    SwerveModuleState[] currentStates;
    SwerveModuleState[] targetStates;

    public SwerveDriveSubsystem(CommandXboxController driverController) {
        frontLeftModule = new Mk4iSwerveModule(Constants.SwerveDriveConstants.frontLeftModuleConstants);
        frontRightModule = new Mk4iSwerveModule(Constants.SwerveDriveConstants.frontRightModuleConstants);
        backLeftModule = new Mk4iSwerveModule(Constants.SwerveDriveConstants.backLeftModuleConstants);
        backRightModule = new Mk4iSwerveModule(Constants.SwerveDriveConstants.backRightModuleConstants);

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
        return swerveDriveKinematics.toSwerveModuleStates(speeds);
    }

    public ChassisSpeeds getSpeedsFromController() {
        double forward = -driverController.getLeftY();
        double strafe = -driverController.getLeftX();
        double rotation = driverController.getRightX();

        return new ChassisSpeeds(forward, strafe, rotation);
    }

}