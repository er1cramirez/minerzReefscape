package frc.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;

public class SwerveDriveSubsystem extends SubsystemBase {
    // Modules
    private final SwerveModule[] swerveModules;

    // Kinematics
    private final double chassisWidth = Units.inchesToMeters(20);
    private final double chassisLength = Units.inchesToMeters(20);
    // The locations for the modules must be relative to the center of the robot. Positive x values represent moving toward the front of the robot whereas positive y values represent moving toward the left of the robot.
    private final Translation2d frontLeftLocation = new Translation2d(chassisWidth / 2, chassisLength / 2);
    private final Translation2d frontRightLocation = new Translation2d(chassisWidth / 2, -chassisLength / 2);
    private final Translation2d backLeftLocation = new Translation2d(-chassisWidth / 2, chassisLength / 2);
    private final Translation2d backRightLocation = new Translation2d(-chassisWidth / 2, -chassisLength / 2);

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    
    CommandXboxController driverController;

    // Field visualization
    private final Field2d field = new Field2d();

    // For debugging purposes
    // private final StructArrayPublisher<SwerveModuleState> currentStatesPublisher;
    // private final StructArrayPublisher<SwerveModuleState> targetStatesPublisher;
    // SwerveModuleState[] currentStates;
    // SwerveModuleState[] targetStates;

    public SwerveDriveSubsystem(CommandXboxController driverController) {
        swerveModules = new SwerveModule[] {
            new SwerveModule(1,2,16, new Rotation2d()),
            new SwerveModule(5,6,19, new Rotation2d(Math.PI)),
            new SwerveModule(3,4,17, new Rotation2d()),
            new SwerveModule(7,8,18, new Rotation2d(Math.PI))
        };
        this.driverController = driverController;

        // currentStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveCurrentStates", SwerveModuleState.struct).publish();
        // targetStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveTargetStates", SwerveModuleState.struct).publish();
    }

    @Override
    public void periodic() {

        setDesiredStates(getStatesFromSpeeds(getSpeedsFromController()));
        // This method will be called once per scheduler run
        // currentStates = new SwerveModuleState[] {
        //     frontLeftModule.getState(),
        //     frontRightModule.getState(),
        //     backLeftModule.getState(),
        //     backRightModule.getState()
        // };

        // targetStates = new SwerveModuleState[] {
        //     frontLeftModule.getTargetState(),
        //     frontRightModule.getTargetState(),
        //     backLeftModule.getTargetState(),
        //     backRightModule.getTargetState()
        // };

        // currentStatesPublisher.set(currentStates);
        // targetStatesPublisher.set(targetStates);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setDesiredStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(desiredStates[i]);
        }
    }

    public SwerveModuleState[] getStatesFromSpeeds(ChassisSpeeds speeds) {
        return swerveKinematics.toSwerveModuleStates(speeds);
    }

    public ChassisSpeeds getSpeedsFromController() {
        double forward = -3*driverController.getLeftY();
        double strafe = -3*driverController.getLeftX();
        double rotation = driverController.getRightX();

        return new ChassisSpeeds(forward, strafe, rotation);
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("YAGSL Swerve Drive");
        
        // Helper method to convert module states to array
        builder.addDoubleArrayProperty("measuredStates", 
            () -> getModuleStateArray(true), null);
            
        builder.addDoubleArrayProperty("desiredStates",
            () -> getModuleStateArray(false), null);

        builder.addDoubleProperty("robotRotation",
            () -> new Rotation2d().getRadians(), null);
        
        builder.addDoubleProperty("maxSpeed",
            () -> Constants.SwerveDriveConstants.kMaxSpeed, null);
        
        // builder.addDoubleProperty("sizeLeftRight",
        //     () -> Constants.SwerveDriveConstants.kTrackWidth, null);
        
        // builder.addDoubleProperty("sizeFrontBack",
        //     () -> Constants.SwerveDriveConstants.kWheelBase, null);
            
        builder.addStringProperty("rotationUnit",
            () -> "radians", null);
    }

    /**
     * Converts module states to an array format expected by the YAGSL widget
     * @param current true for current states, false for target states
     * @return array of alternating angles (rad) and velocities (m/s)
     */
    private double[] getModuleStateArray(boolean current) {
        return Arrays.stream(swerveModules)
            .flatMap(module -> {
                SwerveModuleState state = current ? module.getState() : module.getTargetState();
                return Stream.of(
                    state.angle.getRadians(),
                    state.speedMetersPerSecond
                );
            })
            .mapToDouble(Double::valueOf)
            .toArray();
    }

}