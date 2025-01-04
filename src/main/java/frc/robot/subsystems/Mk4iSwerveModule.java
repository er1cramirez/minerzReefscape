package frc.robot.subsystems;
// Rev Robotics imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
// CTRE imports
import com.ctre.phoenix6.hardware.CANcoder;

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants;
import frc.robot.constants.Mk4iSwerveModuleConstants;
import frc.robot.constants.PIDConstants;



public class Mk4iSwerveModule {
    // Hardware
    // * Motors
    private final CANSparkMax drivingMotor;
    private final CANSparkMax steeringMotor;
    // * Encoders
    private final RelativeEncoder drivingEncoder;
    private final RelativeEncoder steeringEncoder;
    private final CANcoder steerAbsoluteEncoder;

    // Motor Controllers
    private final SparkPIDController drivingController;
    // private final SimpleMotorFeedforward drivingFeedforward;
    private final SparkPIDController steeringController;
    // private final SimpleMotorFeedforward steeringFeedforward;

    // Constants
    // private SwerveModuleState currentState;
    private SwerveModuleState targetState;
    private int resetIteration = 0;
    private final Rotation2d chassisAngularOffset;
    private final PIDConstants drivingPIDConstants;
    private final PIDConstants steeringPIDConstants;

    // Constructor
    public Mk4iSwerveModule(Mk4iSwerveModuleConstants moduleConstants) {
        drivingMotor = new CANSparkMax(moduleConstants.drivingMotorID(), MotorType.kBrushless);
        steeringMotor = new CANSparkMax(moduleConstants.drivingMotorID(), MotorType.kBrushless);
        drivingEncoder = drivingMotor.getEncoder();
        steeringEncoder = steeringMotor.getEncoder();
        steerAbsoluteEncoder = new CANcoder(moduleConstants.steerAbsoluteEncoderID());
        chassisAngularOffset = moduleConstants.steerAngleOffset();
        // com.ctre.phoenix6.hardware.ParentDevice.optimizeBusUtilizationForAll(20, steerAbsoluteEncoder);
        drivingController = drivingMotor.getPIDController();
        // drivingFeedforward = new SimpleMotorFeedforward(Constants.SwerveDriveConstants.driveKS, Constants.SwerveDriveConstants.driveKV, Constants.SwerveDriveConstants.driveKA);
        steeringController = steeringMotor.getPIDController();
        // steeringFeedforward = new SimpleMotorFeedforward(Constants.SwerveDriveConstants.driveKS, Constants.SwerveDriveConstants.driveKV, Constants.SwerveDriveConstants.driveKA);
        drivingPIDConstants = moduleConstants.drivingPIDConstants();
        steeringPIDConstants = moduleConstants.steeringPIDConstants();
        initializeModule();
    }
    // Methods
    private void initializeModule() {
        canCoderConfig();
        drivingConfig();
        steeringConfig();
        
        // currentState = new SwerveModuleState(0, Rotation2d.fromRadians(steeringEncoder.getPosition()));
        setDesiredState(getState());// For avoiding the module to move when the robot is enabled
    }

    private void canCoderConfig(){
        // while (steerAbsoluteEncoder.optimizeBusUtilization(50, 0.05) != com.ctre.phoenix6.StatusCode.OK);

        // Set the CANCoder to absolute position
        // com.ctre.phoenix6.
        // steerAbsoluteEncoder.configSensorInitializationStrategy(CANCoder.SensorInitializationStrategy.BootToAbsolutePosition, 0);
    }

    private void drivingConfig(){
        drivingMotor.restoreFactoryDefaults();
        drivingMotor.setIdleMode(IdleMode.kBrake);
        drivingMotor.setSmartCurrentLimit(50);
        // Encoder
        
        drivingEncoder.setPositionConversionFactor(
            /* Conversion Factor = drivingFactor*/Constants.SwerveDriveConstants.Mk4iMechanicalConstants.drivingFactor);
        drivingEncoder.setVelocityConversionFactor(
            /* Conversion Factor = drivingFactor / 60.0*/Constants.SwerveDriveConstants.Mk4iMechanicalConstants.drivingFactor/60.0);
        drivingEncoder.setPosition(0);
        // Closed Loop
        drivingController.setFeedbackDevice(drivingEncoder);
        drivingController.setP(drivingPIDConstants.kP());
        drivingController.setI(drivingPIDConstants.kI());
        drivingController.setD(drivingPIDConstants.kD());
        drivingController.setOutputRange(drivingPIDConstants.kMinOutput(), drivingPIDConstants.kMaxOutput());
        drivingMotor.burnFlash();
    }
    
    private void steeringConfig(){
        steeringMotor.restoreFactoryDefaults();
        steeringMotor.setIdleMode(IdleMode.kBrake);
        steeringMotor.setSmartCurrentLimit(20);
        steeringMotor.setInverted(true);
        // Encoder
        
        // steeringEncoder.setInverted(isEncoderInverted);
        steeringEncoder.setPositionConversionFactor(
            /* Conversion Factor = steeringFactor*/Constants.SwerveDriveConstants.Mk4iMechanicalConstants.steeringFactor);
        steeringEncoder.setVelocityConversionFactor(
            /* Conversion Factor = steeringFactor / 60.0*/Constants.SwerveDriveConstants.Mk4iMechanicalConstants.steeringFactor/60.0);
        resetSteeringEncoderToAbsolute();
        // Closed Loop
        steeringController.setFeedbackDevice(steeringEncoder);
        steeringController.setP(steeringPIDConstants.kP());
        steeringController.setI(steeringPIDConstants.kI());
        steeringController.setD(steeringPIDConstants.kD());
        steeringController.setOutputRange(steeringPIDConstants.kMinOutput(), steeringPIDConstants.kMaxOutput());
        // Position wrapping enabled
        steeringController.setPositionPIDWrappingEnabled(true);
        steeringController.setPositionPIDWrappingMinInput(- Math.PI);
        steeringController.setPositionPIDWrappingMaxInput(Math.PI);
        steeringMotor.burnFlash();
    }

    public void setDrivePID(double p, double i, double d){
        drivingController.setP(p);
        drivingController.setI(i);
        drivingController.setD(d);
    }

    public void setSteeringPID(double p, double i, double d){
        steeringController.setP(p);
        steeringController.setI(i);
        steeringController.setD(d);
    }

    private void resetSteeringEncoderToAbsolute(){
        // Reset the steering encoder to the absolute position
        steeringEncoder.setPosition(Rotation2d.fromRotations(steerAbsoluteEncoder.getAbsolutePosition().getValue()).minus(chassisAngularOffset).getRadians());

    }

    public SwerveModuleState getState() {//revisar
        return new SwerveModuleState(drivingEncoder.getVelocity(), Rotation2d.fromRadians(steeringEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivingEncoder.getPosition(), Rotation2d.fromRadians(steeringEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (++resetIteration >= 1000) {
            if (steeringEncoder.getVelocity() < 0.1) {
                resetIteration = 0;
                resetSteeringEncoderToAbsolute();
                System.out.println("Resetting steering encoder to absolute");
            }
        }
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(getState().angle).getCos();
        // Calculate the control output
        drivingController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);//, 0, drivingFeedforward.calculate(desiredState.speedMetersPerSecond));
        // m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        steeringController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
        // steeringController.setReference(desiredState.angle.getRadians(), ControlType.kPosition, 0, steeringFeedforward.calculate(desiredState.angle.getRadians()));
        // m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    
        targetState = desiredState;
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public void updateModule() {
        // Update the module

    }
}
