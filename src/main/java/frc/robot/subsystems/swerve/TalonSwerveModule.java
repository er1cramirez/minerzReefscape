package frc.robot.subsystems.swerve;

import static frc.robot.util.PhoenixUtil.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Mk4iModuleConstants;

public class TalonSwerveModule extends SubsystemBase {
    // Drive motor (NEO with SparkMax)
    private final SparkMax drivingMotor;
    private final RelativeEncoder drivingEncoder;
    private final SparkClosedLoopController drivingController;

    // Steering motor (TalonFX with CANcoder)
    private final TalonFX steeringMotor;
    private final CANcoder steerAbsoluteEncoder;

    // Configuration
    private final Mk4iModuleConstants moduleConstants;

    // State variables
    private SwerveModuleState currentState;
    private SwerveModuleState targetState;

    public TalonSwerveModule(Mk4iModuleConstants moduleConstants) {
        this.moduleConstants = moduleConstants;

        // Initialize drive hardware (NEO with SparkMax)
        drivingMotor = new SparkMax(moduleConstants.drivingMotorID(), MotorType.kBrushless);
        drivingEncoder = drivingMotor.getEncoder();
        drivingController = drivingMotor.getClosedLoopController();
        configureDriveMotor();

        // Initialize steering hardware (TalonFX with CANcoder)
        steeringMotor = new TalonFX(moduleConstants.steeringMotorID(), "can_fd");
        steerAbsoluteEncoder = new CANcoder(moduleConstants.steerAbsoluteEncoderID(), "can_fd");
        configureSteeringMotor();
        
        // Initialize states
        currentState = new SwerveModuleState(0, getSteerAngle());
        targetState = new SwerveModuleState(0, currentState.angle);
    }

    private void configureDriveMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(moduleConstants.isDriveMotorInverted());
        config.smartCurrentLimit(Constants.MotorConstants.driveCurrentLimit);
        config.voltageCompensation(Constants.MotorConstants.voltageCompensation);
        config.idleMode(Constants.MotorConstants.driveIdleMode);

        // Set conversion factors
        double drivingFactor = Constants.SwerveConstants.Mk4iMechanicalConstants.drivingFactor;
        config.encoder.positionConversionFactor(drivingFactor);
        config.encoder.velocityConversionFactor(drivingFactor / 60.0);

        // Configure PID
        config.closedLoop.p(moduleConstants.drivingPIDConstants().kP());
        config.closedLoop.i(moduleConstants.drivingPIDConstants().kI());
        config.closedLoop.d(moduleConstants.drivingPIDConstants().kD());
        config.closedLoop.minOutput(moduleConstants.drivingPIDConstants().kMinOutput());
        config.closedLoop.maxOutput(moduleConstants.drivingPIDConstants().kMaxOutput());

        drivingMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        drivingEncoder.setPosition(0);
    }

    private void configureSteeringMotor() {
        var turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0.kP = moduleConstants.steeringPIDConstants().kP();
        turnConfig.Slot0.kI = moduleConstants.steeringPIDConstants().kI();
        turnConfig.Slot0.kD = moduleConstants.steeringPIDConstants().kD();
        turnConfig.Feedback.FeedbackRemoteSensorID = steerAbsoluteEncoder.getDeviceID();
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnConfig.Feedback.RotorToSensorRatio = Constants.SwerveConstants.Mk4iMechanicalConstants.steeringFactor;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / Constants.SwerveConstants.Mk4iMechanicalConstants.steeringFactor;
        turnConfig.MotionMagic.MotionMagicAcceleration =
            turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.SwerveConstants.Mk4iMechanicalConstants.steeringFactor;
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.MotorOutput.Inverted =
            moduleConstants.isSteerMotorInverted() ? 
                InvertedValue.Clockwise_Positive : 
                InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> steeringMotor.getConfigurator().apply(turnConfig, 0.25));
        // steeringMotor.
        /* Configure CANcoder to zero the magnet appropriately */
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // cc_cfg.MagnetSensor.MagnetOffset = 0.4;
        steerAbsoluteEncoder.getConfigurator().apply(cc_cfg);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Update current state before optimization
        updateCurrentState();

        // Create mutable copy of desired state
        SwerveModuleState optimizedState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle
        );

        // Optimize the reference state to avoid spinning further than 90 degrees
        optimizedState.optimize(currentState.angle);

        // Update target state after optimization
        targetState = desiredState;

        // Apply speed scaling based on angle difference
        optimizedState.cosineScale(currentState.angle);
        // Set motor references

        // Set motor references
        drivingController.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
        final PositionVoltage anglePosition = new PositionVoltage(0);
        steeringMotor.setControl(anglePosition.withPosition(optimizedState.angle.getRotations()));
    }

    private double getDriveVelocity() {
        return drivingEncoder.getVelocity();
    }

    private Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(steeringMotor.getPosition().getValueAsDouble());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            getSteerAngle()
        );
    }

    private void updateCurrentState() {
        currentState = new SwerveModuleState(
            getDriveVelocity(),
            getSteerAngle()
        );
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            getSteerAngle());
    }

    public SwerveModuleState getCurrentState() {
        return currentState;
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    

    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     builder.setSmartDashboardType("Talon Swerve Module");
    //     System.out.println("FX Position: " + fx_pos.toString());
    //     System.out.println("CANcoder Position: " + cc_pos.toString());
}
