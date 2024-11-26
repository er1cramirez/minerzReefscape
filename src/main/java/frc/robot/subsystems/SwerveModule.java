package frc.robot.subsystems;
// Rev Robotics imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
// CTRE imports
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;



public class SwerveModule {
    // Hardware
    private final CANSparkMax drivingMotor;
    private final CANSparkMax steeringMotor;
    // Encoders
    private final RelativeEncoder drivingEncoder;
    private final RelativeEncoder steeringEncoder;
    private final CANcoder steerAbsoluteEncoder;

    // Motor Controllers
    private final SparkPIDController drivingController;
    private final SparkPIDController steeringController;

    // Constants
    private SwerveModuleState currentState;
    private SwerveModuleState targetState;
    private final Rotation2d chassisAngularOffset;

    // Constructor
    public SwerveModule(int drivingMotorID, int steeringMotorID, int steerEncoderID) {
        drivingMotor = new CANSparkMax(drivingMotorID, MotorType.kBrushless);
        steeringMotor = new CANSparkMax(steeringMotorID, MotorType.kBrushless);
        
        drivingEncoder = drivingMotor.getEncoder();
        steeringEncoder = steeringMotor.getEncoder();
        steerAbsoluteEncoder = new CANcoder(steerEncoderID);
        // com.ctre.phoenix6.hardware.ParentDevice.optimizeBusUtilizationForAll(20, steerAbsoluteEncoder);

        drivingController = drivingMotor.getPIDController();
        steeringController = steeringMotor.getPIDController();
        this.chassisAngularOffset = new Rotation2d();
        initializeModule();
    }
    // Methods
    private void initializeModule() {
        canCoderConfig();
        drivingConfig();
        steeringConfig(false);
        
        currentState = new SwerveModuleState(0, Rotation2d.fromRadians(steeringEncoder.getPosition()));
    }

    private void canCoderConfig(){
        // while (steerAbsoluteEncoder.optimizeBusUtilization(50, 0.05) != com.ctre.phoenix6.StatusCode.OK);

        // Set the CANCoder to absolute position
        // com.ctre.phoenix6.
        // steerAbsoluteEncoder.configSensorInitializationStrategy(CANCoder.SensorInitializationStrategy.BootToAbsolutePosition, 0);
    }

    private void drivingConfig(){
        drivingMotor.setIdleMode(IdleMode.kBrake);
        drivingMotor.setSmartCurrentLimit(50);
        // Encoder
        // double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
        // double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;
        drivingEncoder.setPositionConversionFactor(/* Conversion Factor = drivingFactor*/0.5);
        drivingEncoder.setVelocityConversionFactor(/* Conversion Factor = drivingFactor / 60.0*/0.5);
        drivingEncoder.setPosition(0);
        // Closed Loop
        drivingController.setFeedbackDevice(drivingEncoder);
        drivingController.setP(0.04);
        drivingController.setI(0);
        drivingController.setD(0);
        drivingController.setOutputRange(-1, 1);
    }
    
    private void steeringConfig(boolean isEncoderInverted){
        steeringMotor.setIdleMode(IdleMode.kBrake);
        steeringMotor.setSmartCurrentLimit(20);
        // Encoder
        // double steeringFactor = 2 * Math.PI;
        steeringEncoder.setInverted(isEncoderInverted);
        steeringEncoder.setPositionConversionFactor(/* Conversion Factor = steeringFactor*/0.5);
        steeringEncoder.setVelocityConversionFactor(/* Conversion Factor = steeringFactor / 60.0*/0.5);
        resetSteeringEncoderToAbsolute();
        // Closed Loop
        steeringController.setFeedbackDevice(steeringEncoder);
        steeringController.setP(1);
        steeringController.setI(0);
        steeringController.setD(0);
        steeringController.setOutputRange(-1, 1);
        // Position wrapping enabled
        steeringController.setPositionPIDWrappingEnabled(true);
        steeringController.setPositionPIDWrappingMinInput(0);
        steeringController.setPositionPIDWrappingMaxInput(2 * Math.PI);

    }

    private void resetSteeringEncoderToAbsolute(){
        // Reset the steering encoder to the absolute position
        steeringEncoder.setPosition(Rotation2d.fromRotations(steerAbsoluteEncoder.getAbsolutePosition().getValue()).getRadians());
        
    }
    /*
     * public void zeroSwerveAngle() {
            if(!hasGoodCANCoderSeedingOccurred){
                mSteeringSensor.configSensorInitializationStrategy(
                SensorInitializationStrategy.BootToAbsolutePosition,Constants.kDefaultTimeout);
                if(mSteeringSensor.getLastError()==ErrorCode.OK){
                    hasGoodCANCoderSeedingOccurred = true;
                } else {
                    System.out.println("Couldn't zero swerve module");
                }
            } else if(!hasSwerveZeroingOccurred){
                mSteeringMotor.setSelectedSensorPosition(
                mSteeringSensor.getPosition(),0,Constants.kDefaultTimeout);
                if(mSteeringMotor.getLastError()==ErrorCode.OK){
                    hasSwerveZeroingOccurred = true;
                }
            }
        }
     */

    public SwerveModuleState getState() {//revisar
        currentState = new SwerveModuleState(drivingEncoder.getVelocity(), Rotation2d.fromRadians(steeringEncoder.getPosition()).minus(chassisAngularOffset));
        return currentState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivingEncoder.getPosition(), Rotation2d.fromRadians(steeringEncoder.getPosition()).minus(chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        // m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        // m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    
        targetState = desiredState;
        // get the current state of the module
    }
        /*
         * if (motor.getSelectedSensorVelocity() * motorEncoderVelocityCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
                if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                    resetIteration = 0;
                    double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
                    motor.setSelectedSensorPosition(absoluteAngle / motorEncoderPositionCoefficient);
                    currentAngleRadians = absoluteAngle;
                }
            } else {
                resetIteration = 0;
            }

            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }
         */

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public void updateModule() {
        // Update the module

    }
}
