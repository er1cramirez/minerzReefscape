package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.SimpleElevatorConstants;

public class CoralGrabberArm extends SubsystemBase {
    // Hardware
    private final SparkMax armMotor;
    // private final VictorSPX armMotor;
    
    // Constants
    
    
    public CoralGrabberArm() {
        // Initialize hardware
        // armMotor = new VictorSPX(CoralConstants.ARM_MOTOR_ID);
        armMotor = new SparkMax(SimpleElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        
        configureMotor();
    }
    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(SimpleElevatorConstants.CURRENT_LIMIT);
        config.voltageCompensation(12.0);
        config.idleMode(IdleMode.kBrake);
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    // private void configureMotor() {
    //     armMotor.configFactoryDefault();
    //     armMotor.setInverted(false);
    //     armMotor.configVoltageCompSaturation(12.0);
    //     armMotor.enableVoltageCompensation(true);
    // }
    
    // Simple joystick control methods
    public void setSpeed(double speed) {
        // armMotor.set(ControlMode.PercentOutput, speed);
        armMotor.set(speed);
    }
    
    public void stop() {
        // armMotor.set(ControlMode.PercentOutput, 0);
        armMotor.set(0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.setSmartDashboardType("CoralGrabberArm");
        // builder.addDoubleProperty("Motor Output", armMotor::getMotorOutputPercent, null);
    }
}
