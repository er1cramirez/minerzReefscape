package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralGrabberArm extends SubsystemBase {
    // Hardware
    private final VictorSPX armMotor;
    
    // Constants
    
    
    public CoralGrabberArm() {
        // Initialize hardware
        armMotor = new VictorSPX(CoralConstants.ARM_MOTOR_ID);
        configureMotor();
    }
    
    private void configureMotor() {
        armMotor.configFactoryDefault();
        armMotor.setInverted(false);
        armMotor.configVoltageCompSaturation(12.0);
        armMotor.enableVoltageCompensation(true);
    }
    
    // Simple joystick control methods
    public void setSpeed(double speed) {
        armMotor.set(ControlMode.PercentOutput, speed);
    }
    
    public void stop() {
        armMotor.set(ControlMode.PercentOutput, 0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CoralGrabberArm");
        builder.addDoubleProperty("Motor Output", armMotor::getMotorOutputPercent, null);
    }
}
