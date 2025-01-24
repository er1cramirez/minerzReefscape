package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeGrabber extends SubsystemBase {
    // Hardware
    private final VictorSPX grabberMotor;
    
    // Constants
    private static final double GRAB_SPEED = 1;
    private static final double RELEASE_SPEED = -1;
    
    public AlgaeGrabber() {
        // Initialize hardware
        grabberMotor = new VictorSPX(AlgaeConstants.GRABBER_MOTOR_ID);
        configureMotor();
    }
    
    private void configureMotor() {
        // Configure motor settings
        grabberMotor.configFactoryDefault();
        grabberMotor.setInverted(false);
        grabberMotor.configVoltageCompSaturation(12.0);
        grabberMotor.enableVoltageCompensation(true);
        grabberMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    public void grab() {
        grabberMotor.set(ControlMode.PercentOutput, GRAB_SPEED);
    }
    
    public void release() {
        grabberMotor.set(ControlMode.PercentOutput, RELEASE_SPEED);
    }
    
    public void stop() {
        grabberMotor.set(ControlMode.PercentOutput, 0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CoralGrabber");
        builder.addDoubleProperty("Motor Output", grabberMotor::getMotorOutputPercent, null);
    }
}
