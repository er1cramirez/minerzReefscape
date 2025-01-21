package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralGrabber extends SubsystemBase {
    // Hardware
    private final VictorSPX grabberMotor;
    
    // Constants
    private static final int GRABBER_MOTOR_ID = 11; // Update this ID as needed
    private static final double GRAB_SPEED = 0.5;
    private static final double RELEASE_SPEED = -0.5;
    
    public CoralGrabber() {
        // Initialize hardware
        grabberMotor = new VictorSPX(GRABBER_MOTOR_ID);
        configureMotor();
    }
    
    private void configureMotor() {
        // Configure motor settings
        grabberMotor.configFactoryDefault();
        grabberMotor.setInverted(false);
        grabberMotor.configVoltageCompSaturation(12.0);
        grabberMotor.enableVoltageCompensation(true);
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
