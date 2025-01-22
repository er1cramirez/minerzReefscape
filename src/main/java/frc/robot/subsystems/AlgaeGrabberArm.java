package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGrabberArm extends SubsystemBase {
    // Hardware
    private final VictorSPX armMotor;
    // Control objects
    private final SlewRateLimiter rateLimiter;
    
    // Constants
    private static final int ARM_MOTOR_ID = 13;
    private static final double SLEW_RATE_LIMIT = 1.0; // units per second
    
    public AlgaeGrabberArm() {
        // Initialize hardware
        armMotor = new VictorSPX(ARM_MOTOR_ID);
        // Initialize controllers
        rateLimiter = new SlewRateLimiter(SLEW_RATE_LIMIT);
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
        double filteredSpeed = rateLimiter.calculate(speed);
        armMotor.set(ControlMode.PercentOutput, filteredSpeed);
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
