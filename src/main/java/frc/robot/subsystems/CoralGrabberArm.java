package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
// import com.ctre.phoenix6.hardware.CANcoder;
// import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralGrabberArm extends SubsystemBase {
    // Hardware
    private final WPI_VictorSPX armMotor;
    // private final CANcoder absoluteEncoder;
    
    // Control objects
    private final SlewRateLimiter rateLimiter;
    // private final ProfiledPIDController pidController;
    
    // Constants
    private static final int ARM_MOTOR_ID = 12;
    // private static final int ENCODER_ID = 12;
    private static final double SLEW_RATE_LIMIT = 2.0; // units per second
    // private static final double MAX_VELOCITY = 2.0;
    // private static final double MAX_ACCELERATION = 4.0;
    // private static final double kP = 1.0;
    // private static final double kI = 0.0;
    // private static final double kD = 0.1;
    
    // State tracking
    private boolean isPositionControl = false;
    
    public CoralGrabberArm() {
        // Initialize hardware
        armMotor = new WPI_VictorSPX(ARM_MOTOR_ID);
        // absoluteEncoder = new CANcoder(ENCODER_ID);
        
        // Initialize controllers
        rateLimiter = new SlewRateLimiter(SLEW_RATE_LIMIT);
        // pidController = new ProfiledPIDController(
        //     kP, kI, kD,
        //     new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION)
        // );
        
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
        if (!isPositionControl) {
            double filteredSpeed = rateLimiter.calculate(speed);
            armMotor.set(ControlMode.PercentOutput, filteredSpeed);
        }
    }
    
    // // Position control methods
    // public void setPosition(double targetPosition) {
    //     isPositionControl = true;
    //     double output = pidController.calculate(getArmPosition(), targetPosition);
    //     armMotor.set(ControlMode.PercentOutput, output);
    // }
    
    // public void disablePositionControl() {
    //     isPositionControl = false;
    //     pidController.reset(getArmPosition());
    // }
    
    // public double getArmPosition() {
    //     return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    // }
    
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
        // builder.addDoubleProperty("Arm Position", this::getArmPosition, null);
        builder.addDoubleProperty("Motor Output", armMotor::getMotorOutputPercent, null);
        builder.addBooleanProperty("Position Control", () -> isPositionControl, null);
    }
}
