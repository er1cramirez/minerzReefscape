package frc.robot.subsystems;

// import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
// import frc.robot.Constants.SimpleElevatorConstants;

public class SimpleElevator extends SubsystemBase {
    // Hardware
    // private final SparkMax motor;
    private final VictorSPX motor;
    // private final DigitalInput bottomLimitSwitch;
    // private final DigitalInput topLimitSwitch;
    // private final Debouncer bottomDebouncer;
    // private final Debouncer topDebouncer;

    // Constants
    

    public SimpleElevator() {
        // Initialize motor
        motor = new VictorSPX(CoralConstants.ARM_MOTOR_ID);
        // motor = new SparkMax(SimpleElevatorConstants.MOTOR_ID, MotorType.kBrushed);
        configureMotor();

        // Initialize limit switches
        // bottomLimitSwitch = new DigitalInput(SimpleElevatorConstants.BOTTOM_SWITCH_PORT);
        // topLimitSwitch = new DigitalInput(SimpleElevatorConstants.TOP_SWITCH_PORT);
        
        // // Initialize debouncers in both rising and falling edges
        // bottomDebouncer = new Debouncer(SimpleElevatorConstants.DEBOUNCE_TIME, DebounceType.kBoth);
        // topDebouncer = new Debouncer(SimpleElevatorConstants.DEBOUNCE_TIME, DebounceType.kBoth);
    }

    private void configureMotor() {
        motor.configFactoryDefault();
        motor.setInverted(false);
        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);
    }
    // private void configureMotor() {
    //     SparkMaxConfig config = new SparkMaxConfig();
    //     config.inverted(true);
    //     config.smartCurrentLimit(SimpleElevatorConstants.CURRENT_LIMIT);
    //     config.voltageCompensation(12.0);
    //     config.idleMode(IdleMode.kBrake);
    //     motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // }

    private boolean isSafeToMove(double speed) {
        // // Get debounced limit switch readings
        // boolean bottomTriggered = bottomDebouncer.calculate(bottomLimitSwitch.get());
        // boolean topTriggered = topDebouncer.calculate(topLimitSwitch.get());

        // if (bottomTriggered && speed < 0) {
        //     return false;
        // }
        // if (topTriggered && speed > 0) {
        //     return false;
        // }
        return true;
    }

    public void setSpeed(double speed) {
        if (isSafeToMove(speed)) {
            // motor.set(speed);
            motor.set(ControlMode.PercentOutput, speed);
        } else {
            stop();
        }
    }

    public void stop() {
        // motor.set(0);
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.setSmartDashboardType("SimpleElevator");
        // builder.addDoubleProperty("Motor Output", motor::get, null);
        // builder.addBooleanProperty("Bottom Limit", () -> bottomDebouncer.calculate(bottomLimitSwitch.get()), null);
        // builder.addBooleanProperty("Top Limit", () -> topDebouncer.calculate(topLimitSwitch.get()), null);
    }
}