package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class SimpleElevator extends SubsystemBase {
    // Hardware
    // private final SparkMax motor;
    private final VictorSPX motor;

    // Constants

    public SimpleElevator() {
        // Initialize motor
        motor = new VictorSPX(CoralConstants.ARM_MOTOR_ID);
        configureMotor();
    }

    private void configureMotor() {
        motor.configFactoryDefault();
        motor.setInverted(false);
        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);
        motor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
    }


    private boolean isSafeToMove(double speed) {
        return true;
    }

    public void setSpeed(double speed) {
        if (isSafeToMove(speed)) {
            motor.set(ControlMode.PercentOutput, speed);
        } else {
            stop();
        }
    }

    public void stop() {
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