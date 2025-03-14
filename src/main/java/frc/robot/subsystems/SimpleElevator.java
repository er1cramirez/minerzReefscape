package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.SimpleElevatorConstants;

public class SimpleElevator extends SubsystemBase {
    // Hardware
    private final VictorSPX motor;

    public SimpleElevator() {
        // Initialize motor
        motor = new VictorSPX(CoralConstants.ARM_MOTOR_ID);
        configureMotor();
        SmartDashboard.putData("SimpleElevator", this);
    }

    private void configureMotor() {
        motor.configFactoryDefault();
        motor.setInverted(false);
        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);
        motor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
    }

    public void setSpeed(double speed) {
        // Fisicamente no es factible
        // if (speed > SimpleElevatorConstants.INPUT_THRESHOLD) {
        //     if(limitSwitch.get())
        //         motor.set(ControlMode.PercentOutput, speed);
        //     else
        //         hold();
        // } else if (Math.abs(speed) > SimpleElevatorConstants.INPUT_THRESHOLD) {
        //     motor.set(ControlMode.PercentOutput, speed);
        // } else {
        //     hold();
        // }

        // Si los triggers no están siendo presionados (con cierto umbral) se mantiene la posición
        if (Math.abs(speed) < SimpleElevatorConstants.INPUT_THRESHOLD)
            hold();
        else 
            motor.set(ControlMode.PercentOutput, speed);
    }

    // Aplica un poco de potencia para mantener la posición
    private void hold() {
        motor.set(ControlMode.PercentOutput, SimpleElevatorConstants.HOLD_POWER);
    }

    public void stop() {
        hold();
    }

    public void totalStop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        // Puede añadir telemetría aquí si lo desea
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.setSmartDashboardType("SimpleElevator");
        // builder.addDoubleProperty("Motor Output", () -> motor.getMotorOutputPercent(), null);
        // builder.addBooleanProperty("Is Holding", () -> Math.abs(motor.getMotorOutputPercent() - HOLD_POWER) < 0.01, null);
    }
}