package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class SimpleElevator extends SubsystemBase {
    // Hardware
    private final VictorSPX motor;
    
    // Estado para el modo de retención
    private boolean holdMode = false;
    private static final double HOLD_POWER = 0.05;

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

    private boolean isSafeToMove(double speed) {
        return true;
    }

    public void setSpeed(double speed) {
        // Si estamos en modo hold, ignoramos los comandos de velocidad
        if (holdMode) {
            return;
        }
        
        if (isSafeToMove(speed)) {
            motor.set(ControlMode.PercentOutput, speed);
        } else {
            stop();
        }
    }

    public void stop() {
        if (!holdMode) {
            motor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void hold() {
        holdMode = true;
        motor.set(ControlMode.PercentOutput, HOLD_POWER);
        System.out.println("Elevator hold activated");
    }
    
    public void releaseHold() {
        holdMode = false;
        stop();
        System.out.println("Elevator hold released");
    }

    public boolean isHolding() {
        return holdMode;
    }

    @Override
    public void periodic() {
        // Puede añadir telemetría aquí si desea monitorear el estado
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.setSmartDashboardType("SimpleElevator");
        // builder.addBooleanProperty("Hold Mode", this::isHolding, null);
        // builder.addDoubleProperty("Motor Output", () -> motor.getMotorOutputPercent(), null);
    }
}