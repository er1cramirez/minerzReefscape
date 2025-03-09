package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.SimpleElevatorConstants;

public class SimpleElevator extends SubsystemBase {
    // Hardware
    private final VictorSPX motor;
    // We'll only use the top limit switch
    private final DigitalInput topLimitSwitch;
    private final Debouncer topDebouncer;

    public SimpleElevator() {
        // Initialize motor
        motor = new VictorSPX(CoralConstants.ARM_MOTOR_ID);
        configureMotor();

        // Initialize only the top limit switch
        topLimitSwitch = new DigitalInput(1);
        
        // Initialize debouncer for the top limit switch
        topDebouncer = new Debouncer(SimpleElevatorConstants.DEBOUNCE_TIME, DebounceType.kBoth);
        SmartDashboard.putData("SimpleEelevator", this);
    }

    private void configureMotor() {
        motor.configFactoryDefault();
        motor.setInverted(false);
        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);
        // Set brake mode
        motor.setNeutralMode(NeutralMode.Brake);
    }

    private boolean isSafeToMove(double speed) {
        // Get debounced limit switch reading for top switch
        boolean topTriggered = isAtTop();

        // Only prevent upward movement when at top
        if (topTriggered && speed > 0) {
            return false;
        }
        return true;
    }

    /**
     * Checks if the elevator is at the top position
     * @return true if top limit switch is triggered
     */
    public boolean isAtTop() {
        return topDebouncer.calculate(!topLimitSwitch.get());
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
        builder.setSmartDashboardType("SimpleElevator");
        builder.addBooleanProperty("At Top Position", this::isAtTop, null);
    }
}