package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ElevatorStates;

public class Elevator extends SubsystemBase {
    // Hardware
    private final SparkMax mainMotor;
    // private final SparkMax followerMotor;
    private final RelativeEncoder mainEncoder;
    // private final RelativeEncoder followerEncoder;

    // Limit switchs
    private final DigitalInput  bottomLimitSwitch;
    private final DigitalInput  topLimitSwitch;

    // Controllers
    private final SparkClosedLoopController mainMotorController;
    // private final SparkClosedLoopController followerMotorController;


    public Elevator() {
        // Initialize hardware
        mainMotor = new SparkMax(Constants.ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        // followerMotor = new SparkMax(Constants.ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
        mainEncoder = mainMotor.getEncoder();
        // rightEncoder = rightMotor.getEncoder();
        // Initialize controllers
        mainMotorController = mainMotor.getClosedLoopController();
        // rightController = rightMotor.getClosedLoopController();
        motorConfig();
        // Initialize limit switches
        bottomLimitSwitch = new DigitalInput(Constants.ElevatorConstants.BOTTOM_LIMIT_SWITCH_PORT);
        topLimitSwitch = new DigitalInput(Constants.ElevatorConstants.TOP_LIMIT_SWITCH_PORT);
    }

    private void motorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(Constants.ElevatorConstants.MOTOR_INVERTED);
        config.smartCurrentLimit(Constants.ElevatorConstants.CURRENT_LIMIT);
        config.voltageCompensation(Constants.MotorConstants.voltageCompensation);
        config.idleMode(Constants.ElevatorConstants.IDLE_MODE);
        double factor = Constants.ElevatorConstants.POS_FACTOR;
        config.encoder.positionConversionFactor(factor);
        config.encoder.velocityConversionFactor(factor / 60.0);
        // drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.p(Constants.ElevatorConstants.elevatorPID.kP());
        config.closedLoop.i(Constants.ElevatorConstants.elevatorPID.kI());
        config.closedLoop.d(Constants.ElevatorConstants.elevatorPID.kD());
        config.closedLoop.minOutput(Constants.ElevatorConstants.elevatorPID.kMinOutput());
        config.closedLoop.maxOutput(Constants.ElevatorConstants.elevatorPID.kMaxOutput());
        mainMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // encoder reset
        mainEncoder.setPosition(0);
        // Configure the motors
    }

    /**
     * Check if it is safe to move the elevator
     * @param speed The speed to move the elevator
     * @return True if it is safe to move the elevator
     */
    private boolean isSafeToMove(double speed) {
        if (bottomLimitSwitch.get() && speed < 0) {
            resetElevatorEncoder(true);
            return false;
        }
        if (topLimitSwitch.get() && speed > 0) {
            resetElevatorEncoder(false);
            return false;
        }
        return true;
    }

    // Public interface methods
    public double getCurrentPosition() {
        return mainEncoder.getPosition();
    }

    public double getCurrentVelocity() {
        return mainEncoder.getVelocity();
    }

    public void setPosition(double position) {
        if (isSafeToMove(mainEncoder.getVelocity())) {
            mainMotorController.setReference(position, ControlType.kPosition);
        }
    }

    public void setSpeed(double speed) {
        if (isSafeToMove(speed)) {
            mainMotor.set(speed);
        }
    }

    public void stopElevator() {
        mainMotor.set(0);
    }
    public void resetElevatorEncoder(boolean bottom) {
        // Reset the elevator encoder when the elevator is at the bottom(limit switch)
        stopElevator();
        if (bottom) {
            mainEncoder.setPosition(0);
        } else {
            mainEncoder.setPosition(ElevatorStates.TOP.getHeight());
        }
    }

    @Override
    public void periodic() {
        // Safety check
        if (!isSafeToMove(mainEncoder.getVelocity())) {
            stopElevator();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This method will be called once per scheduler run
    }
    
}
