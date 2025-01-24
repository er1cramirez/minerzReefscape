package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    // Hardware
    private final SparkMax masterMotor;
    private final SparkMax followerMotor;
    
    // Constants
    private static final int MASTER_MOTOR_ID = 15;
    private static final int FOLLOWER_MOTOR_ID = 16;
    private static final int CURRENT_LIMIT = 40;

    
    
    public Climber() {
        masterMotor = new SparkMax(MASTER_MOTOR_ID, MotorType.kBrushless);
        followerMotor = new SparkMax(FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        configureMotors();
    }
    
    private void configureMotors() {
        // Master config
        SparkMaxConfig masterConfig = new SparkMaxConfig();
        masterConfig.inverted(false);
        masterConfig.smartCurrentLimit(CURRENT_LIMIT);
        masterConfig.voltageCompensation(12.0);
        masterConfig.idleMode(IdleMode.kBrake);
        masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Follower config
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.inverted(true); // Opposite direction
        followerConfig.smartCurrentLimit(CURRENT_LIMIT);
        followerConfig.voltageCompensation(12.0);
        followerConfig.idleMode(IdleMode.kBrake);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void setSpeed(double speed) {
        masterMotor.set(speed);
        followerMotor.set(speed);
    }
    
    public void stop() {
        masterMotor.set(0);
        followerMotor.set(0);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Climber");
        builder.addDoubleProperty("Master Output", masterMotor::get, null);
        builder.addDoubleProperty("Follower Output", followerMotor::get, null);
    }
}
