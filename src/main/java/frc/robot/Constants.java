package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Mk4iModuleConstants;
import frc.robot.util.PIDConstants;

public class Constants {
    public static class ControllersConstants {
        public static final int chassisControllerPort = 0;
        public static final int operatorControllerPort = 1;
        public static final double chassisControllerDeadband = 0.05;
        public static final double operatorControllerDeadband = 0.05;
    }

    public static class SwerveConstants {
         // Chassis dimensions
        public static final double chassisWidth = Units.inchesToMeters(20);//inch
        public static final double chassisLength = Units.inchesToMeters(20);//inch
        // Kinematics
        public static final double maxSpeed = 3.0;//m/s
        public static final double maxAngularSpeed = 2;//rad/s
        public static final double translationalSlew = 3.0;//m/s^2
        public static final double rotationalSlew = Math.PI/2;//rad/s^2
        public static final double precisionModeSpeedMultiplier = 0.5;
        public static final double turboModeSpeedMultiplier = 1.5;
        public static final double xLockModeSpeedMultiplier = 0.0;
        public static final double steeringErrorTolerance = 0.1; // rad
        public static final double speedDeadband = 0.05; // m/s
        // Module positions
        public static final Translation2d frontLeftModulePosition = new Translation2d(chassisWidth / 2, chassisLength / 2);
        public static final Translation2d frontRightModulePosition = new Translation2d(chassisWidth / 2, -chassisLength / 2);
        public static final Translation2d backLeftModulePosition = new Translation2d(-chassisWidth / 2, chassisLength / 2);
        public static final Translation2d backRightModulePosition = new Translation2d(-chassisWidth / 2, -chassisLength / 2);
        // Kinematics object
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftModulePosition, frontRightModulePosition, backLeftModulePosition, backRightModulePosition);
            
        // Module steer offsets
        public static final Rotation2d frontLeftSteerOffset = new Rotation2d();
        public static final Rotation2d frontRightSteerOffset = new Rotation2d(Math.PI);
        public static final Rotation2d backLeftSteerOffset = new Rotation2d();
        public static final Rotation2d backRightSteerOffset = new Rotation2d(Math.PI);
        
        // NavX
        public static final boolean kGyroReversed = false;


        // Rev Motors Configuration
        /**
         * Configuration for the driving motors on the swerve modules.
         * 
         * @see SparkMaxConfig
         * @see SparkMaxConfig#SparkMaxConfig()
         * 
         */
        public static final class Mk4iMechanicalConstants {
            // L2 ratios for the module Mk4i from Swerve Drive Specialties
            public static final double driveGearRatio = (6.75 / 1.0); 
            public static final double angleGearRatio = ((150/7) / 1.0);
            // Wheel diameter
            public static final double wheelDiameter = Units.inchesToMeters(4);//4 inch
            // Calculations required for driving motor conversion factors and feed forward
            public static final double drivingMotorFreeSpeedRps = MotorConstants.freeSpeedRpm / 60;
            public static final double wheelCircumferenceMeters = wheelDiameter * Math.PI;
            public static final double driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps * wheelCircumferenceMeters)
                / driveGearRatio;
            // Conversion factors
            public static final double drivingFactor = wheelDiameter * Math.PI / driveGearRatio;
            public static final double drivingVelocityFeedForward  = 1 / driveWheelFreeSpeedRps;
            public static final double steeringFactor = (2.0 * Math.PI) / angleGearRatio;
            public static final double steeringVelocityFeedForward = 1 / driveWheelFreeSpeedRps;
        }

        // PID constants
        public static final PIDConstants drivePIDConstants = new PIDConstants(
            0.4, 0, 0.01, 0, 0, 1, -1);
        public static final PIDConstants steerPIDConstants = new PIDConstants(
            1, 0, 0.4, 0, 0, 1, -1);

        // Module constants
        public static final Mk4iModuleConstants frontLeftModuleConstants = new Mk4iModuleConstants(
            1, 1, 2, 16, frontLeftSteerOffset, 
            false, true, false, 
            drivePIDConstants, steerPIDConstants);
        public static final Mk4iModuleConstants frontRightModuleConstants = new Mk4iModuleConstants(
            2, 5, 6, 19, frontRightSteerOffset, 
            false, true, false, 
            drivePIDConstants, steerPIDConstants);
        public static final Mk4iModuleConstants backLeftModuleConstants = new Mk4iModuleConstants(
            3, 3, 4, 17, backLeftSteerOffset, 
            false, true, false, 
            drivePIDConstants, steerPIDConstants);
        public static final Mk4iModuleConstants backRightModuleConstants = new Mk4iModuleConstants(
            4, 7, 8, 18, backRightSteerOffset, 
            false, true, false, 
            drivePIDConstants, steerPIDConstants);
    }
    public static final class MotorConstants {
        public static final double freeSpeedRpm = 5676;
        public static final double voltageCompensation = 12.0;
        // Current limits (Actually for NEO Motors)
        public static final int driveCurrentLimit = 40;
        public static final int steerCurrentLimit = 20;
        // Neutral Modes
        public static final IdleMode driveIdleMode = IdleMode.kBrake;
        public static final IdleMode steerIdleMode = IdleMode.kBrake;
    }

    public static final class ElevatorConstants {
        public static final int LEFT_MOTOR_ID = 0;
        public static final int RIGHT_MOTOR_ID = 1;
        public static final int BOTTOM_LIMIT_SWITCH_PORT = 2;
        public static final int TOP_LIMIT_SWITCH_PORT = 3;
        public static final double MAX_VELOCITY = 0.5;
        public static final double MAX_ACCELERATION = 0.5;
        public static final boolean MOTOR_INVERTED = false;
        public static final int CURRENT_LIMIT = 40;
        public static final IdleMode IDLE_MODE = IdleMode.kBrake;
        public static final double POS_FACTOR = 1.0;
        // PID constants
        public static final PIDConstants elevatorPID = new PIDConstants(
            0.4, 0, 0.01, 0, 0, 1, -1);
    }

    public static final class AutoConstants {
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final double MAX_SPEED = 3;
        public static final double MAX_ACCELERATION = 1;
    }

    
}
