// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class ControllersConstants {
    public static final int chassisControllerPort = 0;
    public static final int operatorControllerPort = 1;
    public static final double chassisControllerDeadband = 0.05;
    public static final double operatorControllerDeadband = 0.05;
  }

  public static class SwerveDriveConstants {
    // Chassis dimensions
    public static final double chassisWidth = Units.inchesToMeters(20);//inch
    public static final double chassisLength = Units.inchesToMeters(20);//inch
    // Kinematics
    public static final double kMaxSpeed = 3.0;//m/s
    public static final double kMaxAngularSpeed = 2;//rad/s
    public static final double kTranslationalSlew = 3.0;//m/s^2
    public static final double kRotationalSlew = Math.PI/2;//rad/s^2
    public static final double kPrecisionModeSpeedMultiplier = 0.5;
    public static final double kTurboModeSpeedMultiplier = 1.5;
    public static final double kXLockModeSpeedMultiplier = 0.0;
    public static final double kSteeringErrorTolerance = 0.1; // rad
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

    // Other types of modules can be added here


    // PID constants
    public static final PIDConstants drivePIDConstants = new PIDConstants(
      0.4, 0, 0.01, 0, 0, 1, -1);
    public static final PIDConstants steerPIDConstants = new PIDConstants(
      1, 0, 0.4, 0, 0, 1, -1);
    // Module constants
    public static final Mk4iSwerveModuleConstants frontLeftModuleConstants = new Mk4iSwerveModuleConstants(
      1, 1, 2, 16, frontLeftSteerOffset, 
      false, true, false, 
      drivePIDConstants, steerPIDConstants);
    public static final Mk4iSwerveModuleConstants frontRightModuleConstants = new Mk4iSwerveModuleConstants(
      2, 5, 6, 19, frontRightSteerOffset, 
      false, true, false, 
      drivePIDConstants, steerPIDConstants);
    public static final Mk4iSwerveModuleConstants backLeftModuleConstants = new Mk4iSwerveModuleConstants(
      3, 3, 4, 17, backLeftSteerOffset, 
      false, true, false, 
      drivePIDConstants, steerPIDConstants);
    public static final Mk4iSwerveModuleConstants backRightModuleConstants = new Mk4iSwerveModuleConstants(
      4, 7, 8, 18, backRightSteerOffset, 
      false, true, false, 
      drivePIDConstants, steerPIDConstants);

    
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI/2;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
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

  // Telemetry constants
  public static final class TelemetryConstants {
    public static final double kTelemetryTransmissionInterval = 0.1;
    public static final class SwerveTopicNames {
      // NetworkTables topics for telemetry
      public static final String CURRENT_STATES_TOPIC = "/SwerveStates/Current";
      public static final String TARGET_STATES_TOPIC = "/SwerveStates/Target";
    }
    
  }

}
