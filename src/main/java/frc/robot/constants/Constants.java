// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveDriveConstants {
    public static final double kChassisWidth = Units.feetToMeters(20);//inch
    public static final double kChassisLength = Units.feetToMeters(20);//inch
    public static final double kMaxSpeed = 3.0;//m/s
    public static final double kMaxAngularSpeed = Math.PI;//rad/s

    public static final double kFrontLeftLocationX = kChassisWidth / 2;
    public static final double kFrontLeftLocationY = kChassisLength / 2;
    public static final double kFrontRightLocationX = kChassisWidth / 2;
    public static final double kFrontRightLocationY = -kChassisLength / 2;
    public static final double kBackLeftLocationX = -kChassisWidth / 2;
    public static final double kBackLeftLocationY = kChassisLength / 2;
    public static final double kBackRightLocationX = -kChassisWidth / 2;
    public static final double kBackRightLocationY = -kChassisLength / 2;

    public static final double kP = 0.04;
    public static final double kI = 0;
    public static final double kD = 0;

    //values to calculate the drive feedforward (KFF)
    public static final double driveKS = 0.667; //to calculate
    public static final double driveKV = 2.44; //to calculate
    public static final double driveKA = 0.27; //to calculate
    
    public static final double kDriveWheelDiameter = Units.inchesToMeters(4);//inch
    public static final double driveGearRatio = (6.75 / 1.0); 
    public static final double angleGearRatio = ((150/7) / 1.0);
  }
   public static final class ModuleConstants {
    public static final double kDriveWheelDiameter = Units.inchesToMeters(4);//inch
    public static final double driveGearRatio = (6.75 / 1.0); 
    public static final double angleGearRatio = ((150/7) / 1.0);
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelCircumferenceMeters = kDriveWheelDiameter * Math.PI;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / driveGearRatio;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
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
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
