package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
/**
 * Constants for configuring a Swerve Module.
 *
 * @param drivingMotorID The CAN ID of the driving motor
 * @param steeringMotorID The CAN ID of the steering motor
 * @param steerAbsoluteEncoderID The CAN ID of the absolute encoder for the steering motor
 * @param steerAngleOffset The offset for the steer angle encoder
 * @param isDriveMotorInverted Whether the driving motor is inverted
 * @param isSteerMotorInverted Whether the steering motor is inverted
 * @param isSteerAbsoluteEncoderInverted Whether the absolute encoder for the steering motor is inverted
 * @param drivingPIDConstants The PID constants for the driving motor
 * @param steeringPIDConstants The PID constants for the steering motor
 */
public record Mk4iSwerveModuleConstants(
    int drivingMotorID,
    int steeringMotorID,
    int steerAbsoluteEncoderID,
    Rotation2d steerAngleOffset,
    boolean isDriveMotorInverted,
    boolean isSteerMotorInverted,
    boolean isSteerAbsoluteEncoderInverted,
    PIDConstants drivingPIDConstants,
    PIDConstants steeringPIDConstants
) {}
