package frc.robot.util;

/**
 * Constants for configuring a PID controller.
 * 
 * @param kP The proportional gain of the PID controller
 * @param kI The integral gain of the PID controller
 * @param kD The derivative gain of the PID controller
 * @param kIz The maximum error for the integral term to accumulate
 * @param kFF The feedforward gain of the PID controller
 * @param kMaxOutput The maximum output of the PID controller
 * @param kMinOutput The minimum output of the PID controller
 */
public record PIDConstants(
    double kP,
    double kI,
    double kD,
    double kIz,
    double kFF,
    double kMaxOutput,
    double kMinOutput
) {}