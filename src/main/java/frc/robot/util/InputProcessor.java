package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class InputProcessor {
    /**
     * Processes a raw input value by applying deadband, slew rate limiting, and scaling.
     *
     * @param input Raw input value (-1 to 1)
     * @param deadband Deadband value to apply
     * @param limiter SlewRateLimiter for smooth acceleration
     * @param maxSpeed Maximum speed to scale to
     * @return Processed input value
     */
    public static double processInput(double input, double deadband, SlewRateLimiter limiter, double maxSpeed) {
        return limiter.calculate(MathUtil.applyDeadband(input, deadband)) * maxSpeed;
    }

    /**
     * Overloaded method for cases without slew rate limiting
     */
    public static double processInput(double input, double deadband, double maxSpeed) {
        return MathUtil.applyDeadband(input, deadband) * maxSpeed;
    }
}