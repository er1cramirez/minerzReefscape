package frc.robot.subsystems.swerve;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimutableSpark extends SparkMax {
    SimDeviceSim sparkSimDevice;
    SimDouble sparkSimAplliedOutput;
    public SimutableSpark(int deviceID, MotorType type) {
        super(deviceID, type);
        sparkSimDevice = new SimDeviceSim("SPARK MAX", deviceID);
        sparkSimAplliedOutput = sparkSimDevice.getDouble("Applied Output");
    }

    public void set(double speed) {
        super.set(speed);
        sparkSimAplliedOutput.set(speed);
    }
    
}
