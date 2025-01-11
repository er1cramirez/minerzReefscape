package frc.robot.telemetry;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotTelemetry implements SubsystemTelemetry{
    // Autonomous selector
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    public RobotTelemetry() {
        // Initialize any default values or configurations
    }

    @Override
    public void initTelemetry(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotTelemetry");
        // Robot State
        builder.addDoubleProperty("Match Time", Timer::getMatchTime, null);
        builder.addDoubleProperty("Battery Voltage", RobotController::getBatteryVoltage, null);
        builder.addBooleanProperty("Brownout", RobotController::isBrownedOut, null);
        
        // CPU and Memory Stats
        builder.addDoubleProperty("CPU Usage", () -> RobotController.getCPUTemp(), null);
        builder.addDoubleProperty("Can Utilization", () -> RobotController.getCANStatus().percentBusUtilization, null);
    }

    @Override
    public void updateTelemetry() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateTelemetry'");
    }

    /**
     * Add an autonomous command to the selector
     */
    public void addAutoCommand(String name, Command command) {
        if (name.equals("Default")) {
            autoChooser.setDefaultOption(name, command);
        } else {
            autoChooser.addOption(name, command);
        }
    }

    /**
     * Get the selected autonomous command
     */
    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
    /**
     * Call this in robotInit() to set up Shuffleboard/SmartDashboard entries
     */
    public void initDashboard() {
        // Put items that need their own widgets
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    
}
