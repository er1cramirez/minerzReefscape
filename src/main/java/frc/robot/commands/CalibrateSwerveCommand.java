package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;

public class CalibrateSwerveCommand extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private boolean isFinished = false;
    
    public CalibrateSwerveCommand(SwerveDriveSubsystem swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }
    
    @Override
    public void initialize() {
        isFinished = false;
    }
    
    @Override
    public void execute() {
        isFinished = swerveDrive.calibrateModules();
    }
    
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}