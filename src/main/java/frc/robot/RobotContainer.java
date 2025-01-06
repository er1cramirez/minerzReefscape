
package frc.robot;

import frc.robot.commands.CalibrateSwerveCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain.DriveMode;
import frc.robot.subsystems.Drivetrain.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // Subsystems
    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    
    // Controllers
    private final XboxController driverController = new XboxController(Constants.ControllersConstants.chassisControllerPort);
    
    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }
    
    private void configureDefaultCommands() {
        swerveDriveSubsystem.setDefaultCommand(
            new TeleopDrive(
                swerveDriveSubsystem,
                () -> -driverController.getLeftY(),  // Forward/backward
                () -> -driverController.getLeftX(),  // Left/right
                () -> -driverController.getRightX()  // Rotation
            )
        );
    }
    
      /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureButtonBindings() {
        // Calibration command
        new JoystickButton(driverController, Button.kY.value)
            .onTrue(new CalibrateSwerveCommand(swerveDriveSubsystem));
            
        // Drive modes
        new JoystickButton(driverController, Button.kLeftBumper.value)
            .whileTrue(new InstantCommand(
                () -> swerveDriveSubsystem.setDriveMode(DriveMode.PRECISION)))
            .onFalse(new InstantCommand(
                () -> swerveDriveSubsystem.setDriveMode(DriveMode.NORMAL)));
                
        new JoystickButton(driverController, Button.kRightBumper.value)
            .whileTrue(new InstantCommand(
                () -> swerveDriveSubsystem.setDriveMode(DriveMode.TURBO)))
            .onFalse(new InstantCommand(
                () -> swerveDriveSubsystem.setDriveMode(DriveMode.NORMAL)));
        
        // X-Lock mode
        new JoystickButton(driverController, Button.kX.value)
            .onTrue(new InstantCommand(
                () -> swerveDriveSubsystem.setDriveMode(DriveMode.X_LOCK)))
            .onFalse(new InstantCommand(
                () -> swerveDriveSubsystem.setDriveMode(DriveMode.NORMAL)));
        
        // Field orientation toggle
        new JoystickButton(driverController, Button.kB.value)
            .onTrue(new InstantCommand(
                () -> swerveDriveSubsystem.setFieldOriented(
                    !swerveDriveSubsystem.isFieldOriented()
                )));
    }
    
    public Command getAutonomousCommand() {
        // Return the autonomous command
        return null; // Replace with actual autonomous command
    }
}
  
