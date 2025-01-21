package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGrabber;
// import frc.robot.subsystems.CoralGrabber;

public class TeleopAlgaeGrabber extends Command {
    private final AlgaeGrabber grabber;
    private final boolean isGrabbing;
    
    /**
     * Creates a new TeleopGrabber command.
     *
     * @param grabber The grabber subsystem
     * @param isGrabbing True for grab, false for release
     */
    public TeleopAlgaeGrabber(AlgaeGrabber grabber, boolean isGrabbing) {
        this.grabber = grabber;
        this.isGrabbing = isGrabbing;
        addRequirements(grabber);
    }
    
    @Override
    public void initialize() {
        if (isGrabbing) {
            grabber.grab();
        } else {
            grabber.release();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        grabber.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;  // Run until interrupted
    }
}