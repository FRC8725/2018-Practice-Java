package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gripper;


public class RunGripper extends CommandBase {

    private final Gripper gripper;
    private final boolean invert;
    private double speed;

    public RunGripper(Gripper gripper, boolean invert) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(gripper);
        this.gripper = gripper;
        this.invert = invert;
    }

    @Override
    public void initialize() {
        if (invert) {
            speed = -Constants.Gripper.spinnerSpeed;
        } else {
            speed = Constants.Gripper.spinnerSpeed;
        }
    }

    @Override
    public void execute() {
        gripper.setSpinner(speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        gripper.setSpinner(0);
    }
}
