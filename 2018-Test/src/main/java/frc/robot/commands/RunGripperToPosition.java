package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class RunGripperToPosition extends CommandBase {

    private final Gripper gripper;
    private final double position;

    public RunGripperToPosition(Gripper gripper, double position) {
        addRequirements(gripper);
        this.gripper = gripper;
        this.position = position;
    }

    @Override
    public void initialize() {
        gripper.setAngleSetpoint(position);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return gripper.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        gripper.setAngleMotor(0);
    }
}
