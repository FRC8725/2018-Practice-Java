package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;


public class RunElevatorToPosition extends CommandBase {

    Elevator elevator;
    double position;

    public RunElevatorToPosition(Elevator elevator, double position) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.position = position;
    }

    @Override
    public void initialize() {
        elevator.setElevatorSetpoint(position);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return elevator.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator();
    }
}
