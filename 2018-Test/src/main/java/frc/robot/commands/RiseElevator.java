package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class RiseElevator extends CommandBase{
    private final Elevator elevator;

    private final double setpoint;

    public RiseElevator(Elevator elevator) {
        this.elevator = elevator;
        setpoint = Constants.Elavator.upper;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setElevatorSetpoint(setpoint);
        elevator.zeroElevatorEncoder();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevator(0);
    }
}