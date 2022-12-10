package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class EnableElevator extends CommandBase{
    private final Elevator elevator;

    private final double setpoint;
    private final BooleanSupplier toggleWinch;

    public EnableElevator(Elevator elevator, BooleanSupplier toggleWinch) {
        this.elevator = elevator;
        this.toggleWinch = toggleWinch;

        setpoint = Constants.Elavator.releaseWinchSetpoint;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setWinchSetpoint(0);
        elevator.setElevatorSetpoint(0);
        elevator.zeroWinchEncoder();
    }

    @Override
    public void execute() {
        if (toggleWinch.getAsBoolean()) {
            elevator.setWinchSetpoint(setpoint);
        } else {
            elevator.setWinchSetpoint(0);
        }
        elevator.toWinchSetpoint();
        elevator.toElevatorSetpoint();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setWinch(0);
        elevator.setElevator(0);
    }
}