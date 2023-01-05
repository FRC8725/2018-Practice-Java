package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.GoStraight;
import frc.robot.commands.auto.GoZShape;
import frc.robot.commands.auto.Slalom;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Paneumatics;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
     private final Elevator elevator = Elevator.getInstantce();
     private final Gripper gripper = Gripper.getInstance();
     private final Paneumatics paneumatics = Paneumatics.getInstance();


    private final GamepadJoystick m_Joystick = new GamepadJoystick(0);

    // SmartDashboard
    private final Field2d m_field = new Field2d();
    private final SendableChooser<Command> autoCommand;


    public RobotContainer() {
        m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                m_swerveSubsystem,
                () -> m_Joystick.get_LStickY(),
                () -> -m_Joystick.get_LStickX(),
                () -> -m_Joystick.get_RStickX(),
                () -> !m_Joystick.btn_A.getAsBoolean()));
         elevator.setDefaultCommand(new ElevatorWithJoystick(
                 elevator,
                 m_Joystick.POV_North::getAsBoolean,
                 m_Joystick.POV_South::getAsBoolean));
        configureButtonBindings();
        autoCommand = new SendableChooser<>();
        autoCommand.addOption("Nothing", new InstantCommand(m_swerveSubsystem::stopModules));
        autoCommand.addOption("GoStraight", new GoStraight(m_swerveSubsystem));
        autoCommand.addOption("GoZShape", new GoZShape(m_swerveSubsystem));
        autoCommand.addOption("Slalom", new Slalom(m_swerveSubsystem));
        SmartDashboard.putData(autoCommand);
    }

    private void configureButtonBindings() {
       m_Joystick.btn_X.whenPressed(() -> m_swerveSubsystem.zeroHeading());
       m_Joystick.btn_topR.whileHeld(() -> new RunGripper(gripper, false));
       m_Joystick.btn_triggerR.whileHeld(() -> new RunGripper(gripper, true));
       m_Joystick.btn_Y.whenPressed(() -> new RunGripperToPosition(gripper, 0));
       m_Joystick.btn_topL.whenPressed(() -> new RunGripperToPosition(gripper, .125));
       m_Joystick.btn_triggerL.whenPressed(() -> new RunGripperToPosition(gripper, .361));
       m_Joystick.btn_A.whenPressed(() -> new ReleaseTexi(paneumatics));
    }

    public Command getAutonomousCommand() {
        SmartDashboard.putData(m_field);
        m_swerveSubsystem.resetEncoders();
        return autoCommand.getSelected();
    }
}