package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
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

    private final Field2d m_field = new Field2d();


    public RobotContainer() {
        m_swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                m_swerveSubsystem,
                m_Joystick::get_LStickX,
                m_Joystick::get_LStickY,
                m_Joystick::get_RStickX,
                () -> !m_Joystick.btn_A.getAsBoolean()));
        elevator.setDefaultCommand(new ElevatorWithJoystick(
                elevator,
                m_Joystick.POV_North::getAsBoolean,
                m_Joystick.POV_South::getAsBoolean));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
      m_Joystick.btn_X.whenPressed(m_swerveSubsystem::zeroHeading);
      m_Joystick.btn_topR.whileHeld(() -> new RunGripper(gripper, false));
      m_Joystick.btn_triggerR.whileHeld(() -> new RunGripper(gripper, true));
      m_Joystick.btn_Y.whenPressed(() -> new RunGripperToPosition(gripper, 0));
      m_Joystick.btn_topL.whenPressed(() -> new RunGripperToPosition(gripper, .125));
      m_Joystick.btn_triggerL.whenPressed(() -> new RunGripperToPosition(gripper, .361));
      m_Joystick.btn_A.whenPressed(() -> new ReleaseTexi(paneumatics));
      m_Joystick.btn_B.whenPressed(() -> new ReleaseClimber(paneumatics));
    }


    Path trajPath = Filesystem.getDeployDirectory().toPath().resolve("paths/Slalom.wpilib.json");
    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);


        // 2. Generate trajectory
        try {
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajPath);
//        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//                new Pose2d(0, 0, new Rotation2d(0)),
//                List.of(
//                        new Translation2d(1, 0),
//                        new Translation2d(1, -1)),
//                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
//                trajectoryConfig);

            // 3. Define PID controllers for tracking trajectory
            m_field.getObject("traj").setTrajectory(trajectory);
            SmartDashboard.putData(m_field);
            PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
            PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
            ProfiledPIDController thetaController = new ProfiledPIDController(
                    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

            // 4. Construct command to follow trajectory
            trajectory.sample(3.5);
            SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                    trajectory,
                    m_swerveSubsystem::getPose,
                    DriveConstants.kDriveKinematics,
                    xController,
                    yController,
                    thetaController,
                    m_swerveSubsystem::setModuleStates,
                    m_swerveSubsystem);

            // 5. Add some init and wrap-up, and return everything
            return new SequentialCommandGroup(
                    new InstantCommand(() -> m_swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                    swerveControllerCommand,
                    new InstantCommand(() -> m_swerveSubsystem.stopModules()));
        } catch (IOException e) {
            return new InstantCommand(() -> m_swerveSubsystem.stopModules());
        }
    }
}