package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.IOException;
import java.nio.file.Path;

public class Slalom extends SequentialCommandGroup {

    Field2d m_field = new Field2d();
    Path trajPath = Filesystem.getDeployDirectory().toPath().resolve("paths/Slalom.wpilib.json");

    public Slalom(SwerveSubsystem m_swerveSubsystem){
        // 2. Generate trajectory
        try {
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajPath);

            // 3. Define PID controllers for tracking trajectory
            m_field.getObject("traj").setTrajectory(trajectory);
            SmartDashboard.putData(m_field);
            PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
            PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
            ProfiledPIDController thetaController = new ProfiledPIDController(
                    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

            // 4. Construct command to follow trajectory
            trajectory.sample(4);
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
            addCommands(
                    new InstantCommand(() -> m_swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                    swerveControllerCommand,
                    new InstantCommand(() -> m_swerveSubsystem.stopModules()));
        } catch (IOException e) {
            System.out.print("Unable to open trajectory");
            addCommands(new InstantCommand(() -> m_swerveSubsystem.stopModules()));
        }
    }
}
