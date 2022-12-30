package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Arrays;


public class GoStraight extends SequentialCommandGroup {

    public GoStraight(SwerveSubsystem m_swerveSubsystem) {

        Field2d m_field = new Field2d();

        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                Arrays.asList(new Pose2d(), new Pose2d(2, 0, new Rotation2d())),
                trajectoryConfig
        );

        m_field.getObject("trej").setTrajectory(trajectory);
        SmartDashboard.putData(m_field);
        System.out.println(trajectory.getTotalTimeSeconds());
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                m_swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                m_swerveSubsystem::setModuleStates,
                m_swerveSubsystem);
        addCommands(
                new InstantCommand(()->m_swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> m_swerveSubsystem.stopModules())
        );
    }
}
