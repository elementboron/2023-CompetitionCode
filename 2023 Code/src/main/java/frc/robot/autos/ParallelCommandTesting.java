package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmToHigh;
import frc.robot.commands.ArmToLow;
import frc.robot.commands.PrintLineCommand;
import frc.robot.commands.WheelsSpitOut;
import frc.robot.commands.WheelsStop;
import frc.robot.commands.WristToHigh;
import frc.robot.subsystems.GripperWheels;
import frc.robot.subsystems.RotateArmMotor;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristMotor;

public class ParallelCommandTesting extends SequentialCommandGroup {

    
    public ParallelCommandTesting(Swerve s_Swerve, RotateArmMotor s_Arm, WristMotor s_Wrist, GripperWheels s_Wheels){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    (Constants.AutoConstants.kMaxSpeedMetersPerSecond/2),
                    (Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared/1.5))
                .setKinematics(Constants.Swerve.swerveKinematics);

        var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

        //PathPlannerTrajectory traj = PathPlanner.loadPath("DriveStraighPath", new PathConstraints(3, 1));
        Trajectory traj =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(5.5, 0, new Rotation2d(0)),
                config);
        Trajectory traj2 =
                TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, new Rotation2d(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(1, 0)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(2, 0, new Rotation2d(0)),
                    config);

        SwerveControllerCommand DriveCommand =
            new SwerveControllerCommand(
                traj,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        
        SwerveControllerCommand OntoDock =
                new SwerveControllerCommand(
                    traj2,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);

        addCommands(
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            new InstantCommand((() -> s_Swerve.resetOdometry(traj.getInitialPose()))),
            new ArmToLow(s_Arm),
                new ParallelCommandGroup(
                    new ArmToHigh(s_Arm),
                    new WristToHigh(s_Wrist)
                ),
            new PrintLineCommand(s_Arm, "Wait Command"),
            new WaitCommand(0.5),
            new PrintLineCommand(s_Arm, "Wheels Spit Out"),
            new WheelsSpitOut(s_Wheels),
            new WaitCommand(0.3),
            new WheelsStop(s_Wheels)
        );

    }
}