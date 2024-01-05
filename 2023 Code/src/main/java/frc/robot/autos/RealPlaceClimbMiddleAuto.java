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
import frc.robot.commands.ArmToHighAuto;
import frc.robot.commands.ArmToHome;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DeactivatePneumatics;
import frc.robot.commands.WheelsStop;
import frc.robot.commands.WheelsSuckIn;
import frc.robot.commands.WristToDown;
import frc.robot.commands.WristToHigh;
import frc.robot.commands.WristToHome;
import frc.robot.subsystems.GripperWheels;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.RotateArmMotor;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristMotor;

public class RealPlaceClimbMiddleAuto extends SequentialCommandGroup {
    
    public RealPlaceClimbMiddleAuto(Swerve s_Swerve, RotateArmMotor s_Arm, WristMotor s_Wrist, GripperWheels s_Wheels, Pneumatics s_Pneumatics){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    (Constants.AutoConstants.kMaxSpeedMetersPerSecond/4),
                    (Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared/2))
                .setKinematics(Constants.Swerve.swerveKinematics);

        var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

        //PathPlannerTrajectory traj = PathPlanner.loadPath("DriveStraighPath", new PathConstraints(3, 1));
        Trajectory ontoDock =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-1, 0.1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-2.4, 0.3, new Rotation2d(0)),
                config.setReversed(true));

        SwerveControllerCommand DriveOntoDock =
            new SwerveControllerCommand(
                ontoDock,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        

        addCommands(
            new InstantCommand((() -> s_Swerve.resetOdometry(ontoDock.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            
            new DeactivatePneumatics(s_Pneumatics),
            
            //PLACE INITIAL CONE
                new ParallelCommandGroup(
                    new ArmToHighAuto(s_Arm),
                    new WristToHigh(s_Wrist)
                ),
            new WaitCommand(0.3),
            new WheelsSuckIn(s_Wheels),
            new WaitCommand(0.4),
            new WheelsStop(s_Wheels),
          
            //RETRACT ARM AND WRIST AND DRIVE TO CONE
            new WristToDown(s_Wrist),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new WaitCommand(0.3),
                            new ArmToHome(s_Arm)
                        ),
                    new WristToHome(s_Wrist)
                ),
            new InstantCommand((() -> s_Swerve.zeroGyro())),

            //DRIVE ONTO DOCK
            DriveOntoDock,
            new AutoBalance(s_Swerve),
            new InstantCommand((() -> s_Swerve.OneEightyGyro()))

        );

    }
}