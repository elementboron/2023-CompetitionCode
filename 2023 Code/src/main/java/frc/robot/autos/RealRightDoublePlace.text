package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmStop;
import frc.robot.commands.ArmToHigh;
import frc.robot.commands.ArmToHome;
import frc.robot.commands.ArmToLow;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.PrintLineCommand;
import frc.robot.commands.RotateAuto;
import frc.robot.commands.SpecialRotateAuto;
import frc.robot.commands.StopRobotAutonomous;
import frc.robot.commands.StopWrist;
import frc.robot.commands.WheelsSpitOut;
import frc.robot.commands.WheelsStop;
import frc.robot.commands.WheelsSuckIn;
import frc.robot.commands.WristToDown;
import frc.robot.commands.WristToHigh;
import frc.robot.commands.WristToHighAuto;
import frc.robot.commands.WristToHome;
import frc.robot.subsystems.GripperWheels;
import frc.robot.subsystems.RotateArmMotor;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristMotor;

public class RealRightDoublePlace extends SequentialCommandGroup {

    
    public RealRightDoublePlace(Swerve s_Swerve, RotateArmMotor s_Arm, WristMotor s_Wrist, GripperWheels s_Wheels){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    (Constants.AutoConstants.kMaxSpeedMetersPerSecond),
                    (Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared))
                .setKinematics(Constants.Swerve.swerveKinematics);
        
        TrajectoryConfig slowConfig =
            new TrajectoryConfig(
                    (1.75),
                    (0.4))
                .setKinematics(Constants.Swerve.swerveKinematics);

        var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory driveToCone =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, -0.3)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(4.5, -0.3, new Rotation2d(0)),
                config);
        

        Trajectory pickUpCone =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0.05, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0.15, 0, new Rotation2d(0)),
                slowConfig);

    Trajectory driveBack =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 0.05)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(4.53, 0.05, new Rotation2d(0)),
                config);

        
        
        SwerveControllerCommand DriveToCone =
            new SwerveControllerCommand(
                driveToCone,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        SwerveControllerCommand PickUpCone =
            new SwerveControllerCommand(
                pickUpCone,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand DriveBackToPlace =
               new SwerveControllerCommand(
                driveBack,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);  

        addCommands(

            new InstantCommand((() -> s_Swerve.resetOdometry(driveToCone.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.OneEightyGyro())),       
            
            //PLACE INITIAL CONE
            new ArmToLow(s_Arm),
                new ParallelCommandGroup(
                    new ArmToHigh(s_Arm),
                    new WristToHigh(s_Wrist)
                ),
            new WheelsSuckIn(s_Wheels),
            new WaitCommand(0.3),
            new WheelsStop(s_Wheels),
          
            //RETRACT ARM AND WRIST
            new WristToDown(s_Wrist),
            new StopWrist(s_Wrist),
            new ParallelCommandGroup(
                new ArmToHome(s_Arm),
                new WristToHome(s_Wrist)
            ),
            new ArmStop(s_Arm),
            new StopWrist(s_Wrist),

            //Drive to cone while putting down wrist after 1 second
            new ParallelCommandGroup(
                DriveToCone,
                    new SequentialCommandGroup(
                        new WaitCommand(1),
                        new WristToDown(s_Wrist)
                    )
            ),
            
            new WheelsSpitOut(s_Wheels),
            new StopRobotAutonomous(s_Swerve),
            new InstantCommand((() -> s_Swerve.resetOdometry(pickUpCone.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.zeroGyro())),
            new WaitCommand(0.1),

            PickUpCone,
            new StopRobotAutonomous(s_Swerve),
            new WheelsStop(s_Wheels),
            new WristToHome(s_Wrist),

            new InstantCommand((() -> s_Swerve.resetOdometry(pickUpCone.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.OneEightyGyro())),

            DriveBackToPlace,
            new StopRobotAutonomous(s_Swerve),
            new WaitCommand(0.1),

            //PLACE SECOND PIECE
            new ArmToLow(s_Arm),
                new ParallelCommandGroup(
                    new ArmToHigh(s_Arm),
                    new WristToHigh(s_Wrist)
                ),
            new WheelsSuckIn(s_Wheels),
            new WaitCommand(0.3),
            new WheelsStop(s_Wheels)
            

        );

    }
}