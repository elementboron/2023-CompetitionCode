package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.autos.AutoBalanceTesting;
import frc.robot.autos.RealRightPlaceGrabClimb;
import frc.robot.autos.TemporaryPlaceAuto;
import frc.robot.autos.ClimbingTest;
import frc.robot.autos.RealLeftPlaceClimb;
import frc.robot.autos.RealLeftPlaceGrab;
//import frc.robot.autos.ParallelCommandTesting;
//import frc.robot.autos.PathPlanningAuto;
import frc.robot.autos.RealPlaceClimbMiddleAuto;
import frc.robot.autos.RealRightPlaceClimb;
import frc.robot.autos.RealRightPlaceGrab;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int wristRotationAxis = XboxController.Axis.kRightY.value;
    private final int extendAxis = XboxController.Axis.kLeftTrigger.value;
    private final int retractAxis = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton slowDown = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton armToLow = new JoystickButton(driver, XboxController.Button.kA.value);

    private final JoystickButton wristToLow = new JoystickButton(driver2, XboxController.Button.kLeftBumper.value);
    private final JoystickButton armToHigh = new JoystickButton(driver2, XboxController.Button.kY.value);
    private final JoystickButton wristToHome = new JoystickButton(driver2, XboxController.Button.kRightBumper.value);
    private final JoystickButton armToHome = new JoystickButton(driver2, XboxController.Button.kB.value);
    private final JoystickButton wristToHigh = new JoystickButton(driver2, XboxController.Button.kA.value); 
    private final JoystickButton cancelCommand = new JoystickButton(driver2, XboxController.Button.kX.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final RotateArmMotor s_Arm = new RotateArmMotor();
    private final WristMotor s_Wrist = new WristMotor();
    private final GripperWheels s_Wheels = new GripperWheels();
    //private final Pneumatics s_Pneumatics = new Pneumatics();
    //private final GripperWheelsSubsystem s_Wheels = new GripperWheelsSubsystem();

    // Driving Control //
    public static final double desiredSpeed = 1;
    public static double speedController = desiredSpeed;
    public static double turnController = speedController*0.5;



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> speedController*Math.pow(-driver.getRawAxis(translationAxis), 3), 
                () -> speedController*Math.pow(-driver.getRawAxis(strafeAxis), 3), 
                () -> turnController*Math.pow(-driver.getRawAxis(rotationAxis), 3), 
                () -> robotCentric.getAsBoolean()
            )
        );

        s_Arm.setDefaultCommand(      
            new TeleopArm(
                s_Arm,
                () -> driver2.getRawAxis(translationAxis)*0.8

            )
        );

        s_Wrist.setDefaultCommand(
            new TeleopWrist(
                s_Wrist,
                () -> driver2.getRawAxis(wristRotationAxis)
            )
        );

        s_Wheels.setDefaultCommand(
            new TeleopWheels(
                s_Wheels,
                () -> driver2.getRawAxis(extendAxis),
                () -> driver2.getRawAxis(retractAxis)
            )
        );

        configureButtonBindings();
	}

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() 
        
    {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        slowDown.onTrue(new SlowDown(s_Swerve));
        slowDown.onFalse(new RegularSpeed(s_Swerve));
        armToHome.onTrue(new ArmToHome(s_Arm));
        armToHigh.onTrue(new ArmToHigh(s_Arm));
        wristToHome.onTrue(new WristToHome(s_Wrist));
        wristToLow.onTrue(new WristToDown(s_Wrist));
        wristToHigh.onTrue(new WristToHigh(s_Wrist));
        cancelCommand.onTrue(new StopWrist(s_Wrist));
        cancelCommand.onTrue(new ArmStop(s_Arm));
        armToLow.onTrue(new ArmToLow(s_Arm));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
    
        //final Command auto = new RealLeftPlaceGrab(s_Swerve, s_Arm, s_Wrist, s_Wheels);
        //final Command auto = new RealRightPlaceGrab(s_Swerve, s_Arm, s_Wrist, s_Wheels);
        final Command auto = new RealPlaceClimbMiddleAuto(s_Swerve, s_Arm, s_Wrist, s_Wheels);

        //final Command auto = new TemporaryPlaceAuto(s_Swerve, s_Arm, s_Wrist, s_Wheels);
        //final Command auto = new ClimbingTest(s_Swerve, s_Arm, s_Wrist, s_Wheels);
        return auto;
    }
}
