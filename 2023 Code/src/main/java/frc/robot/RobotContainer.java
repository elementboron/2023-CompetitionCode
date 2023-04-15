package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.TemporaryPlaceAuto;
import frc.robot.autos.ClimbingTest;
import frc.robot.autos.ParallelCommandTesting;
import frc.robot.autos.RealLeftDoublePlace;
import frc.robot.autos.RealLeftPlaceClimb;
import frc.robot.autos.RealLeftPlaceGrab;
import frc.robot.autos.RealPlaceClimbMiddleAuto;
import frc.robot.autos.RealRightDoublePlace;
import frc.robot.autos.RealRightPlaceGrab;
import frc.robot.autos.RealRightDoublePlace;
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
    private final Joystick operator = new Joystick(1);

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
    private final JoystickButton armToMid = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton armToGroundPickup = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Operator Buttons */
    private final JoystickButton wristToLow = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton armToHigh = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton wristToHome = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton armToHome = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton wristToHigh = new JoystickButton(operator, XboxController.Button.kA.value); 
    private final JoystickButton armToLow = new JoystickButton(operator, XboxController.Button.kX.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final RotateArmMotor s_Arm = new RotateArmMotor();
    private final WristMotor s_Wrist = new WristMotor();
    private final GripperWheels s_Wheels = new GripperWheels();
    private final Pneumatics s_Pneumatics = new Pneumatics();


    /* Driving Speed Control */
    public static final double desiredSpeed = 1;
    public static double speedController = desiredSpeed;
    public static double turnController = speedController*0.5;



    /* The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        //Setting the default commands makes a certain command, such as joystick input, the primary driver of each subsystem. 
        //This makes any other command that requires the same subsystem, such as the ones in ConfigureButtonBindings(), interrupt the default command until it is finished.

        //Cubing the input leads to smoother acceleration while still keeping the same maximum value
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
                () -> operator.getRawAxis(translationAxis)*0.8
            )
        );

        s_Wrist.setDefaultCommand(
            new TeleopWrist(
                s_Wrist,
                () -> operator.getRawAxis(wristRotationAxis)
            )
        );

        s_Wheels.setDefaultCommand(
            new TeleopWheels(
                s_Wheels,
                () -> operator.getRawAxis(extendAxis),
                () -> operator.getRawAxis(retractAxis)
            )
        );

        //Periodically checks buttons and executes commands accordingly
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
        armToMid.onTrue(new ArmToMid(s_Arm));
        armToLow.onTrue(new ArmToLow(s_Arm));
        armToGroundPickup.onTrue(new ArmToGroundPickUp(s_Arm).andThen(new WristToGroundPickUp(s_Wrist)).andThen(new ActivatePneumatics(s_Pneumatics)));
        wristToHome.onTrue(new DeactivatePneumatics(s_Pneumatics).andThen(new WristToHome(s_Wrist)));
        wristToLow.onTrue(new WristToDown(s_Wrist));
        wristToHigh.onTrue(new WristToHigh(s_Wrist));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
    
        final Command auto = new RealLeftPlaceGrab(s_Swerve, s_Arm, s_Wrist, s_Wheels, s_Pneumatics);
        //final Command auto = new RealRightDoublePlace(s_Swerve, s_Arm, s_Wrist, s_Wheels, s_Pneumatics);
        //final Command auto = new RealRightPlaceGrab(s_Swerve, s_Arm, s_Wrist, s_Wheels, s_Pneumatics);
        //final Command auto = new RealLeftDoublePlace(s_Swerve, s_Arm, s_Wrist, s_Wheels, s_Pneumatics);
        //final Command auto = new RealPlaceClimbMiddleAuto(s_Swerve, s_Arm, s_Wrist, s_Wheels, s_Pneumatics);

        //final Command auto = new ParallelCommandTesting(s_Swerve, s_Arm, s_Wrist, s_Wheels);
        //final Command auto = new TemporaryPlaceAuto(s_Swerve, s_Arm, s_Wrist, s_Wheels);
        //final Command auto = new ClimbingTest(s_Swerve, s_Arm, s_Wrist, s_Wheels);
        return auto;
    }
}
