/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CommonMethodExtensions;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class AprilTracker extends CommandBase
{
    private final Limelight s_Limelight;
    private final Swerve s_Swerve;
    
    

    public AprilTracker(Limelight subsystem, Swerve subsystem2)
    {
        s_Limelight = subsystem;
        s_Swerve = subsystem2;
        
        addRequirements(s_Limelight, s_Swerve);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        if(s_Limelight.checkForAprilTags()){
            s_Swerve.drive(new Translation2d(1,0), 0, true, true);
        } else {
            s_Swerve.drive(new Translation2d(0,0), 0, true, true);
        }
    }

    @Override
    public boolean isFinished() 
    {
        return !s_Limelight.checkForAprilTags();
    }
}
