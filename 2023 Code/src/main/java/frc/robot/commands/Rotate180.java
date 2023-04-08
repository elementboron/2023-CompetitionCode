/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class Rotate180 extends CommandBase
{
    private final Swerve s_Swerve;
    private double desiredRot;

    public Rotate180(Swerve subsystem, double desiredRot)
    {
        s_Swerve = subsystem;
        this.desiredRot = desiredRot;
        
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        s_Swerve.drive(new Translation2d(0,0), 180, false, false);
    }

    @Override
    public boolean isFinished() 
    {
        return true;
        
    }
}
