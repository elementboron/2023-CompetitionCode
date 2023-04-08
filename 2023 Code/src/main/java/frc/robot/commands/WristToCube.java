/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CommonMethodExtensions;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class WristToCube extends CommandBase
{
    private final WristMotor s_Wrist;
    
    

    public WristToCube(WristMotor subsystem)
    {
        s_Wrist = subsystem;
        
        addRequirements(s_Wrist);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute() 
    {  
        s_Wrist.ToPosition(110, 1);
    }

    @Override
    public void end(boolean interrupted)
    {
        s_Wrist.Stop();
    }

    @Override
    public boolean isFinished() 
    {
        if(s_Wrist.WristPosition()<(110 + 4) && s_Wrist.WristPosition()>(110 - 4))
        {
            return true;
        } else
        {
            return false;
        }
    }
}
