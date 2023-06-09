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


public class ArmToLow extends CommandBase
{
    private final RotateArmMotor s_Arm;
    
    

    public ArmToLow(RotateArmMotor subsystem)
    {
        s_Arm = subsystem;
        
        addRequirements(s_Arm);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        s_Arm.ToPosition(-12, 0.6);
    }

    @Override
    public void end(boolean interrupted)
    {
        s_Arm.Stop();
    }

    @Override
    public boolean isFinished() 
    {
        if(s_Arm.ShoulderPosition()<(-12*2048) + 1000 && s_Arm.ShoulderPosition()>(-12*2048)-1000 )
        {
            return true;
        } else
        {
            return false;
        }
    }
}
