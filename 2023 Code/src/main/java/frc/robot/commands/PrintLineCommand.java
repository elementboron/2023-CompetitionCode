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


public class PrintLineCommand extends CommandBase
{
    private final RotateArmMotor s_Arm;
    private final String text;
    
    

    public PrintLineCommand(RotateArmMotor subsystem, String text)
    {
        this.text = text;
        s_Arm = subsystem;
        
        addRequirements(s_Arm);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        s_Arm.TextOutput(text);
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
