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
import frc.robot.autos.AprilAuto2;
import frc.robot.subsystems.*;


public class AprilExecutor extends CommandBase
{
    private final Limelight s_Limelight;
    private final Swerve s_Swerve;
    double [] botpose;
    
    

    public AprilExecutor(Limelight subsystem, Swerve subsystem2, double[] botpose)
    {
        s_Limelight = subsystem;
        s_Swerve = subsystem2;
        this.botpose = botpose;
        
        addRequirements(s_Limelight, s_Swerve);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        new AprilAuto2(s_Swerve, s_Limelight, s_Limelight.distanceCalculator(6, -1, botpose));

    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
