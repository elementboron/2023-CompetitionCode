/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class LimelightCalculator extends CommandBase
{
    private final Limelight s_Limelight;
    private double desiredX;
    private double desiredY;
    private double[] botpose;
    
    

    public LimelightCalculator(Limelight subsystem, double desiredX, double desiredY, double[] botpose)
    {
        s_Limelight = subsystem;
        this.desiredX = desiredX;
        this.desiredY = desiredY;
        this.botpose = botpose;
        
        addRequirements(s_Limelight);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        double[] calculatedDistance = s_Limelight.distanceCalculator(desiredX, desiredY, botpose);
        RobotContainer.distances = calculatedDistance;
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
