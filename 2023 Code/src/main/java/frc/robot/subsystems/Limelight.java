/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import javax.sql.rowset.WebRowSet;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommonMethodExtensions;
import frc.robot.Constants;
//import frc.robot.Commands.SpinNeo550;
import frc.robot.Robot;

public class Limelight extends SubsystemBase 
{

  /* Creates an Instance of the Limelight if One Does Not Exist */
  public static Limelight Instance;

  public static Limelight Instance(){
    if (Instance==null){
        Instance= new Limelight();
    }
    return Instance;
}

  public static double[] distance = {0,0,0,0};
  

  /* Sets the Limelight to DriveCamera Mode (increases exposure) */
  public void InitLimelight()
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setDouble(0);
  }


  
  public boolean checkForAprilTags(){
    NetworkTable aprilTable = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry aprilDetection = aprilTable.getEntry("tv");
    double aprilValue = aprilDetection.getDouble(0.0);
    if (aprilValue == 1)
    {
      return true;
    } else {
      return false;
    }
    
  }

  public double[] getBotPose()
  {
    
    double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    return botpose;
  }

  public double[] distanceCalculator(double desiredX, double desiredY, double[] botpose)
  {
    double xDistance = desiredX - botpose[0];
    double yDistance = desiredY - botpose[1];
    double midXPoint = (desiredX + botpose[0])/2;
    double midYPoint = (desiredY + botpose[1])/2;
    
    double slope = yDistance/xDistance;
    double slopeDirection = slope/Math.abs(slope);
    SmartDashboard.putNumber("slope direction", slopeDirection);
    SmartDashboard.putNumber("Y correction", yDistance);
    SmartDashboard.putNumber("X Correction", xDistance);
    SmartDashboard.putNumber("estimated botpose X", botpose[0]);
    SmartDashboard.putNumber("estimated botpose Y", botpose[1]);
    double[] distances = {xDistance, yDistance, botpose[0], botpose[1], desiredX, desiredY};
    distance = distances;
    SmartDashboard.putNumberArray("Limelight distances", distance);


    return distances;
  }


/* Periodically Posts Limelight Values to SmartDashboard For Viewing */
  public void UpdateSmartDashboardNums()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");    
    double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[3]);


    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");


    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double v = tv.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Target?", v);
    SmartDashboard.putNumberArray("botpose" , botpose);
    SmartDashboard.putNumberArray("Limelight distances", distance);
    }


  public boolean isFinished() 
  {
    return true;
  }
}
