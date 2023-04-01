/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;
import javax.sql.rowset.WebRowSet;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommonMethodExtensions;
import frc.robot.Constants;
//import frc.robot.Commands.SpinNeo550;
import frc.robot.Robot;

public class RotateArmMotor extends SubsystemBase 
{

  /* Motors/Encoders */
  public WPI_TalonFX motorRotate = new WPI_TalonFX(Constants.Swerve.rotateArmID);

 
  /* Default Teleop Command (driven directly by joystick) */
  public void TeleOp(DoubleSupplier armRotation) 
  {
      motorRotate.set(armRotation.getAsDouble());
  }


  /* Custom PID Program for Set Position (called by a button press) */
  public void SetPosition(double desiredPosition, double speed)
  {
    double targetPosition = desiredPosition * 2048;
    if(motorRotate.getSelectedSensorPosition()>targetPosition)
    {
      motorRotate.set((speed*((4*(Math.abs((motorRotate.getSelectedSensorPosition()-targetPosition))/targetPosition)))-0.1));
    } else if (motorRotate.getSelectedSensorPosition()<targetPosition)
    {
      motorRotate.set((-speed*((4*(Math.abs((motorRotate.getSelectedSensorPosition()-targetPosition))/targetPosition)))+0.1));
    }
  }


  /* Basic Set Position Command (called by a button press) */
  public void ToPosition(double desiredPosition, double speed)
  {
    double targetPosition = desiredPosition * 2048;
    if(motorRotate.getSelectedSensorPosition() > targetPosition)
    {
      motorRotate.set(-speed);
    } else if (motorRotate.getSelectedSensorPosition() < targetPosition)
    {
      motorRotate.set(speed);
    }
  }


  /* Brings the Arm To Home */
  public void ToHome()
  {
    if(motorRotate.getSelectedSensorPosition()<0)
    {
      motorRotate.set(0.8);
    } 
  }


  /* Sets Forward and Reverse Soft Limits for the Shoulder */
  public void ShoulderSoftLimits()
  {
    motorRotate.configForwardSoftLimitEnable(true);
    motorRotate.configForwardSoftLimitThreshold(-6*2048);
    motorRotate.configReverseSoftLimitEnable(true);
    motorRotate.configReverseSoftLimitThreshold(-110*2048);
  }


  /* Returns the Current Shoulder Position */
  public double ShoulderPosition()
  {
    return motorRotate.getSelectedSensorPosition();
  }


  /* Periodically Posts Arm Values to SmartDashboard */
  public void UpdateSmartDashNums()
  {
    SmartDashboard.putNumber("RotateMotor Temperature:", motorRotate.getTemperature());
    SmartDashboard.putNumber("RotateMotor Current:", motorRotate.getSupplyCurrent());
    SmartDashboard.putNumber("RotateMotor Rotations:",motorRotate.getSelectedSensorPosition()/2048);
  }
  

  /* Stops the Shoulder */
  public void Stop()
  {
    motorRotate.set(0);
  }


  public boolean isFinished() 
  {
    return true;
  }
}
