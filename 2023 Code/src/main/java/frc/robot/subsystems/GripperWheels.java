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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommonMethodExtensions;
import frc.robot.Constants;
//import frc.robot.Commands.SpinNeo550;
import frc.robot.Robot;

public class GripperWheels extends SubsystemBase 
{

  /* Motors/Encoders/Pneumatics */
  CANSparkMax wheelMotor = Robot.wheelsMotor;
  RelativeEncoder encoder = wheelMotor.getEncoder();
  Solenoid solenoid = Robot.wristSolenoid;
  

  /* Default Command for Teleop (driven directly as the difference of the right and left trigger) */
  public void Drive(DoubleSupplier positiveRotation, DoubleSupplier negativeRotation)
  {
    wheelMotor.set((positiveRotation.getAsDouble()-negativeRotation.getAsDouble()));
  }


  /* Toggles the Pneumatics */
  public void Shift()
  {
    solenoid.toggle();
  }


  /* Turn On Pneumatics */
  public void ActivatePneumatics()
  {
    solenoid.set(true);
  }


  /* Turn Off Pneumatics */
  public void DeactivatePneumatics()
  {
    solenoid.set(false);
  }
  

  /* Stops the Intake */
  public void Stop()
  {
    wheelMotor.set(0);
  }


  /* Sets the Intake To Suck In Cones (auto use only) */
  public void SuckIn()
  {
    wheelMotor.set(0.6);
  }


  /* Sets the Intake To Spit Out Cones (auto use only) */
  public void SpitOut()
  {
    wheelMotor.set(-1);
  }


  public boolean isFinished() 
  {
    return true;
  }
}
