// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//CTRE Imports Motor Imports
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShoulderSubsystemSPARK extends SubsystemBase {


  private double position;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private ShuffleboardTab tab = Shuffleboard.getTab("Pivot");
  private GenericEntry pivotEncoder = tab.add("Shoulder Encoder", 0).getEntry();
  private GenericEntry pivotVoltage =
        tab.add("Shoulder Voltage", 0)
          .getEntry();
  /** Creates a new PivotSubsystem. */


  private SparkMax pivot = new SparkMax(0, MotorType.kBrushless);
  private SparkClosedLoopController c_pidController;  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Start at position 0, use slot 1 */
  private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();


  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);


  double homePosition = 0;
  double L4Position = 0;
  double L3Position = 0;
  double L2Position = 0;



  public ShoulderSubsystemSPARK() {

    position = 0;
  
    
SparkMaxConfig config = new SparkMaxConfig();
     
     // PID coefficients
     kP = 0.03; 
     kI = 0;
     kD = 0; 
     kIz = 0; 
     kFF = 0.000015; 
     kMaxOutput = 1; 
     kMinOutput = -1;
     maxRPM = 5700;


config
    .inverted(false)
    .idleMode(IdleMode.kBrake);
config.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pidf(kP, kI, kD, kFF, ClosedLoopSlot.kSlot0);

    pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivot.getEncoder().setPosition(position);

  }

public void setHoldPosition(double holdposition) {
  position = holdposition;
}

public void setVelocity(double speed)
{
  pivot.set(speed);
  position = pivot.getEncoder().getPosition();
}

public boolean CheckPositionHome()
{
 return MathUtil.isNear(homePosition,pivot.getEncoder().getPosition(), 1);
}

public boolean CheckPositionL4()
{
 return MathUtil.isNear(L4Position,pivot.getEncoder().getPosition(), 1);
}

public boolean CheckPositionL3()
{
 return MathUtil.isNear(L3Position,pivot.getEncoder().getPosition(), 1);
}

public boolean CheckPositionL2()
{
 return MathUtil.isNear(L2Position,pivot.getEncoder().getPosition(), 1);
}

private void setPosition(double setPoint)
{
  position = setPoint;
}


public double getEncoder()
{
  return pivot.getEncoder().getPosition();
}

public Command slowUp()
{
  return run(() -> this.setVelocity(.1));
}


public Command slowDown()
{
  return run(() -> this.setVelocity(-.1));
}


public Command stop()
{
  return run(() -> this.setVelocity(0));
}


public Command withPosition(double setPoint)
{
  return runOnce(() -> this.setPosition(setPoint));
}

public Command setHomePosition()
{
  return run(() -> this.setPosition(homePosition)); 
}

public Command setL4Position()
{
  return run(() -> this.setPosition(L4Position)); 
}

public Command setL3Position()
{
  return run(() -> this.setPosition(L3Position)); 
}

public Command setL2Position()
{
  return run(() -> this.setPosition(L2Position)); 
}


@Override
public void periodic() {
  c_pidController.setReference(position, SparkMax.ControlType.kPosition);
  //SmartDashboard.putBoolean("limit checks", LimitChecks());
SmartDashboard.putNumber("Pivot Encoder", pivot.getEncoder().getPosition());

}
}