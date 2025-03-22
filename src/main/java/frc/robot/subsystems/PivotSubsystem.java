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

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class PivotSubsystem extends SubsystemBase {


  private double position;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private ShuffleboardTab tab = Shuffleboard.getTab("Pivot");
  private GenericEntry pivotEncoder = tab.add("Pivot Encoder", 0).getEntry();
  private GenericEntry pivotVoltage =
        tab.add("Pivot Voltage", 0)
          .getEntry();
  /** Creates a new PivotSubsystem. */


  private final TalonFX pivot = new TalonFX(21);
  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Start at position 0, use slot 1 */
  private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);


  double homePosition = 0;
  double L4Position = -33.9;
  double L3Position = 0;
  double L2Position = 0;
  double L1Position = 12.2;
  double loadPosition = 26.18;



  public PivotSubsystem() {

    position = 0;
  
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    configs.Slot1.kP = 60; // An error of 1 rotation results in 60 A output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
    // Peak output of 120 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
      .withPeakReverseTorqueCurrent(Amps.of(-120));

     /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
     configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
     configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
     configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
     configs.Slot0.kI = 0; // No output for integrated error
     configs.Slot0.kD = 0; // No output for error derivative
     // Peak output of 8 volts
     configs.Voltage.withPeakForwardVoltage(Volts.of(8))
       .withPeakReverseVoltage(Volts.of(-8));
      pivot.getConfigurator().apply(configs);
      pivot.setNeutralMode(NeutralModeValue.Brake);
      pivot.setPosition(position);

  }

public void setHoldPosition(double holdposition) {
  position = holdposition;
}

public void setVelocity(double speed)
{
  pivot.setControl(m_velocityVoltage.withVelocity(speed));
  position = pivot.getPosition().getValueAsDouble();
}

public boolean CheckPositionHome()
{
 return MathUtil.isNear(homePosition,pivot.getPosition().getValueAsDouble(), 1);
}

public boolean CheckPositionLoad()
{
 return MathUtil.isNear(loadPosition,pivot.getPosition().getValueAsDouble(), 1);
}

public boolean CheckPositionL4()
{
 return MathUtil.isNear(L4Position,pivot.getPosition().getValueAsDouble(), 1);
}

public boolean CheckPositionL4LOWER()
{
 return MathUtil.isNear(-34.9,pivot.getPosition().getValueAsDouble(), .5);
}

public boolean CheckPositionL3()
{
 return MathUtil.isNear(L3Position,pivot.getPosition().getValueAsDouble(), 1);
}

public boolean CheckPositionL2()
{
 return MathUtil.isNear(L2Position,pivot.getPosition().getValueAsDouble(), 1);
}

public boolean CheckPositionL1()
{
 return MathUtil.isNear(L1Position,pivot.getPosition().getValueAsDouble(), 1);
}

public void setPosition(double setPoint)
{
  pivot.setControl(m_positionTorque.withPosition(setPoint));
}


public double getEncoder()
{
  return pivot.getPosition().getValueAsDouble();
}

public Command slowUp()
{
  return run(() -> this.setVelocity(10));
}


public Command slowDown()
{
  return run(() -> this.setVelocity(-10));
}

public void stop()
{
  pivot.setControl(m_brake);
}


public Command stopCommand()
{
  return run(() -> this.stop());
}


public Command withPosition(double setPoint)
{
  return run(() -> this.setPosition(setPoint));
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
//pivot.setControl(m_positionTorque.withPosition(position));
//SmartDashboard.putBoolean("limit checks", LimitChecks());
SmartDashboard.putNumber("Pivot Encoder", pivot.getPosition().getValueAsDouble());
SmartDashboard.putNumber("Pivot Velocity", pivot.getVelocity().getValueAsDouble());

}
}