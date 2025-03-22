package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.CoralIntakeConstants;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.Timer;


public class CoralIntakeSubsystem extends SubsystemBase {

  private final CANBus canbus = new CANBus();
  private final TalonFX m_intake = new TalonFX(50, canbus);
  private final TalonFX m_intakeFollower = new TalonFX(51, canbus);
  private final CANrange canrange = new CANrange(52,canbus);

  /* Start at velocity 0, use slot 1 */
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();


private ShuffleboardTab tab = Shuffleboard.getTab("Intake");
private GenericEntry intakeSpeed =
      tab.add("Intake Speed", 0)
         .getEntry();
private GenericEntry intakeVoltage =
      tab.add("Intake Voltage", 0)
         .getEntry();



  /** Creates a new IntakeSubsystem. */
  public CoralIntakeSubsystem() {

         
  TalonFXConfiguration configs = new TalonFXConfiguration();
  /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
  configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
  configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
  configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
  configs.Slot0.kI = 0; // No output for integrated error
  configs.Slot0.kD = 0; // No output for error derivative
  // Peak output of 8 volts
  configs.Voltage.withPeakForwardVoltage(8).withPeakReverseVoltage(8);
  m_intake.getConfigurator().apply(configs);
  m_intakeFollower.setControl(new Follower(m_intake.getDeviceID(), true));
  m_intake.setNeutralMode(NeutralModeValue.Coast);
  m_intakeFollower.setNeutralMode(NeutralModeValue.Coast);
  }

//  COMMANDS  //

  public void disable()
  {
    m_intake.setControl(m_brake);
  }

  public void setVelocity(double desiredRotationsPerSecond)
{
    m_intake.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
}

  public void intake()
{
  m_intake.setControl(m_velocityVoltage.withVelocity(-10));
}

  public void outtake()
{
    m_intake.setControl(m_velocityVoltage.withVelocity(30));
}

public void outtakeAUTO()
{
    m_intake.setControl(m_velocityVoltage.withVelocity(28));
}

public void outtakeL1AUTO()
{
    m_intake.setControl(m_velocityVoltage.withVelocity(15));
}


public boolean coralCheck()
{
  return canrange.getIsDetected().getValue();
}

public Command intakeCommand()
{
  return run(() -> this.intake());
}

public Command outtakeCommand()
{
  return run(() -> this.outtake());
}

public Command withVelocity(double desiredRotationsPerSecond)
{
  return runOnce(() -> this.setVelocity(desiredRotationsPerSecond));
}

public Command stop()
{
    return run(() -> this.disable());
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("canDistance ", canrange.getDistance().getValueAsDouble());
    SmartDashboard.putBoolean("Coral CHECK", canrange.getIsDetected().getValue());


  }
}
