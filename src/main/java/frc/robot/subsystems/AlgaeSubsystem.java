// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
  
  private final CANBus canbus = new CANBus();
  private final TalonFX m_algae = new TalonFX(60, canbus);
  private final NeutralOut m_brake = new NeutralOut();
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

  public AlgaeSubsystem() {

    
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(8).withPeakReverseVoltage(8);
    m_algae.getConfigurator().apply(configs);
    m_algae.setNeutralMode(NeutralModeValue.Coast);
  
  }

  private void disable()
  {
    m_algae.setControl(m_brake);
  }
  public void setVelocity(double desiredRotationsPerSecond)
  {
    m_algae.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
  }

  public Command withVelocity(double desiredRotationsPerSecond)
  {
    return run(()-> this.setVelocity(desiredRotationsPerSecond));
  }

  public Command stop()
  {
    return run(()-> this.disable());
  }
}