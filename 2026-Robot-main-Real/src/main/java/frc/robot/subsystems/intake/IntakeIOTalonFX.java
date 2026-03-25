// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor;
  private final TalonFX indexMotor;
  private final TalonFX startIntakeMotor;
  private final NeutralOut neutralOut = new NeutralOut();
  private TalonFXConfiguration config;

  public IntakeIOTalonFX() {
    intakeMotor = new TalonFX(IntakeConstants.MOTOR_ID);
    indexMotor = new TalonFX(IntakeConstants.MOTOR_ID2);
    startIntakeMotor = new TalonFX(IntakeConstants.MOTOR_ID3);

    config = new TalonFXConfiguration();
    
    // Setting Stator Current Limits
    config.CurrentLimits.StatorCurrentLimit = 25;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Setting Supply Current Limits
    config.CurrentLimits.SupplyCurrentLimit = 25; //25
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicAcceleration = 100;

    intakeMotor.getConfigurator().apply(config, 0.5);
    indexMotor.getConfigurator().apply(config, 0.5);
    startIntakeMotor.getConfigurator().apply(config, 0.5);

    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    indexMotor.setNeutralMode(NeutralModeValue.Brake);
    startIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void setCurrentLimit(double amps) {
    config.CurrentLimits.StatorCurrentLimit = amps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    config.CurrentLimits.SupplyCurrentLimit = amps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeAppliedVolts =
        intakeMotor.getDutyCycle().getValueAsDouble()
            * intakeMotor.getSupplyVoltage().getValueAsDouble();
    inputs.intakeVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(intakeMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setBottomIntakeVoltage(double volts){
    startIntakeMotor.setVoltage(-volts);
  }

  @Override
  public void setTopIntakeVoltage(double volts){
    indexMotor.setVoltage(volts);
  }

  @Override
  public double getCurrent() {
    return intakeMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void stop() {
    intakeMotor.setControl(neutralOut);
    indexMotor.setControl(neutralOut);
    startIntakeMotor.setControl(neutralOut);
  }
}
