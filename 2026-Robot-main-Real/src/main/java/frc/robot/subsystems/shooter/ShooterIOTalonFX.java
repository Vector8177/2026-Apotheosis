// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterMotor;
  private final TalonFX helperMotor;
  private final NeutralOut neutralOut = new NeutralOut();

  public ShooterIOTalonFX() {
    shooterMotor = new TalonFX(ShooterConstants.MOTOR_ID);
    helperMotor = new TalonFX(ShooterConstants.HELPER_ID);
    
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotionMagic.MotionMagicAcceleration = 100;

    shooterMotor.getConfigurator().apply(config, 0.5);

    shooterMotor.setNeutralMode(NeutralModeValue.Brake);

    helperMotor.getConfigurator().apply(config, .5);
    
    helperMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterAppliedVolts =
        shooterMotor.getDutyCycle().getValueAsDouble()
            * shooterMotor.getSupplyVoltage().getValueAsDouble();
    inputs.shooterVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(shooterMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void setShooterVoltage(double volts) {
    //shooterMotor.setVoltage(volts);
    VelocityTorqueCurrentFOC vtc = new VelocityTorqueCurrentFOC(volts);
    Logger.recordOutput("Target Velocity Shooter", vtc.getVelocityMeasure().toString());
    //shooterMotor.setControl(new VelocityTorqueCurrentFOC(volts));
  }

  @Override
  public void stop() {
    shooterMotor.setControl(neutralOut);
  }
}
