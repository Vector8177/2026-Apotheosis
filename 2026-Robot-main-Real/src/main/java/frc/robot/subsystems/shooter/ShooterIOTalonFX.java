// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;


import java.rmi.dgc.VMID;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterMotor;
  private final TalonFX helperMotor;
  private final NeutralOut neutralOut = new NeutralOut();
  private final CANBus canbus = new CANBus("Turret Canivore");
  private VelocityVoltage velocityVoltage;

  public ShooterIOTalonFX() {
    shooterMotor = new TalonFX(ShooterConstants.MOTOR_ID, canbus);
    helperMotor = new TalonFX(ShooterConstants.HELPER_ID, canbus);
    
    TalonFXConfiguration config = new TalonFXConfiguration();

    // config.Slot0.kP = 1;
    // config.Slot0.kI = 0;
    // config.Slot0.kD = 0.01;

    //config.Slot0.kS = 0;


    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotionMagic.MotionMagicAcceleration = 100;

    shooterMotor.getConfigurator().apply(config, 0.5);

    shooterMotor.setNeutralMode(NeutralModeValue.Coast);

    helperMotor.getConfigurator().apply(config, .5);
    
    helperMotor.setNeutralMode(NeutralModeValue.Coast);
    
    velocityVoltage = new VelocityVoltage(0);
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
  public void setShooterVoltage(double velocity) {
    // VelocityVoltage v = new VelocityVoltage(velocity);
    
    
    
    // shooterMotor.setControl(v);
    // helperMotor.setControl(v);


    
    // Logger.recordOutput("Velocity Voltage", v.Velocity);

     Logger.recordOutput("Shooter Target Speed", velocity);
    // Logger.recordOutput("Shooter Motor Target Speed", shooterMotor.getVelocity().getValueAsDouble());
    // Logger.recordOutput("Shooter Helper Motor Target Speed", helperMotor.getVelocity().getValueAsDouble());

  
    //shooterMotor.setVoltage(velocity);
    helperMotor.setVoltage(velocity);
  }

  @Override
  public double getPosition(){
    return shooterMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void stop() {
    shooterMotor.setControl(neutralOut);
  }
}
