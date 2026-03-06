package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  default void updateInputs(ShooterIOInputs inputs) {}

  default void setShooterVoltage(double volts) {}

  default void stop() {}

  @AutoLog
  class ShooterIOInputs {
    double shooterVelocityRadPerSec = 0d;
    double shooterAppliedVolts = 0d;
  }
}
