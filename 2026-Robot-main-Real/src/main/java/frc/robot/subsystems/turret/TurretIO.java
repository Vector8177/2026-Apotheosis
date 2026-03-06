package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  class TurretIOInputs {
    public double turretVelocityRadPerSec = 0.0;
    public double turretAbsoluteEncoderPosition = 0.0;
    public double turretAppliedVolts = 0.0;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void setPosition(double position) {}

  default void setVoltage(double speed) {}

  default double getPosition() {
    return 0;
  }

  
}
