package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  class HoodIOInputs {
    public double hoodVelocityRadPerSec = 0.0;
    public double hoodAbsoluteEncoderPosition = 0.0;
    public double hoodAppliedVolts = 0.0;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void setPosition(double position) {}

  default void setVoltage(double speed) {}

  default double getPosition() {
    return 0;
  }
}
