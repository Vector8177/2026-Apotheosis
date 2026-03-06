package frc.robot.subsystems.intakePivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  class IntakePivotIOInputs {
    public double intakePivotVelocityRadPerSec = 0.0;
    public double intakePivotAbsoluteEncoderPosition = 0.0;
    public double intakePivotAppliedVolts = 0.0;
  }

  default void updateInputs(IntakePivotIOInputs inputs) {}

  default void setPosition(double position) {}


  default void setVoltage(double speed) {}

  default double getPosition() {
    return 0;
  }

  default double getEncoderPosition(){
    return 0;
  }
}
