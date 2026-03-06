package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();
  private double targetSpeed = 0;
  private double targetSpeed2 = 0;
  private boolean stopIntake = false;

  public Intake(IntakeIO io) {
    this.io = io;
    new SlewRateLimiter(.4);
  }

  

  private void setSpeedRaw(double speed, double speed2) {
    speed = MathUtil.clamp(speed, -1, 1);
    speed2 = MathUtil.clamp(speed2, -1, 1);
    // Logger.recordOutput("Intake speed", speed);
    
    if(speed2==0 && speed!=0){
      io.setTopIntakeVoltage((Math.abs(speed)+.1)*IntakeConstants.MAX_VOLTAGE);
      io.setIntakeVoltage(speed * IntakeConstants.MAX_VOLTAGE);
    }
    else{
      io.setTopIntakeVoltage(speed2*IntakeConstants.MAX_VOLTAGE);
      io.setIntakeVoltage((Math.abs(speed2)) * IntakeConstants.MAX_VOLTAGE);
    }
  }

  public void setSpeed(double speed) {
    targetSpeed = speed;
    
  }

  public void setTopSpeed(double speed){
    targetSpeed2 = speed;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("Intake", inputs);

    setSpeedRaw(targetSpeed, targetSpeed2);
  }

  public void stop() {
    io.stop();
  }
}
