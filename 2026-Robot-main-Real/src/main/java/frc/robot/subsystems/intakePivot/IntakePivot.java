package frc.robot.subsystems.intakePivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.subsystems.intakePivot.IntakePivotIO.IntakePivotIOInputs;

import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final PIDController pidController;
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
  private double targetPosition;
  private ArmFeedforward feedForward;

  public IntakePivot(IntakePivotIO io) {

    this.io = io;
    pidController = new PIDController(IntakePivotConstants.kP, IntakePivotConstants.kI, IntakePivotConstants.kD);
    pidController.setTolerance(.2);

    feedForward =
        new ArmFeedforward(
            IntakePivotConstants.kS, IntakePivotConstants.kG, IntakePivotConstants.kV, IntakePivotConstants.kA);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    //Logger.processInputs("IntakePivot", inputs);
    Logger.recordOutput("IntakePivot Current Position", io.getPosition());
    Logger.recordOutput("Encoder Position", io.getEncoderPosition());

    double pidMotorSpeed = 
        pidController.calculate(io.getPosition(), targetPosition)
            + feedForward.calculate(targetPosition, 0);
    
    Logger.recordOutput("IntakePivot Speed", pidMotorSpeed);

    // pidMotorSpeed = MathUtil.clamp(pidMotorSpeed, -1, 1);

    setMotor(
        MathUtil.clamp((pidMotorSpeed), -IntakePivotConstants.MAX_VOLTAGE, IntakePivotConstants.MAX_VOLTAGE));

    // setMotor(pidMotorSpeed * IntakePivotConstants.MAX_VOLTAGE);

    Logger.recordOutput("Intake Pivot position", io.getPosition());
    Logger.recordOutput("Intake Pivot Target Position", targetPosition);
  }

  public void setMotor(double voltage) {
    io.setVoltage(voltage);
  }

  public void setPosition(double position) {
    Logger.recordOutput("IntakePivot Target Position", position);
    System.out.println(io.getPosition() + " IntakePivot");
    
    targetPosition = (((io.getEncoderPosition()-position)*(180/Math.PI))/360)*40;
  }

  public void setIntakePivotSetpoint(double offset) {
    targetPosition = (io.getPosition() + offset);
  }

  public void stopPivot() {
    targetPosition = 0;
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }
}
