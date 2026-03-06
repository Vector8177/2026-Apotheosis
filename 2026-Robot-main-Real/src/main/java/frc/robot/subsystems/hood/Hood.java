package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;

import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final PIDController pidController;
  private final HoodIO io;
  private final HoodIOInputs inputs = new HoodIOInputs();
  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  private double targetPosition;
  private ArmFeedforward feedForward;

  static {
    //fill in hood angle values
  }

  public Hood(HoodIO io) {

    this.io = io;
    pidController = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);
    pidController.setTolerance(.2);

    feedForward =
        new ArmFeedforward(
            HoodConstants.kS, HoodConstants.kG, HoodConstants.kV, HoodConstants.kA);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.processInputs("Hood", inputs);
    //Logger.recordOutput("Hood Current Position", io.getPosition());

    int closest = (int)LimelightHelpers.getFiducialID("limelight-turret");
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", new int[]{closest});
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-turret");
    double distance = fiducials[0].distToCamera;
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", TurretConstants.allId);
    targetPosition = hoodAngleMap.get(distance);

    double pidMotorSpeed =
        pidController.calculate(io.getPosition(), targetPosition)
            + feedForward.calculate(targetPosition, 0);
    // Logger.recordOutput("Hood Speed", pidMotorSpeed);
    setMotor(
        MathUtil.clamp((pidMotorSpeed), -HoodConstants.MAX_VOLTAGE, HoodConstants.MAX_VOLTAGE));
  }

  public void setMotor(double voltage) {
    io.setVoltage(voltage);
  }

  public void setPosition(double position) {
    Logger.recordOutput("Hood Target Position", position);
    System.out.println(io.getPosition() + " Hood");
    targetPosition = position;
  }

  public void setHoodSetpoint(double offset) {
    targetPosition = (io.getPosition() + offset);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }
}
