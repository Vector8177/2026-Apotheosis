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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final PIDController pidController;
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  private double targetPosition;
  private ArmFeedforward feedForward;
  double tempDistance = -1;
  double tempClosest = -1;
  boolean autoAlign;

  static {
    //fill in hood angle values

    hoodAngleMap.put(1.79, .019);
    hoodAngleMap.put(2.22, 2.5);
    hoodAngleMap.put(2.75, 3.937);
    hoodAngleMap.put(3.129, 4.99);
    

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
    Logger.recordOutput("Hood Angle", io.getPosition());
    SmartDashboard.putBoolean("Trench Able", trenchAble());
    //Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood Current Position", io.getPosition());
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();
    boolean red = switch (alliance) {
      case Red -> true;
      case Blue -> false;
    };
    double distanceToHub = 0d;
    if(red){
      distanceToHub = 
        Math.sqrt(Math.pow(Math.abs(HoodConstants.RED_HUB_X-RobotContainer.drive.getPose().getX()), 2)+
                  Math.pow(Math.abs(HoodConstants.RED_HUB_Y-RobotContainer.drive.getPose().getY()), 2));
    }
    else{
      distanceToHub = 
        Math.sqrt(Math.pow(Math.abs(HoodConstants.BLUE_HUB_X-RobotContainer.drive.getPose().getX()), 2)+
                  Math.pow(Math.abs(HoodConstants.BLUE_HUB_Y-RobotContainer.drive.getPose().getY()), 2));
    }
    Logger.recordOutput("Dist to Hub", distanceToHub);


    try{
      if(autoAlign) {
       targetPosition = hoodAngleMap.get(distanceToHub);
      }
      else{
        targetPosition = 0;
      }
    }
    catch(Exception e){}
    Logger.recordOutput("Hood Target Position", targetPosition);

    double pidMotorSpeed = 
        pidController.calculate(io.getPosition(), targetPosition)
            + feedForward.calculate(targetPosition, 0);
    Logger.recordOutput("Hood Speed", pidMotorSpeed);
      setMotor(
          MathUtil.clamp((pidMotorSpeed), -HoodConstants.MAX_VOLTAGE, HoodConstants.MAX_VOLTAGE));
    
    // setMotor(pidMotorSpeed * HoodConstants.MAX_VOLTAGE);
  
      Logger.recordOutput("Dist to Target", tempDistance);
      Logger.recordOutput("closest id", tempClosest);
  }

  public boolean trenchAble() {
    if(targetPosition > 0) {
      return false;
    }
    return true;
  }

  public void setMotor(double voltage) {
    io.setVoltage(voltage);
  }

  public void setAutoAlign(){
    this.autoAlign = !autoAlign;
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
