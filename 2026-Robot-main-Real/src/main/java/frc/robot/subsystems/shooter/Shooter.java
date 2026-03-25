package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import frc.robot.Constants.HoodConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.VelocityVoltage;

import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private static final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private double targetSpeed = 0;
  private double distance2Hub = 0d;

  private int time = 1;

  private boolean shootAble = false;
  private boolean shuttle = false;
  

  static {
    //put values for interpolating tree maps
    //add more values as testing each case continues
    shooterSpeedMap.put(2d, 80d);
    shooterSpeedMap.put(3.25, 85d);
    shooterSpeedMap.put(3.6, 90d);//97
    shooterSpeedMap.put(4.28, 95d);//107
    shooterSpeedMap.put(4.83, 105d);//115
  }

  public Shooter(ShooterIO io) {
    this.io = io;
    new SlewRateLimiter(.4);
  }

  private void setSpeedRaw(double speed) {
    // speed = MathUtil.clamp(speed, -12, 12);
    
    // Logger.recordOutput("Shooter speed", speed);
    io.setShooterVoltage(speed);
  }

  public void setSpeed(boolean shoot) {
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
    distance2Hub = distanceToHub;
    Logger.recordOutput("Dist to Hub", distanceToHub);
    double speed = 0;
    try {
      speed = shooterSpeedMap.get(distanceToHub);
    }
    catch(Exception e) {
      speed = 90;
    }
    targetSpeed = shoot ? speed : 0.5; // .8
  }

  public void setBackSpeed(double speed){
    targetSpeed = speed;
  }

  public void setShuttleSpeed(double speed){
    if(!shuttle) {
      shuttle = !shuttle;
      targetSpeed = speed;
    }
    else {
      shuttle = !shuttle;
      targetSpeed = 0;
    }
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Shooter Current", io.getCurrent());
    SmartDashboard.putBoolean("Shootable", distance2Hub < 2.5);
    // time++;

    // if(time%3==0 && time>=50){
    //   int id = (int) LimelightHelpers.getFiducialID("limelight-turret");
    //   switch(id){
    //     case 2 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", 0, 0, 0);//-.603758, 0, 0
    //     }
    //     case 3 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0.3556, 0);
    //     }
    //     case 4 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
    //     }
    //     case 5 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
    //     }
    //     case 8 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, -0.3556, 0);
    //     }
    //     case 9 -> {
    //        LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0.3556, 0);
    //     }
    //     case 10 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
    //     }
    //     case 11 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0.3556, 0);
    //     }
    //     case 18 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
    //     }
    //     case 19 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0.3556, 0);
    //     }
    //     case 20 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
    //     }
    //     case 21 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
    //     }
    //     case 24 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, -0.3556, 0);
    //     }
    //     case 25 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0.3556, 0);
    //     }
    //     case 26 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
    //     }
    //     case 27 -> {
    //       LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0.3556, 0);
    //     }
    //     default -> {
    //       break;
    //     }
    //   }
    //   time = 51;
    // }

    // io.updateInputs(inputs);

    // Logger.recordOutput("Flywheel position", io.getPosition());

    // Logger.processInputs("Shooter", inputs);

    // double limelightDistance = -1;
    // double trigDistance = -1;

    // int test = 0;

    // // try{
    // //     int closest = (int)LimelightHelpers.getFiducialID("limelight-turret");
    // //     LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", new int[]{closest});
    // //     RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-turret");

    // //     limelightDistance = fiducials[0].distToCamera;
    // //     trigDistance = (TurretConstants.TAG_HEIGHT_METERS-TurretConstants.LIMELIGHT_HEIGHT_METERS)/
    // //     (Math.tan(Math.toRadians(TurretConstants.LIMELIGHT_ANGLE-LimelightHelpers.getTY("limelight-turret"))));

    // //     LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", TurretConstants.allId);
    // //     //targetSpeed = shooterSpeedMap.get(trigDistance);
    // //     //setSpeedRaw(targetSpeed);//targetSpeed
    // //     //Logger.recordOutput("target_speed", targetSpeed);
    // //     test = 1;
    // // }
    // // catch(Exception e){
    // //   // Logger.recordOutput("Test", 0);
    // // }
    // try{
    //   // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", new int[]{2});
    //   // limelightDistance = LimelightHelpers.getRawFiducials("limelight-turret")[0].distToCamera;
    // trigDistance = (TurretConstants.TAG_HEIGHT_METERS-TurretConstants.LIMELIGHT_HEIGHT_METERS)/
    //      (Math.tan(Math.toRadians(TurretConstants.LIMELIGHT_ANGLE-LimelightHelpers.getTY("limelight-turret"))));

    // }
    // catch(Exception e){

    // }
    
    //   Logger.recordOutput("Test", test);

    // Logger.recordOutput("Distance_by_trig", trigDistance);
    // Logger.recordOutput("Distance_by_limelight", limelightDistance);
    setSpeedRaw(targetSpeed);

  }

  public void stop() {
    io.stop();
  }
}
