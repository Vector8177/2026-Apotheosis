package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

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

import com.ctre.phoenix6.controls.VelocityVoltage;

import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private static final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private double targetSpeed = 0;

  private boolean shoot = false;
  

  static {
    //put values for interpolating tree maps
    //add more values as testing each case continues
    shooterSpeedMap.put(0d,0d);
  }

  public Shooter(ShooterIO io) {
    this.io = io;
    new SlewRateLimiter(.4);
  }

  private void setSpeedRaw(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    
    // Logger.recordOutput("Shooter speed", speed);
    io.setShooterVoltage(speed * ShooterConstants.MAX_VOLTAGE);
  }

  public void setSpeed(boolean shoot) {
    targetSpeed = shoot ? .7 : 0; // .1
  }

  @Override
  public void periodic() {


    io.updateInputs(inputs);

    Logger.recordOutput("Flywheel position", io.getPosition());

    // Logger.processInputs("Shooter", inputs);
    // try{
    //     int closest = (int)LimelightHelpers.getFiducialID("limelight-turret");
    //     LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", new int[]{closest});
    //     RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-turret");
    //     double distance = fiducials[0].distToCamera;
    //     LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", TurretConstants.allId);
    //     //targetSpeed = shooterSpeedMap.get(distance);
    //     setSpeedRaw(targetSpeed);
    //     Logger.recordOutput("target_speed", targetSpeed);
    // }
    // catch(Exception e){}
    setSpeedRaw(targetSpeed);
  }

  public void stop() {
    io.stop();
  }
}
