package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

public class MainCommands {

  private MainCommands() {}

  // public static Command track(Turret turret, Vision vision){

  // }

  // public static Command turnTurret(Turret turret){
  //   return runOnce(()-> turret.setSpeed(.3), turret);
  // }

  // public static Command turnTurretBackward(Turret turret){
  //   return runOnce(()-> turret.setSpeed(-.3), turret);
  // }

  // public static Command stopTurret(Turret turret){
  //   return runOnce(()-> turret.setSpeed(.0), turret);}
  
  
  public static Command stopIntake(Intake intake) {

    return runOnce(() -> intake.setSpeed(0), intake);
  }

  public static Command toggleAutoAlign(Turret turret, Hood hood){

    return sequence(runOnce(()-> turret.setAutoAlign(), turret));//, 
                   // runOnce( ()-> hood.setAutoAlign()));
  }

  public static Command runOutakeFast(Intake intake) {
    return runOnce(() -> intake.setSpeed(-3), intake);
  }

  public static Command runIntake(Intake intake) {
    return runOnce(() -> intake.setSpeed(.7), intake); // .3
  }

  public static Command runOuttake(Intake intake) {
    return runOnce(() -> intake.setSpeed(-.7), intake); // -.3
  }

  public static Command runOuttakeSlow(Intake intake) {
    return runOnce(() -> intake.setSpeed(-.05), intake);
  }

  public static Command runIntakeSlow(Intake intake) {
    return runOnce(() -> intake.setSpeed(.05), intake);
  }

  // Change Later
  public static Command runTopIntake(Intake intake) {
    return runOnce(() -> intake.setTopSpeed(-1), intake); // .1
  }

  public static Command runTopIntakeBackward(Intake intake) {
    return runOnce(() -> intake.setTopSpeed(1), intake); // .1
  }

  // Change Later
  public static Command stopTopIntake(Intake intake) {
    return runOnce(() -> intake.setTopSpeed(0), intake);
  }

  public static Command movePivot(IntakePivot pivot) {
    return runOnce(() -> pivot.setPosition(IntakePivotConstants.INTAKE_POSITION), pivot);
  }

  public static Command stowPivot(IntakePivot pivot) {
    return runOnce(() -> pivot.setPosition(IntakePivotConstants.INTAKE_STOW), pivot);
  }

  public static Command shoot(Intake intake, Shooter shooter) {

    return sequence(runOnce(() -> intake.setTopSpeed(-1), intake), 
                   runOnce(() -> shooter.setSpeed(true), shooter));
    
    
  }

  public static Command stopShooter(Intake intake, Shooter shooter){
    return sequence(runOnce(() -> intake.setTopSpeed(0), intake), 
                    runOnce(() -> shooter.setSpeed(false), shooter));
  }

  // public static Command stopShooter(Shooter shooter) {
  //   return runOnce(() -> shooter.setSpeed(), shooter);//.05
  // }

  public static Command moveHood(Hood hood){
    return runOnce(() -> hood.setPosition(10), hood);
  }

  public static Command spinShooterFlywheel(Shooter shooter){
      return runOnce(() -> shooter.setSpeed(true));
   }
  


  // public static Command turretFollow(Turret turret){
  //   return runOnce(()-> turret.followCamera());
      
  //   //return runOnce(() -> turret.followCamera(true), turret);
  // }

  public static Command test(Turret turret){
    return runOnce((() -> turret.move()));
    //should hypothetically move the turret while the button is pressed
    // return run(() -> {
    //   if(LimelightHelpers.getTargetCount("limelight-turret")!=1){
    //     return;
    //   }
    //   if(LimelightHelpers.getTX("limelight-turret")<10){
    //     return;
    //   }
    //   turret.move();
    // }, turret);
  }
  // public static Command stopTurretFollow(Turret turret){
  //   return runOnce(() -> turret.followCamera(false), turret);
  // }


 

  // Shuts hood down and unspools turret
  public static Command stow(Turret turret, Hood hood) {
    return sequence(
        //runOnce(() -> hood.setPosition(0), hood),
        runOnce(() -> turret.setPosition(0), turret));
  }

  public static Command intakePosition(IntakePivot intakePivot){
    return runOnce(() -> intakePivot.setPosition(IntakePivotConstants.INTAKE_POSITION));
  }
  
  public static Command stowIntake(IntakePivot intakePivot){
    return runOnce(() -> intakePivot.setPosition(0));
  }

   // public static Command moveTurret0(Turret turret){
  //   return runOnce(() -> turret.setPosition(TurretConstants.posO), turret);
  // }
  // public static Command moveTurret1(Turret turret){
  //   return runOnce(() -> turret.setPosition(TurretConstants.pos1), turret);
  // }
  // public static Command moveTurret2(Turret turret){
  //   return runOnce(() -> turret.setPosition(TurretConstants.pos2), turret);
  // }
  // public static Command moveTurret3(Turret turret){
  //   return runOnce(() -> turret.setPosition(TurretConstants.pos3), turret);
  // }
}