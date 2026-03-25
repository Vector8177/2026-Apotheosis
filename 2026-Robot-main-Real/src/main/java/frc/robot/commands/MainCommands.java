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

import frc.robot.subsystems.drive.Drive;

public class MainCommands {

  private MainCommands() {}

  // public static Command track(Turret turret, Vision vision){

  // }

  public static Command rotateHood(Hood hood) {
    return runOnce(() -> hood.setPosition(.1), hood);
  }

  public static Command rotateHoodBackward(Hood hood) {
    return runOnce(() -> hood.setPosition(-.1), hood);
  }

  public static Command stopHood(Hood hood) {
    return runOnce(() -> hood.setPosition(0), hood);
  }


  public static Command turnTurret(Turret turret){
    return runOnce(()-> turret.setPosition(.3), turret);
  }

  public static Command turnTurretBackward(Turret turret){
    return runOnce(()-> turret.setPosition(-.3), turret);
  }

  public static Command stopTurret(Turret turret){
    return runOnce(()-> turret.setPosition(0), turret);
  }
  
  public static Command stow(Turret turret, Hood hood) {
    return sequence(runOnce(() -> turret.setPosition(0), turret),
                    runOnce(() -> hood.setPosition(0), hood));
  }
  
  public static Command stopIntake(Intake intake) {

    return sequence(runOnce(() -> intake.setSpeed(0), intake), 
                    runOnce(() -> intake.setIntakeSpeed(0), intake));
  }

  public static Command toggleAutoAlign(Turret turret){

    return sequence(runOnce(()-> turret.setAutoAlign(), turret));
                    //runOnce( ()-> hood.setAutoAlign(), hood));
  }

  public static Command runOutakeFast(Intake intake) {
    return runOnce(() -> intake.setSpeed(-3), intake);
  }

  public static Command runIntake(Intake intake) {
    return runOnce(() -> intake.setSpeed(.45), intake); // .6
  }

  public static Command runOuttake(Intake intake) {
    return runOnce(() -> intake.setSpeed(-.45), intake); // -.6
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

  public static Command runBottomIntake(Intake intake){
    return runOnce(() -> intake.setIntakeSpeed(.5));
  }

  public static Command runBottomOuttake(Intake intake){
    return runOnce(() -> intake.setIntakeSpeed(-.5));
  }

  // Change Later
  public static Command stopTopIntake(Intake intake) {
    return runOnce(() -> intake.setTopSpeed(0), intake);
  }

  public static Command movePivot(IntakePivot pivot) {
    return runOnce(() -> pivot.setPosition(0.1), pivot);
  }

  public static Command stowPivot(IntakePivot pivot) {
    return runOnce(() -> pivot.setPosition(-0.34), pivot);
  }

  public static Command stopPivot(IntakePivot pivot) {
    return runOnce(() -> pivot.stopPivot(), pivot);
  }

  public static Command shoot(Intake intake, Shooter shooter, Hood hood) {

    return sequence(runOnce(() -> shooter.setSpeed(true), shooter),
                  runOnce( () -> hood.setAutoAlign(true), hood),
                  waitSeconds(.5), 
                  runOnce(() -> intake.setTopSpeed(-.7), intake)); //-.8
    
    
  }

  // public static Command shuttle(Turret turret, Hood hood, Shooter shooter) {
  //   return sequence(runOnce(() -> turret.setShuttlePosition(0), turret),
  //                   runOnce(() -> hood.setShuttlePosition(HoodConstants.SHUTTLE_HOOD), hood),
  //                   runOnce(() -> shooter.setShuttleSpeed(ShooterConstants.SHUTTLE_SPEED), shooter));
  // }

  public static Command shootNoRev(Intake intake, Shooter shooter, Hood hood) {

    return sequence(runOnce(() -> shooter.setSpeed(true), shooter),
                  runOnce(() -> hood.setAutoAlign(true), hood),
                  runOnce(() -> intake.setTopSpeed(-.5), intake));
    
    
  }

  public static Command intakeShoot(Shooter shooter, Intake intake){
    return sequence(runOnce(() -> shooter.setBackSpeed(-70), shooter), 
                    runOnce(() -> intake.setTopSpeed(.5), intake));
  }


  public static Command stopShooter(Intake intake, Shooter shooter, Hood hood){
    return sequence(runOnce(() -> shooter.setSpeed(false), shooter),
                    runOnce(() -> intake.setTopSpeed(0), intake), 
                    runOnce(() -> hood.setAutoAlign(false), hood));
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
  // public static Command stow(Turret turret, Hood hood) {
  //   return sequence(
  //       //runOnce(() -> hood.setPosition(0), hood),
  //       runOnce(() -> turret.setPosition(0), turret));
  // }

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


  // public static Command autoAlign(){
  //   return runOnce(() -> Drive.setAutoAlign(true));
  // }
}