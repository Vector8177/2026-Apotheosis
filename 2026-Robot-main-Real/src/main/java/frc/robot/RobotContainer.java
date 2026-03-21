// Copyright 2021-2026 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
/// import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.HoodConstants;
// import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.MainCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.IntakePivotIO;
import frc.robot.subsystems.intakePivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOTalonFX;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static Drive drive;
  private final Intake intake;
  private final IntakePivot intakePivot;
//   private final Climber climber;
  private final Hood hood;
  private final Turret turret;
  private final Vision vision;
  private final Shooter shooter;


  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        intake = new Intake(new IntakeIOTalonFX());

        intakePivot = new IntakePivot(new IntakePivotIOTalonFX());

        // climber = new Climber(new ClimberIOTalonFX());

        hood = new Hood(new HoodIOTalonFX());

        turret = new Turret(new TurretIOTalonFX());

        shooter = new Shooter(new ShooterIOTalonFX());


        

        

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // NamedCommands.registerCommand("Auto Align Left", new AutoAlign(drive, false));
        // NamedCommands.registerCommand("Auto Align Right", new AutoAlign(drive, true));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
            /*new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));*/
       NamedCommands.registerCommand("Stop Intake", MainCommands.stopIntake(intake));
    NamedCommands.registerCommand("Run Intake", MainCommands.runBottomIntake(intake));
    NamedCommands.registerCommand("Slow Intake", MainCommands.runIntakeSlow(intake));
    NamedCommands.registerCommand("Run Outake", MainCommands.runBottomOuttake(intake));
    NamedCommands.registerCommand("Run Top Intake", MainCommands.runTopIntake(intake));
    NamedCommands.registerCommand("Stop Top Intake", MainCommands.stopTopIntake(intake));
    NamedCommands.registerCommand("Shoot", MainCommands.shoot(intake, shooter, hood));
    NamedCommands.registerCommand("Stop Shooter", MainCommands.stopShooter(intake, shooter, hood));
    NamedCommands.registerCommand("AutoAlignToggle", MainCommands.toggleAutoAlign(turret));
    NamedCommands.registerCommand("Spin Flywheel", MainCommands.spinShooterFlywheel(shooter));
    NamedCommands.registerCommand("Pivot Down", MainCommands.stowPivot(intakePivot));
    NamedCommands.registerCommand("Pivot Up", MainCommands.movePivot(intakePivot));
    //NamedCommands.registerCommand("Stop Flywheel", MainCommands)

      
            }

      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        intake = new Intake(new IntakeIOTalonFX());

        intakePivot = new IntakePivot(new IntakePivotIOTalonFX());

        // climber = new Climber(new ClimberIOTalonFX());

        hood = new Hood(new HoodIOTalonFX());

        turret = new Turret(new TurretIOTalonFX());

        shooter = new Shooter(new ShooterIOTalonFX());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
      
       NamedCommands.registerCommand("Stop Intake", MainCommands.stopIntake(intake));
    NamedCommands.registerCommand("Run Intake", MainCommands.runBottomIntake(intake));
    NamedCommands.registerCommand("Slow Intake", MainCommands.runIntakeSlow(intake));
    NamedCommands.registerCommand("Run Outake", MainCommands.runBottomOuttake(intake));
    NamedCommands.registerCommand("Run Top Intake", MainCommands.runTopIntake(intake));
    NamedCommands.registerCommand("Stop Top Intake", MainCommands.stopTopIntake(intake));
    NamedCommands.registerCommand("Shoot", MainCommands.shoot(intake, shooter, hood));
    NamedCommands.registerCommand("Stop Shooter", MainCommands.stopShooter(intake, shooter, hood));
    NamedCommands.registerCommand("AutoAlignToggle", MainCommands.toggleAutoAlign(turret));
    NamedCommands.registerCommand("Spin Flywheel", MainCommands.spinShooterFlywheel(shooter));
        NamedCommands.registerCommand("Pivot Down", MainCommands.stowPivot(intakePivot));
            NamedCommands.registerCommand("Pivot Up", MainCommands.movePivot(intakePivot));



      
      }

      default -> {
        intake = new Intake(new IntakeIOTalonFX());

        intakePivot = new IntakePivot(new IntakePivotIOTalonFX());

        // climber = new Climber(new ClimberIOTalonFX());

        hood = new Hood(new HoodIOTalonFX());
        
        turret = new Turret(new TurretIOTalonFX());

        shooter = new Shooter(new ShooterIOTalonFX());


        

    

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
            /*new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));*/
       NamedCommands.registerCommand("Stop Intake", MainCommands.stopIntake(intake));
    NamedCommands.registerCommand("Run Intake", MainCommands.runBottomIntake(intake));
    NamedCommands.registerCommand("Slow Intake", MainCommands.runIntakeSlow(intake));
    NamedCommands.registerCommand("Run Outake", MainCommands.runBottomOuttake(intake));
    NamedCommands.registerCommand("Run Top Intake", MainCommands.runTopIntake(intake));
    NamedCommands.registerCommand("Stop Top Intake", MainCommands.stopTopIntake(intake));
    NamedCommands.registerCommand("Shoot", MainCommands.shoot(intake, shooter, hood));
    NamedCommands.registerCommand("Stop Shooter", MainCommands.stopShooter(intake, shooter, hood));
    NamedCommands.registerCommand("AutoAlignToggle", MainCommands.toggleAutoAlign(turret));
    NamedCommands.registerCommand("Spin Flywheel", MainCommands.spinShooterFlywheel(shooter));
    NamedCommands.registerCommand("Pivot Down", MainCommands.stowPivot(intakePivot));
        NamedCommands.registerCommand("Pivot Up", MainCommands.movePivot(intakePivot));


      }
    }

   
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));


    configureButtonBindings();
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> /*driverController.povUp().getAsBoolean()?5:*/-driverController.getRightX()));
    
   
    driverController.y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    

    driverController
        .x()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY() * .5,
                () -> -driverController.getLeftX() * .5,
                () -> -driverController.getRightX() * .5));
    

    driverController
        .rightTrigger()
        .onTrue(MainCommands.runBottomIntake(intake))
        .onFalse(MainCommands.stopIntake(intake));
    driverController
        .leftTrigger()
        .onTrue(MainCommands.runBottomOuttake(intake))
        .onFalse(MainCommands.stopIntake(intake));

    // driverController.a().onTrue(MainCommands.shoot(shooter));

   
    // driverController.b()
    //     .onTrue(MainCommands.moveHood(hood));

    

    driverController.b().onTrue(MainCommands.toggleAutoAlign(turret));
    // driverController.a().onTrue(MainCommands.spinShooterFlywheel(shooter)).onFalse(MainCommands.stopShooter(intake, shooter, hood));
    driverController.a().onTrue(MainCommands.shootNoRev(intake, shooter, hood)).onFalse(MainCommands.stopShooter(intake, shooter, hood));
    driverController.rightBumper().onTrue(MainCommands.shoot(intake, shooter, hood)).onFalse(MainCommands.stopShooter(intake, shooter, hood));
    // driverController.leftBumper().onTrue(MainCommands.runTopIntakeBackward(intake)).onFalse(MainCommands.stopTopIntake(intake));
    driverController.leftBumper().onTrue(MainCommands.intakeShoot(shooter, intake)).onFalse(MainCommands.stopShooter(intake,shooter, hood));
    driverController.povLeft().onTrue(MainCommands.movePivot(intakePivot));
    driverController.povRight().onTrue(MainCommands.stowPivot(intakePivot));
    driverController.povUp().onTrue(MainCommands.rotateHood(hood)).onFalse(MainCommands.stopHood(hood));
    // driverController.povDown().onTrue(MainCommands.rotateHoodBackward(hood)).onFalse(MainCommands.stopHood(hood));
    driverController.povDown().onTrue(MainCommands.stow(turret, hood));
    // driverController.povUp().onTrue(MainCommands.shuttle(turret, hood, shooter));

    // driverController.povUp().onTrue(MainCo`mmands.autoAlign());
    

    // operatorController
    //     .povRight()
    //     .onTrue(
    //         MainCommands.moveTurret3(turret));
    // operatorController
    //     .povLeft()
    //     .onTrue(
    //         MainCommands.moveTurret1(
    //             turret));
    // operatorController
    //     .povUp()
    //     .onTrue(
    //         MainCommands.moveTurret2(
    //             turret));
    // operatorController
    //     .povDown()
    //     .onTrue(
    //         MainCommands.moveTurret0(turret));



    // operatorController
      //   .rightBumper()
       //  .onTrue(MainCommands.turnTurret(turret))
        // .onFalse(MainCommands.stopTurret(turret));
    // operatorController
      //   .leftBumper()
        //.onTrue(MainCommands.turnTurretBackward(turret))
         //.onFalse(MainCommands.stopTurret(turret));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
