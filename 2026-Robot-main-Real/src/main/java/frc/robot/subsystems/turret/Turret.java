package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;

public class Turret extends SubsystemBase {
  private final PIDController pidController;
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private double targetPosition;
  private ArmFeedforward feedForward;
  private boolean follow;
  private DoubleSupplier gyro;
  private boolean autoAlign;
  double delH = -1;
    double theta1 = -1;
        double theta2 = -1;
        double L1 = -1;
        double L2 = -1;
        double beta = -1;
        double betap = -1;
        double phi = -1;
        double phip = -1;
        double thetap1 = -1;
        double thetap2 = -1;
        double Sx = -1;
        double Sy = -1;
        double S = -1;
        double theta = -1;
        double y1 = -1;
        double y2 = -1;
        int casee = -1;
        double d1 = -1;
        double d2 = -1;


        int time = 1;

  public Turret(TurretIO io) {

    this.io = io;
    pidController = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
    pidController.setTolerance(.2);
    follow = false;
    gyro = null;
    feedForward =
        new ArmFeedforward(
            TurretConstants.kS, TurretConstants.kG, TurretConstants.kV, TurretConstants.kA);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", TurretConstants.allId);
    }
    

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    time++;

    

    if(time%3==0 && time>=50){
      int id = (int) LimelightHelpers.getFiducialID("limelight-turret");
      switch(id){
        case 2 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
        }
        case 3 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0.3556, 0);
        }
        case 4 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
        }
        case 5 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
        }
        case 8 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, -0.3556, 0);
        }
        case 9 -> {
           LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0.3556, 0);
        }
        case 10 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
        }
        case 11 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0.3556, 0);
        }
        case 18 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
        }
        case 19 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0.3556, 0);
        }
        case 20 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
        }
        case 21 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
        }
        case 24 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, -0.3556, 0);
        }
        case 25 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0.3556, 0);
        }
        case 26 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0, 0);
        }
        case 27 -> {
          LimelightHelpers.SetFidcuial3DOffset("limelight-turret", -0.603758, 0.3556, 0);
        }
        default -> {
          break;
        }

      }
      
      if(LimelightHelpers.getTV("limelight-turret") && !DriverStation.isDisabled() && autoAlign){
        move();
      }
      time = 51;
    }
    
    //MathUtil.clamp(targetPosition, -2.5, 2.5);

    double pidMotorSpeed =
        pidController.calculate(io.getPosition(), targetPosition)
            + feedForward.calculate(targetPosition, 0);
      setMotor(
        MathUtil.clamp((pidMotorSpeed), -TurretConstants.MAX_VOLTAGE, TurretConstants.MAX_VOLTAGE));
      
      
      
      
      
      Logger.recordOutput("Theta1", theta1);
      Logger.recordOutput("Theta2", theta2);
      Logger.recordOutput("DelH", delH);
      Logger.recordOutput("beta", beta);
      Logger.recordOutput("betap", betap);
      Logger.recordOutput("S", S);
      Logger.recordOutput("Sx", Sx);
      Logger.recordOutput("Sy", Sy);
      Logger.recordOutput("L1", L1);
      Logger.recordOutput("L2", L2);
      Logger.recordOutput("thetap1", thetap1);
      Logger.recordOutput("thetap2", thetap2);
      Logger.recordOutput("Turret Current Position", io.getPosition());
      Logger.recordOutput("Turret Target Position", targetPosition);
      Logger.recordOutput("Phi", phi);
      Logger.recordOutput("TY - CM", y1);
      Logger.recordOutput("TY - Left", y2);
      Logger.recordOutput("Case", casee);
      Logger.recordOutput("PhiP", phip);
      Logger.recordOutput("d1", d1);
      Logger.recordOutput("d2", d2);
    }

  public void setMotor(double voltage) {
    io.setVoltage(voltage);
  }

  public void setPosition(double position) {
    Logger.recordOutput("Turret Target Position", position);
    System.out.println(io.getPosition() + " Turret");
    targetPosition = position;
  }

  public void setTurretSetpoint(double offset) {
    targetPosition = (io.getPosition() + offset);
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void setAutoAlign(){
    this.autoAlign = !this.autoAlign;
  }

  public void followGyro(boolean follow, DoubleSupplier gyro){
    this.follow = follow;
    this.gyro = gyro;
    if(follow && LimelightHelpers.getTV("limelight-turret") && gyro != null){
      double distance = (TurretConstants.TAG_HEIGHT_METERS-TurretConstants.LIMELIGHT_HEIGHT_METERS)/
        (Math.tan(Math.toRadians(TurretConstants.LIMELIGHT_ANGLE-LimelightHelpers.getTY("limelight-turret"))));
      System.out.println(distance);
      double angle = -io.getPosition()-gyro.getAsDouble()-LimelightHelpers.getTX("limelight-turret");
      double x = Math.sqrt(Math.pow(TurretConstants.DISTANCE_TAG_TO_TARGET, 2)+Math.pow(distance, 2)
                -2*TurretConstants.DISTANCE_TAG_TO_TARGET*distance*Math.cos(Math.toRadians(180-angle)));
      


      double theta = Math.asin((TurretConstants.DISTANCE_TAG_TO_TARGET*Math.sin(Math.toRadians(180-angle)))/x)
        +LimelightHelpers.getTX("limelight-turret");
      //setTurretSetpoint(theta/360.0);
      
      //setTurretSetpoint(LimelightHelpers.getTX("limelight-right"));
    }
  
    // Logger.recordOutput("Wrist Speed", pidMotorSpeed);
    // setMotor(
    //     MathUtil.clamp((pidMotorSpeed), -TurretConstants.MAX_VOLTAGE, TurretConstants.MAX_VOLTAGE));
  }

  public void followCamera(){
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", TurretConstants.cIds[0]);
    double theta = 0d;
    double delH = TurretConstants.TAG_HEIGHT_METERS-TurretConstants.LIMELIGHT_HEIGHT_METERS;
        double theta1 = 0d;
        double theta2 = 0d;
        double L1 = 0d;
        double L2 = 0d;
        double beta = 0d;
        double betap = 0d;
        double phi = 0d;
        double thetap1 = 0d;
        double thetap2 = 0d;
        double Sx = 0d;
        double Sy = 0d;
        double S = 0d;
    if(LimelightHelpers.getTargetCount("limelight-turret")==2){
        
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-turret");
        for(RawFiducial f : fiducials){
          int id = f.id;
          if(id==TurretConstants.cIds[0][0]){
            this.y1 = f.tync;
            theta1 = f.txnc;

            d1 = f.distToCamera;
            L1 = (delH)/(Math.tan(Math.toRadians(TurretConstants.LIMELIGHT_ANGLE+y1)));
          }
          else{
            this.y2 = f.tync;
            theta2 = f.txnc;
            d2 = f.distToCamera; 
            L2 = (delH)/(Math.tan(Math.toRadians(TurretConstants.LIMELIGHT_ANGLE+y2)));
          }
        }
        beta = Math.toDegrees(Math.asin((L2*Math.sin(Math.toRadians((theta1-theta2))) / TurretConstants.DISTANCE_BETWEEN_TAGS)));
        betap = Math.toDegrees(Math.acos((L2*L2-L1*L1-Math.pow(TurretConstants.DISTANCE_BETWEEN_TAGS, 2))/(-2*(L1)*(TurretConstants.DISTANCE_BETWEEN_TAGS))));
        phip = Math.toDegrees(Math.asin((L1*Math.sin(Math.toRadians((theta1-theta2))) / TurretConstants.DISTANCE_BETWEEN_TAGS)));
        phi = 180-beta-(theta1-theta2);

        if(phi>90 && beta<90){
          thetap1 = 90 - beta;
          thetap2 = -Math.abs(theta2 - theta1) + thetap1; //Check
          casee = 1;
        }
        else if(phi<90 && beta>90){
          thetap1 = beta-90;
          thetap2 = -Math.abs(theta1-theta2)+thetap1;
          thetap1 = -thetap1;
          thetap2 = -thetap2;
          casee = 3;
        }
        // else{
        //   thetap1 = 90-beta;
        //   thetap2 = Math.abs(theta1)+theta2-thetap1;
        //   casee = 2;
        // }



        Sx = L2 * Math.sin(Math.toRadians(thetap2)) + TurretConstants.DISTANCE_TAG_SIDE_TO_TARGET * Math.sin(Math.toRadians(90-TurretConstants.alpha));
        Sy = L2 * Math.cos(Math.toRadians(thetap2)) + TurretConstants.DISTANCE_TAG_SIDE_TO_TARGET * Math.cos(Math.toRadians(90-TurretConstants.alpha));
        theta = Math.abs((Math.toDegrees((Math.atan(Sx / Sy))))); // maybe abs
        double oldTheta = theta;
        //theta += -thetap1 + Math.abs(theta1);


        if(theta1>0 && theta2>0){
          if(beta<90 && phi>90){
            theta += Math.abs(-Math.abs(thetap1) + Math.abs(theta1));
          }
          else{
            theta = Math.abs(Math.abs(thetap1)+theta1-Math.abs(theta));
          }
        }
        else if(theta1<0 && theta2<0){
          if(beta>90 && phi<90){
            theta += Math.abs(-Math.abs(thetap1) + Math.abs(theta1));
          }
          else{
            theta = Math.abs(Math.abs(theta1)+Math.abs(thetap1)-Math.abs(theta));
          }
        }
        
        // if((theta1<0 && theta2<0 || theta1>0 && theta2>0)){
        //   theta += Math.abs(-thetap1 + Math.abs(theta1));
        // }
        // else{
        //   theta += theta2-thetap2;
        // }
        S = Math.sqrt(Math.pow(Sx, 2) + Math.pow(Sy, 2));
        // this.x = L1;
        if(theta1<0 && theta2<0){
          theta = -theta;
          System.out.println("somethingcrazythatwouldn'totherwiseoccur");
        }

        // if(oldTheta<theta2-thetap2){
        //   theta = -theta;
        // }
        targetPosition = io.getPosition()+(theta/36);





        //if(2*theta1-theta!=thetap1){
        //   theta = -theta;
        // }
        // theat = theta;
        // theat2 = theta2;
        
        //System.out.println("Theta: "+(theta/10));
        // this.L1 = L1;
        // this.L2 = L2;
        //final double t = theta;
        
        
        //double L1 = (delH)/(Math.tan(TurretConstants.LIMELIGHT_ANGLE+LimelightHelpers.get))
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", TurretConstants.allId);
        // if((int)theta / Integer.MAX_VALUE!=0){
        //     throw new RuntimeException();
        // }
      }
    this.theta = theta;
    this.delH = delH;
    this.L1 = L1;
    this.L2 = L2;
    this.S = S;
    this.Sx = Sx;
    this.Sy = Sy;
    this.beta = beta;
    this.betap = betap;
    this.phi = phi;
    this.theta1 = theta1;
    this.theta2 = theta2;
    this.thetap1 = thetap1;
    this.thetap2 = thetap2;
    
  } 
  public void wrap(){
    if(targetPosition > TurretConstants.MAX_POSITION){
      targetPosition = targetPosition - 4.6 * 2;
    }
    else if(targetPosition < -TurretConstants.MAX_POSITION){
      targetPosition = targetPosition + 4.6 * 2;
    }
  }
  public void move(){
    
    double x = LimelightHelpers.getTX("limelight-turret");
    targetPosition = targetPosition+x/36;
    wrap();
    this.theta = x/36;
  }
}