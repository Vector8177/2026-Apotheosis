package frc.robot.subsystems.intakePivot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  private final TalonFX intakePivotMotor;
  private final CANcoder encoder;
  private final TalonFXConfiguration configuration;
  private final CANcoderConfiguration config;
  // private final DigitalInput input;
  // private final DutyCycleEncoder encoder;

  public IntakePivotIOTalonFX() {
    intakePivotMotor = new TalonFX(IntakePivotConstants.MOTOR_ID);
    encoder = new CANcoder(IntakePivotConstants.ENCODER_ID);
    configuration = new TalonFXConfiguration();
    config = new CANcoderConfiguration();

    intakePivotMotor.getConfigurator().apply(configuration, .05);
    intakePivotMotor.setPosition(0);
    //intakePivotMotor.setControl(null);
    intakePivotMotor.setNeutralMode(NeutralModeValue.Brake);
    
    

    

    // input = new DigitalInput(0);
    // encoder = new DutyCycleEncoder(input);
    // // encoder.setDutyCycleRange(0.01, .99);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.intakePivotAppliedVolts =
        intakePivotMotor.getDutyCycle().getValueAsDouble()
            * intakePivotMotor.getSupplyVoltage().getValueAsDouble();
    inputs.intakePivotVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(intakePivotMotor.getVelocity().getValueAsDouble());
    // inputs.intakePivotAbsoluteEncoderPosition = encoder.get();
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Voltage Intake Pivot", voltage);
    intakePivotMotor.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return intakePivotMotor.getPosition().getValueAsDouble();
    // if (encoder.get() > .75) {
    //   return .6;
    // }
    // return encoder.get();

  }
   @Override
  public double getEncoderPosition(){
    return encoder.getAbsolutePosition().getValueAsDouble();
  }



  @Override
  public void setPosition(double position) {
    intakePivotMotor.setPosition(position);
  }

 
}
