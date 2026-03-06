package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.HoodConstants;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor;
  private final TalonFXConfiguration configuration;
  // private final DigitalInput input;
  // private final DutyCycleEncoder encoder;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(HoodConstants.MOTOR_ID);
    configuration = new TalonFXConfiguration();

    hoodMotor.getConfigurator().apply(configuration, .05);
    hoodMotor.setPosition(0);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    // input = new DigitalInput(0);
    // encoder = new DutyCycleEncoder(input);
    // // encoder.setDutyCycleRange(0.01, .99);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodAppliedVolts =
        hoodMotor.getDutyCycle().getValueAsDouble()
            * hoodMotor.getSupplyVoltage().getValueAsDouble();
    inputs.hoodVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(hoodMotor.getVelocity().getValueAsDouble());
    // inputs.hoodAbsoluteEncoderPosition = encoder.get();
  }

  @Override
  public void setVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
    // if (encoder.get() > .75) {
    //   return .6;
    // }
    // return encoder.get();

  }

  @Override
  public void setPosition(double position) {
    hoodMotor.setPosition(position);
  }
}
