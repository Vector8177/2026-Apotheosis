package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.TurretConstants;;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor;
  private final TalonFXConfiguration configuration;
  private final CANBus canbus = new CANBus("Turret Canivore");
  // private final DigitalInput input;
  // private final DutyCycleEncoder encoder;

  public TurretIOTalonFX() {
    turretMotor = new TalonFX(TurretConstants.MOTOR_ID, canbus);
    configuration = new TalonFXConfiguration();

    turretMotor.getConfigurator().apply(configuration, .05);
    turretMotor.setPosition(0);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);

    // input = new DigitalInput(0);
    // encoder = new DutyCycleEncoder(input);
    // // encoder.setDutyCycleRange(0.01, .99);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretAppliedVolts =
        turretMotor.getDutyCycle().getValueAsDouble()
            * turretMotor.getSupplyVoltage().getValueAsDouble();
    inputs.turretVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turretMotor.getVelocity().getValueAsDouble());
    // inputs.wristAbsoluteEncoderPosition = encoder.get();
  }

  @Override
  public void setVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return turretMotor.getPosition().getValueAsDouble();
    // if (encoder.get() > .75) {
    //   return .6;
    // }
    // return encoder.get();

  }

  @Override
  public void setPosition(double position) {
    turretMotor.setPosition(position);
  }
  
}
