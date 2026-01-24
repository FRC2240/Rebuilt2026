package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.units.measure.Current;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class Climber extends SubsystemBase {
  TalonFX motor = new TalonFX(ClimberConstants.MOTOR_ID);
  TorqueCurrentFOC req = new TorqueCurrentFOC(0);
  private final TalonFXConfiguration conf;

  Mechanism2d climbRoot = new Mechanism2d(27, 27);
  MechanismRoot2d climbBase = climbRoot.getRoot("ClimbBase", 2.25, 14.5);
  MechanismLigament2d climberSim = climbBase.append(new MechanismLigament2d("ClimbArm", 10, 90));

  public Climber() {
    conf = new TalonFXConfiguration();

    conf.Slot0.kP = 0;
    conf.Slot0.kD = 0;
    conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(conf);

    SmartDashboard.putData("Mech2d", climbRoot);
  }

  public Command extendCommand() {
    return this.run(() -> motor.setControl(req.withOutput(ClimberConstants.EXTEND_CURRENT)));
  }

  public void extendsim(Double length) {
    climberSim.setLength(length);
  }

  public Command extendCommandsim(Double length) {
    return this.run(() -> extendsim(length));
  }
}
