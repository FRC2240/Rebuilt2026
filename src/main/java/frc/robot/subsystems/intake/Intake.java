package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX hopper = new TalonFX(IntakeConstants.HOPPER_ID);
  private TalonFX intake = new TalonFX(IntakeConstants.INTAKE_ID);

  public Command extendIntakeCommand() {
    return runOnce(() -> hopper.setControl(new PositionTorqueCurrentFOC(IntakeConstants.EXTENDED_POSITION)));
  }

  public Command enableIntakeCommand() {
    return runOnce(() -> intake.setControl(new VelocityTorqueCurrentFOC(IntakeConstants.INTAKE_SPEED)));
  }

  public Command disableIntakeCommand() {
    return runOnce(() -> intake.stopMotor());
  }

  public Command reverseIntakeCommand() {
    return runOnce(() -> intake.setControl(new VelocityTorqueCurrentFOC(IntakeConstants.INTAKE_SPEED.unaryMinus())));
  }
}