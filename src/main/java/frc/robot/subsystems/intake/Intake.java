package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private TalonFX hopperMotor = new TalonFX(IntakeConstants.HOPPER_ID);
    private TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_ID);

    public Intake() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 1;

        hopperMotor.getConfigurator().apply(conf);
        intakeMotor.getConfigurator().apply(conf);
    }

    public void spin(AngularVelocity speed) {
        intakeMotor.setControl(new VelocityTorqueCurrentFOC(speed));
    }

    public Command extendIntakeCommand() {
        return runOnce(() -> hopperMotor.setControl(new PositionTorqueCurrentFOC(IntakeConstants.EXTENDED_POSITION)));
    }

    public Command enableIntakeCommand() {
        return run(() -> spin(IntakeConstants.INTAKE_SPEED));
    }

    public Command disableIntakeCommand() {
        return run(() -> spin(RotationsPerSecond.of(0)));
    }

    public Command reverseIntakeCommand() {
        return runOnce(
                () -> intakeMotor.setControl(new VelocityTorqueCurrentFOC(IntakeConstants.INTAKE_SPEED.unaryMinus())));
    }
}