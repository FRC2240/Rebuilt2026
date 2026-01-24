package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
    private TalonFX hopper = new TalonFX(intakeConstants.Intake.HOPPER_ID);
    private TalonFX intake = new TalonFX(intakeConstants.Intake.INTAKE_ID);

    TorqueCurrentFOC req = new TorqueCurrentFOC(0);

    public void extend() {
        hopper.setPosition(intakeConstants.Intake.EXTEND_DIST);

    }

    public void enableIntake() {
        intake.setControl(new VelocityTorqueCurrentFOC(intakeConstants.Intake.INTAKE_SPEED));
    }

    // commands
    public Command extendIntakeCommand() {
        return this.run(() -> extend());
    }

    public Command enableIntakeCommand() {
        return this.runOnce(() -> enableIntake());
    }

    public Command disableIntakeCommand() {
        return this.runOnce(() -> intake.stopMotor());
    }

    public Command reverseIntakeCommand() {
        return this.runOnce(
                () -> intake.setControl(new VelocityTorqueCurrentFOC(intakeConstants.Intake.INTAKE_SPEED.negate())));
    }

}