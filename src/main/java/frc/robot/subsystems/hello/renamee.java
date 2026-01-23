package frc.robot.subsystems.hello;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class renamee extends SubsystemBase {
    private TalonFX hopper = new TalonFX(rename.Intake.renamee);
    private TalonFX intake = new TalonFX(rename.Intake.renamee);

    TorqueCurrentFOC req = new TorqueCurrentFOC(0);

    public void extend() {
        hopper.setPosition(rename.Intake.renamee);

    }

    public void enableIntake() {
        intake.setControl(new VelocityTorqueCurrentFOC(rename.Intake.renamee));
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
                () -> intake.setControl(new VelocityTorqueCurrentFOC(rename.Intake.renamee.negate())));
    }

}