package frc.robot.subsystems.Intake;


import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Current;

public class Intake extends SubsystemBase {
    private TalonFX Hopper = new TalonFX(IntakeConstants.Intake.HOPPER_ID);
    private TalonFX Intake = new TalonFX(IntakeConstants.Intake.INTAKE_ID);

    TorqueCurrentFOC req = new TorqueCurrentFOC(0);

    public void Extend(Current current) {
        Hopper.setControl(req.withOutput(current));

    }

    public void RunIntake(Current current) {
        Intake.setControl(req.withOutput(current));
    }

    // commands
    public Command ExtendCommand(Current current) {
        return this.run(
                () -> {
                    Extend(current);
                });
    }

    public Command IntakeCommand(Current current) {
        return this.run(
                () -> {
                    RunIntake(current);
                });
    }
}
