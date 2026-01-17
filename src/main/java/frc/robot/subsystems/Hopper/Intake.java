package frc.robot.subsystems.Hopper;


import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Current;

public class Intake extends SubsystemBase {
    private TalonFX Hopper = new TalonFX(IntakeConstants.Intake.HOPPER_ID);
    private TalonFX Intake = new TalonFX(IntakeConstants.Intake.INTAKE_ID);

    TorqueCurrentFOC req = new TorqueCurrentFOC(0);

    public void runExtend(Current current) {
        Hopper.setControl(req.withOutput(current));

    }

    public void runIntake(Current current) {
        Intake.setControl(req.withOutput(current));
    }

    // commands
    public Command runExtendCommand(Current current) {
        return this.run(
                () -> {
                    runExtend(current);
                });
    }

    public Command runIntakeCommand(Current current) {
        return this.run(
                () -> {
                    runIntake(current);
                });
    }
}