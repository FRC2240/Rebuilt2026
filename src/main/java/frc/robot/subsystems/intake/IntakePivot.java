package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
    private TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
    private String currentState = "None";

    public IntakePivot() {
        TalonFXConfiguration conf = new TalonFXConfiguration();
        conf.MotionMagic.MotionMagicCruiseVelocity = 10;
        conf.MotionMagic.MotionMagicAcceleration = 16;

        conf.Slot0.kP = 17;
        conf.Slot0.kD = 2;
        conf.Slot0.kI = 2;

        conf.Slot1.kP = 20;
        conf.Slot1.kD = 4;
        conf.Slot1.kI = 10;

        // Slot 2 has a small PID for rezeroing
        conf.Slot2.kP = 5;

        conf.CurrentLimits.SupplyCurrentLimit = 100;
        conf.CurrentLimits.StatorCurrentLimit = 100;

        pivotMotor.getConfigurator().apply(conf);
        pivotMotor.setPosition(Rotations.of(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Pivot Position", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putString("Intake/Pivot State", currentState);
        SmartDashboard.putString("Intake/Current Pivot Command", getCurrentCommand() == null ? "None" : getCurrentCommand().getName());
    }

    public Command setPositionCommand(Angle position) {
        return runOnce(() -> pivotMotor.setControl(new PositionTorqueCurrentFOC(position))).andThen(run(() -> {
        }));
    }

    public void extendMotionMagic() {
        pivotMotor.setControl(new MotionMagicTorqueCurrentFOC(IntakeConstants.PIVOT_EXTENDED_POSITION));
    }

    public void extend() {
        currentState = "Extend";
        pivotMotor.setControl(new PositionTorqueCurrentFOC(IntakeConstants.PIVOT_EXTENDED_POSITION));
    }

    public void ramp() {
        currentState = "Ramp";
        pivotMotor.setControl(new PositionTorqueCurrentFOC(IntakeConstants.PIVOT_RAMP_POSITION).withSlot(1));
    }

    public Command extendCommand() {
        return runOnce(this::extend).withName("Extend");
    }

    public Command contractCommand() {
        return runOnce(() -> {
            pivotMotor.setControl(new MotionMagicTorqueCurrentFOC(Rotations.of(0)));
            currentState = "Contract";
        })
                .andThen(run(() -> {
                })).withName("Contract");
    }

    public Command rezeroCommand() {
        return Commands.sequence(
                runOnce(() -> pivotMotor.setControl(new VelocityTorqueCurrentFOC(RotationsPerSecond.of(-2)).withSlot(2))),
                Commands.waitSeconds(0.3),
                Commands.waitUntil(() -> Math.abs(pivotMotor.getVelocity().getValueAsDouble()) < 0.2),
                runOnce(() -> pivotMotor.setPosition(Rotations.of(-10))),
                extendCommand())
                .withName("Rezero");
    }

    public Command rampCommand() {
        return Commands.repeatingSequence(
                runOnce(this::ramp),
                Commands.waitSeconds(0.25),
                runOnce(this::extend),
                Commands.waitSeconds(0.25))
                .finallyDo(this::extend)
                .withName("Ramp");
    }
}
