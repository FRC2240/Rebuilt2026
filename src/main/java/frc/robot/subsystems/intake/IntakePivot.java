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

    public IntakePivot() {
        TalonFXConfiguration conf = new TalonFXConfiguration();
        conf.MotionMagic.MotionMagicCruiseVelocity = 10;
        conf.MotionMagic.MotionMagicAcceleration = 16;

        conf.Slot0.kP = 17;
        conf.Slot0.kD = 2;
        conf.Slot0.kI = 2;

        conf.Slot1.kP = 20;
        conf.Slot0.kD = 4;
        conf.Slot1.kI = 10;

        conf.CurrentLimits.SupplyCurrentLimit = 100;
        conf.CurrentLimits.StatorCurrentLimit = 100;

        pivotMotor.getConfigurator().apply(conf);
        pivotMotor.setPosition(Rotations.of(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Pivot Position", pivotMotor.getPosition().getValueAsDouble());
    }

    public Command setPositionCommand(Angle position) {
        return runOnce(() -> pivotMotor.setControl(new PositionTorqueCurrentFOC(position))).andThen(run(() -> {
        }));
    }

    public void extendMotionMagic() {
        pivotMotor.setControl(new MotionMagicTorqueCurrentFOC(IntakeConstants.PIVOT_EXTENDED_POSITION));
        /*
         * double velocityRPS = pivotMotor.getVelocity().getValueAsDouble();
         * double position = pivotMotor.getPosition().getValueAsDouble();
         * if (Math.abs(velocityRPS) < 0.1 && position < -8) {
         * pivotMotor.setPosition(-10);
         * }
         */
    }

    public void extend() {
        pivotMotor.setControl(new PositionTorqueCurrentFOC(IntakeConstants.PIVOT_EXTENDED_POSITION));
        double velocityRPS = pivotMotor.getVelocity().getValueAsDouble();
        double position = pivotMotor.getPosition().getValueAsDouble();
        if (Math.abs(velocityRPS) < 0.1 && position < -8) {
            pivotMotor.setPosition(-10);
        }
    }

    public void ramp() {
        pivotMotor.setControl(new PositionTorqueCurrentFOC(IntakeConstants.PIVOT_RAMP_POSITION).withSlot(1));
    }

    public Command extendCommand() {
        return runOnce(this::extend);
    }

    public Command contractCommand() {
        return runOnce(() -> pivotMotor.setControl(new MotionMagicTorqueCurrentFOC(Rotations.of(0))))
                .andThen(run(() -> {
                }));
    }

    public Command rezeroCommand() {
        return Commands.sequence(
                runOnce(() -> pivotMotor.setControl(new VelocityTorqueCurrentFOC(RotationsPerSecond.of(-10)))),
                Commands.waitSeconds(0.3),
                Commands.waitUntil(() -> Math.abs(pivotMotor.getVelocity().getValueAsDouble()) < 0.2),
                runOnce(() -> pivotMotor.setPosition(Rotations.of(-10))),
                extendCommand());
    }

    public Command rampCommand() {
        return Commands.repeatingSequence(
                runOnce(this::ramp),
                Commands.waitSeconds(0.25),
                runOnce(this::extend),
                Commands.waitSeconds(0.25));
    }
}
