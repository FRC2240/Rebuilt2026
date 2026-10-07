package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TunableNumber;

public class IntakePivot extends SubsystemBase {
    private TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
    private String currentState = "None";
    private CoastOut coast = new CoastOut();

    /*
    private TunableNumber extensionP = new TunableNumber("Intake/PID/extension P", 25);
    private TunableNumber extensionI = new TunableNumber("Intake/PID/extension I", 7);
    private TunableNumber extensionD = new TunableNumber("Intake/PID/extension D", 10);

    private TunableNumber retractP = new TunableNumber("Intake/PID/retract P", 40);
    private TunableNumber retractI = new TunableNumber("Intake/PID/retract I", 0);
    private TunableNumber retractD = new TunableNumber("Intake/PID/retract D", 0);
     */


    private TunableNumber extensionP = new TunableNumber("Intake/PID/extension P", 10);
    private TunableNumber extensionI = new TunableNumber("Intake/PID/extension I", 0);
    private TunableNumber extensionD = new TunableNumber("Intake/PID/extension D", 0.4);

    private TunableNumber retractP = new TunableNumber("Intake/PID/retract P", 11);
    private TunableNumber retractI = new TunableNumber("Intake/PID/retract I", 0);
    private TunableNumber retractD = new TunableNumber("Intake/PID/retract D", 1);

    public IntakePivot() {
        configureMotors();
        extensionP.addChangeListener((v) -> configureMotors());
        extensionI.addChangeListener((v) -> configureMotors());
        extensionD.addChangeListener((v) -> configureMotors());

        retractP.addChangeListener((v) -> configureMotors());
        retractI.addChangeListener((v) -> configureMotors());
        retractD.addChangeListener((v) -> configureMotors());

        pivotMotor.setPosition(Rotations.of(0));
    }

    private void configureMotors() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        // For extension
        conf.Slot0.kP = extensionP.get();
        conf.Slot0.kD = extensionD.get();
        conf.Slot0.kI = extensionI.get();

        // Slot 1 is for contracting in the ramp command
        conf.Slot1.kP = retractP.get();
        conf.Slot1.kD = retractD.get();
        conf.Slot1.kI = retractI.get();

        // Slot 2 has a small PID for rezeroing
        conf.Slot2.kP = 5;

        conf.CurrentLimits.SupplyCurrentLimit = 45;
        //conf.CurrentLimits.StatorCurrentLimit = 80;

        pivotMotor.getConfigurator().apply(conf);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Pivot Position", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putString("Intake/Pivot State", currentState);
        SmartDashboard.putNumber("Intake/Pivot Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot Supply Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
    }

    public void extend() {
        currentState = "Extend";
        pivotMotor.setControl(new PositionTorqueCurrentFOC(IntakeConstants.PIVOT_EXTENDED_POSITION));
    }

    public void ramp() {
        currentState = "Ramp";
        pivotMotor.setControl(new PositionTorqueCurrentFOC(IntakeConstants.PIVOT_RAMP_POSITION).withSlot(1));
    }

    public void rampBottom() {
        currentState = "Ramp Bottom";
        pivotMotor.setControl(new PositionTorqueCurrentFOC(IntakeConstants.PIVOT_RAMP_BOTTOM_POSITION));
    }

    public void coast() {
        currentState = "Coast";
        pivotMotor.setControl(coast);
    }

    public Command extendCommand() {
        return runOnce(this::extend).withName("Extend");
    }

    public Command contractCommand() {
        return runOnce(() -> {
            pivotMotor.setControl(new PositionTorqueCurrentFOC(Rotations.of(0)).withSlot(1));
            currentState = "Contract";
        })
                .andThen(run(() -> {
                })).withName("Contract");
    }

    public Command rezeroCommand() {
        return Commands.sequence(
                runOnce(() -> pivotMotor
                        .setControl(new VelocityTorqueCurrentFOC(RotationsPerSecond.of(-2)).withSlot(2))),
                Commands.waitSeconds(0.3),
                Commands.waitUntil(() -> Math.abs(pivotMotor.getVelocity().getValueAsDouble()) < 0.2),
                runOnce(() -> pivotMotor.setPosition(IntakeConstants.PIVOT_EXTENDED_POSITION)),
                extendCommand())
                .withName("Rezero");
    }

    public Command rampCommand() {
        return Commands.repeatingSequence(
             runOnce(this::ramp),
             Commands.waitSeconds(1.1),
             runOnce(this::extend),
             Commands.waitSeconds(1.1)
        );
    }
}