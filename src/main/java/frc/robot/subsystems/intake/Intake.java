package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotPosition;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    private TalonFX intakeFollower = new TalonFX(IntakeConstants.INTAKE_MOTOR_FOLLOWER_ID);

    public final IntakePivot pivot = new IntakePivot();

    private String state = "None";

    public Intake() {

        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 8;

        conf.CurrentLimits.SupplyCurrentLimit = 35;
        //conf.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT.in(Amps);

        intakeMotor.getConfigurator().apply(conf);

        intakeFollower.setControl(new Follower(IntakeConstants.INTAKE_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    public void setVelocity(AngularVelocity velocity) {
        intakeMotor.setControl(new VelocityTorqueCurrentFOC(velocity));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Roller Velocity", intakeMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putString("Intake/Roller state", state);
        SmartDashboard.putNumber("Intake/Roller Supply", intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Stator", intakeMotor.getStatorCurrent().getValueAsDouble());
    }

    public Command setIntakeVelocityCommand(AngularVelocity velocity) {
        return runOnce(() -> setVelocity(velocity));
    }

    public Command setIntakeVelocityCommand(Supplier<AngularVelocity> velocity) {
        return runOnce(() -> setVelocity(velocity.get()));
    }

    public Command enableIntakeCommand() {
        return Commands.repeatingSequence(
                run(() -> {
                    double MAX_ROBOT_SPEED = 4.5;
                    double MIN_ROLLER_SPEED = 50;
                    double MAX_ROLLER_SPEED = 90;
                    ChassisSpeeds speeds = RobotPosition.getChassisSpeeds();


                    double speed = Math
                            .sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
                    speed = Math.min(speed, MAX_ROBOT_SPEED); // Caps to 4.5 m/s
                    double fraction = speed / MAX_ROBOT_SPEED;
                    double desiredRollerSpeed = MIN_ROLLER_SPEED + (MAX_ROLLER_SPEED - MIN_ROLLER_SPEED) * fraction;
                    SmartDashboard.putNumber("Intake/Desired Roller Speed", -desiredRollerSpeed);

                    intakeMotor.setControl(new VelocityTorqueCurrentFOC(RotationsPerSecond.of(-desiredRollerSpeed)));
                }));
    }

    public Command disableIntakeCommand() {
        return setIntakeVelocityCommand(RotationsPerSecond.of(0)).andThen(run(() -> {
            state = "Disable";
        })).withName("Disable");
    }

    public Command reverseIntakeCommand() {
        return setIntakeVelocityCommand(IntakeConstants.INTAKE_VELOCITY.unaryMinus()).andThen(run(() -> {
            state = "Reverse";
        })).withName("Reverse");
    }

    public Command slowReverseIntakeCommand() {
        return setIntakeVelocityCommand(RotationsPerSecond.of(1)).andThen(run(() -> {
            state = "slow";
        }));
    }
}