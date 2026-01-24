package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Field;

public class Shooter extends SubsystemBase {

    private TalonFX flywheelLeftMotor = new TalonFX(ShooterConstants.FLYWHEEL_LEFT_MOTOR_ID);
    private TalonFX flywheelRightMotor = new TalonFX(ShooterConstants.FLYWHEEL_RIGHT_MOTOR_ID);
    private TalonFX launchLeftMotor = new TalonFX(ShooterConstants.LAUNCH_LEFT_MOTOR_ID);
    private TalonFX launchRightMotor = new TalonFX(ShooterConstants.LAUNCH_RIGHT_MOTOR_ID);

    private VelocityTorqueCurrentFOC req = new VelocityTorqueCurrentFOC(0);

    private AngularVelocity currSpeed;

    public boolean canHit = true;
    private boolean isShooting = false;

    public Shooter() {
        TalonFXConfiguration conf = new TalonFXConfiguration();

        conf.Slot0.kP = 10;
        conf.Slot0.kI = 6;
        conf.Slot0.kD = 6;

        flywheelLeftMotor.getConfigurator().apply(conf);

        conf.Slot0.kP = 10;
        conf.Slot0.kI = 6;
        conf.Slot0.kD = 6;

        flywheelRightMotor.getConfigurator().apply(conf);
    }

    public void setOutput(AngularVelocity vel) {
        flywheelLeftMotor.setControl(req.withVelocity(vel));
        flywheelRightMotor.setControl(req.withVelocity(vel));
    }

    public void setLaunchOutput(AngularVelocity vel) {
        launchLeftMotor.setControl(req.withVelocity(vel));
        launchRightMotor.setControl(req.withVelocity(vel));
    }

    public AngularVelocity getOutput() {
        if (flywheelLeftMotor.get() >= flywheelRightMotor.get()) {
            return flywheelLeftMotor.getVelocity().getValue();
        } else {
            return flywheelRightMotor.getVelocity().getValue();
        }
        // This has no Check if both go down, but at that point it should be obvious to
        // the drive team.
    }

    public void shoot() {
        setLaunchOutput(ShooterConstants.LAUNCH_MOTOR_OUTPUT);
    }

    // you may be able to move the isNear() stuff to canShoot()
    public Command setOutputCommand(double dist) {
        return Commands.run(() -> {

            if (Field.inAllianceZone.getAsBoolean()) {
            setOutput(calculateShooterOutput(dist));
            }
            else {
                setOutput(ShooterConstants.PASSING_OUTPUT);
            }
        }, this)
        .until(() -> {
            return
            getOutput().isNear(ShooterConstants.PASSING_OUTPUT, AngularVelocity.ofBaseUnits(3, RotationsPerSecond))
            || // OR
            getOutput().isNear(calculateShooterOutput(dist), AngularVelocity.ofBaseUnits(3, RotationsPerSecond));
        });
    }

    public Command shootCommand(boolean facingHub) {
        // Currently no way to shoot/pass while in allianceZone and not facing hub or when hub is inactive
        return Commands.run(() -> {
            currSpeed = getOutput();

            if (!Field.inAllianceZone.getAsBoolean()){
                this.isShooting = true;
                shoot();
            }
            else if (canShoot(facingHub)) {
                this.isShooting = true;
                shoot();
            }
            else {
                //vibrate controller
            }
            canHit = true;
        }, this);
    }

    public Command resetCommand() {
        return Commands.runOnce(() -> {
            setLaunchOutput(AngularVelocity.ofBaseUnits(0, RotationsPerSecond));
            setIsShooting(false);
        }, this);
    }

    public Boolean canShoot(boolean isfacingHub) { // When calling this wrap it in an if(isfacinghub) { canShoot() } or do && isfacinghub

        if (!this.canHit) return false;

        if (!isfacingHub) return false;

        return Field.isHubActive();
    }

    // TODO Implement once you know the robot architecture
    public LinearVelocity getBallVelocity() {
        // You will likely need to get the current flywheel rpm
        // this could also be done via a regression
        // switch for l/r/both

        return MetersPerSecond.of(5);
    }

    // Returns target state for shooter
    public AngularVelocity calculateShooterOutput(double dist /* In Meters */) {
        // If the shot is not possible(too clos/too far), return the current state(aka
        // no change) and do something to alert the driver(controller vibrate?)

        // We could do a gradient where depending how far we are into the band(dist from
        // hub, that is achieveable) we ramp up the motor output
        // We could do regressions
        // We could do Kinematics

        // TODO: add calculations so that canHit will change if need be

        if (!this.canHit) {
            return this.currSpeed;
        }

        return AngularVelocity.ofBaseUnits(0, RotationsPerSecond);
    }

    public void setIsShooting(boolean  val) {
        this.isShooting = val;
    }

    public boolean isShooting() {
        return this.isShooting;
    }
}
