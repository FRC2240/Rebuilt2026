package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.DriveCommands.TranslationalVelocity;
import frc.robot.utils.Field;

public class DriveAssist {
    private DriveCommands driveCommands;
    private Drivetrain drivetrain;

    public DriveAssist(DriveCommands driveCommands) {
        this.driveCommands = driveCommands;
        this.drivetrain = driveCommands.drivetrain;
    }

    public Command driveInLineCommand() {
        return Commands.defer(() -> {
            if (Field.inTrenchZone()) {
                double distToLower = drivetrain.getPose().getY() - Field.TRENCH_MIDPOINT;
                double distToUpper = drivetrain.getPose().getY()
                        - (Field.FIELD_WIDTH.in(Meters) - Field.TRENCH_MIDPOINT);
                double closer = Math.abs(distToUpper) < Math.abs(distToLower)
                        ? Field.FIELD_WIDTH.in(Meters) - Field.TRENCH_MIDPOINT
                        : Field.TRENCH_MIDPOINT;

                return driveCommands.drive(driveInLine(closer), trenchAlign());
            } else if (Field.inBumpZone()) {
                double distToLower = drivetrain.getPose().getY() - Field.BUMP_MIDPOINT;
                double distToUpper = drivetrain.getPose().getY() - (Field.FIELD_WIDTH.in(Meters) - Field.BUMP_MIDPOINT);
                double closer = Math.abs(distToUpper) < Math.abs(distToLower)
                        ? Field.FIELD_WIDTH.in(Meters) - Field.BUMP_MIDPOINT
                        : Field.BUMP_MIDPOINT;

                return driveCommands.drive(driveInLine(closer), driveCommands.rotateWithJoystick());
            }
            return driveCommands.drive(driveCommands.driveWithJoystick(), driveCommands.rotateWithJoystick());
        },
                Set.of(drivetrain));
    }

    public Command driveFacingTarget(){
        return Commands.defer(() -> {
            if (Field.inAllianceZone()) {
                return driveCommands.drive(driveCommands.driveWithJoystick(), 
                        driveCommands.rotateToFacePoint(Field.HUB_CENTER_TRANSLATION::get));
            } else {
                return driveCommands.drive(driveCommands.driveWithJoystick(), 
                        driveCommands.rotateToFacePoint(Field::getTranslationOfPassPoint));
            }
        }, 
                Set.of(drivetrain));
        }

    public Supplier<AngularVelocity> trenchAlign() {
        return driveCommands.rotateToRotation(() -> {
            double currentRotation = Math.abs(drivetrain.getHeading().getDegrees());

            if (currentRotation >= 90) {
                return Rotation2d.fromDegrees(180);
            }
            return Rotation2d.fromDegrees(0);
        });
    }

    public Supplier<TranslationalVelocity> driveInLine(double targetY) {
        TranslationalVelocity velocity = TranslationalVelocity.none();
        return () -> {
            int allianceMultiplier = (DriverStation.getAlliance().isEmpty()
                    || DriverStation.getAlliance().get() == Alliance.Red) ? 1 : -1;

            double currentY = drivetrain.getTranslation().getY();
            double linearDistance = targetY - currentY;

            double velocityOutput = DriveConstants.TRANSLATION_PID_CONTROLLER.calculate(linearDistance, 0);
            // Limit the resulting velocity
            velocityOutput = Math.min(velocityOutput/2, driveCommands.getMaxDriveSpeed().in(MetersPerSecond));

            velocity.x = driveCommands.getMaxDriveSpeed()
                    .times(driveCommands.delinearize(driveCommands.applyDeadband(driveCommands.joystick.getLeftY(), DriveConstants.CONTROLLER_DEADBAND), 1.5) * allianceMultiplier);
            velocity.y = MetersPerSecond.of(-velocityOutput);

            return velocity;
        };
    }
}