package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
    public static class Intake {
        public static final int INTAKE_ID = 6; // TBD
        public static final int HOPPER_ID = 7; // TBD
        public static final Angle EXTEND_DIST = Degrees.of(40); // TBD
        public static final AngularVelocity INTAKE_SPEED = RotationsPerSecond.of(5); // TBD

    }
}
