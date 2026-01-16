# Drivetrain Subsystem
## Drivetrain.java
Holds the actual Drivetrain subsystem created by the CTRE Swerve Generator. Contains methods for getting estimated pose, adding vision data to the estimator, and setting the operator perspective.

## DriveConstants.java
Contains constants such as speed limits or tolerances that all drivetrain files use. The file makes it easy to quickly configure the drivetrain.

## Logger.java
Publishes telemetry data from swerve to network tables. Additionally, the file publishes multiple mechanism2d objects that reflect the state of the swerve drive. 

## SwerveLimiter.java
The SwerveLimiter object exposes the `calculate` function, which returns a new ChassisSpeeds object scaled proportionally so that no wheel violates the acceleration and deceleration limits set in `DriveConstants`

## SysIdRunner.java
Contains the methods and varables required for preforming SysId. This is broken out of `Drivetrain.java`(Where it was originally) to reduce clutter. We have not had much success using the results of SysId in the past.

## DriveCommands.java
This class is used to control the drivetrain. It seperates the control of translation and rotation, letting users mix and match different control schemes. The controls are built around suppliers, which can be chained together to create complex behaviors (for example `rotateToFacePoint` uses `rotateToRotation`). The class also exposes a method to wrap a command so it ends when the driver makes an input.