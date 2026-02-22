package frc.robot.controls;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.util.maplesim.RebuiltFuelOnFly;

public class DriverControls {

  private static Pose2d getTargetPose() {
    Pose2d hubPose = new Pose2d(
        Meter.of(11.902),
        Meter.of(4.031),
        Rotation2d.kZero);

    Logger.recordOutput("DriverControls/TargetHubPose", hubPose);

    return hubPose;
  }

  public static void configure(int port, TankDriveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    // Arcade drive: left stick Y = forward/back, right stick X = rotation
    // Apply deadband and scaling
    drivetrain.setDefaultCommand(
        drivetrain.arcadeDriveCommand(
            () -> {
              double input = -controller.getLeftY();
              if (Math.abs(input) < ControllerConstants.DEADBAND) {
                return 0.0;
              }
              return input * 0.5; // Scale down for safety
            },
            () -> {
              double input = -controller.getRightX();
              if (Math.abs(input) < ControllerConstants.DEADBAND) {
                return 0.0;
              }
              return input * 0.5; // Scale down for safety
            })
            .withName("Drive"));

    if (DriverStation.isTest()) {
      // Test mode controls
      controller.y().onTrue(Commands.runOnce(drivetrain::zeroGyro));
      // Add other test controls as needed
    } else if (Robot.isSimulation()) {
      // Fire fuel 10 times per second while button is held
      controller.back().whileTrue(
          Commands.repeatingSequence(
              fireFuel(drivetrain, superstructure),
              Commands.waitSeconds(0.1)));
    } else {
      // Teleop controls
      controller.start().onTrue(Commands.runOnce(drivetrain::zeroGyro));

      controller.leftBumper().whileTrue(
          superstructure.feedAllCommand()
              .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

      controller.rightBumper()
          .whileTrue(superstructure.setIntakeDeployAndRoll().withName("OperatorControls.intakeDeployed"));
    }
  }

  public static Command fireFuel(TankDriveSubsystem drivetrain, Superstructure superstructure) {
    return Commands.runOnce(() -> {
      SimulatedArena arena = SimulatedArena.getInstance();

      GamePieceProjectile fuel = new RebuiltFuelOnFly(
          drivetrain.getPose().getTranslation(),
          new Translation2d(
              superstructure.turret.turretTranslation.getX() * -1,
              superstructure.turret.turretTranslation.getY()),
          drivetrain.getRobotVelocity(),
          drivetrain.getPose().getRotation().rotateBy(superstructure.getAimRotation3d().toRotation2d()),
          superstructure.turret.turretTranslation.getMeasureZ(),

          // 0.5 times because we're applying spin to the fuel as we shoot it
          superstructure.getTangentialVelocity().times(0.5),
          superstructure.getHoodAngle());

      // Configure callbacks to visualize the flight trajectory of the projectile
      fuel.withProjectileTrajectoryDisplayCallBack(
          // Callback for when the note will eventually hit the target (if configured)
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileSuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)),
          // Callback for when the note will eventually miss the target, or if no target
          // is configured
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileUnsuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)));

      arena.addGamePieceProjectile(fuel);
    });
  }
}
