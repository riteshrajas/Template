/*
 * *****************************************************************************
 *  * Copyright (c) 2024 FEDS 201. All rights reserved.
 *  *
 *  * This codebase is the property of FEDS 201 Robotics Team.
 *  * Unauthorized copying, reproduction, or distribution of this code, or any
 *  * portion thereof, is strictly prohibited.
 *  *
 *  * This code is provided "as is" and without any express or implied warranties,
 *  * including, without limitation, the implied warranties of merchantability
 *  * and fitness for a particular purpose.
 *  *
 *  * For inquiries or permissions regarding the use of this code, please contact
 *  * feds201@gmail.com
 *  ****************************************************************************
 *
 */

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.OIConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.utils.Telemetry;

import java.util.List;
import java.util.Map;


public class RobotContainer {
		public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

		private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
				                                                 .withDeadband(SwerveConstants.MaxSpeed * 0.1)
				                                                 .withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1)
				                                                 .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
		private final SwerveRequest.FieldCentricFacingAngle autoAim = new SwerveRequest.FieldCentricFacingAngle()
				                                                              .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
		private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
		private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
		private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

		public double swerveSpeedMultiplier = 1;

		// SHOOTER


		public final CommandXboxController driverController;
		public final CommandXboxController operatorController;

		SendableChooser<Command> autonChooser = new SendableChooser<>();

		ShuffleboardTab commandsTab = Shuffleboard.getTab("commands");

		public RobotContainer() {

				driverController = new CommandXboxController(OIConstants.kDriverController);
				operatorController = new CommandXboxController(OIConstants.kOperatorController);

				registerAllAutoCommands();
				configureDefaultCommands();
				configureDriverController();
				configureOperatorController();

				setupErrorTriggers();
				setupAutonCommands();
		}

		private void registerAllAutoCommands() {
				NamedCommands.registerCommands(( List<Pair<String, Command>> ) drivetrain);
		}

		private void setupAutonCommands() {



				Shuffleboard.getTab("autons").add(autonChooser);
		}

		private void configureDefaultCommands() {
				drivetrain.setDefaultCommand(new ParallelCommandGroup(
						drivetrain.applyRequest(() -> drive
								                              .withVelocityX(-driverController.getLeftY()
										                                             * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
								                              .withVelocityY(-driverController.getLeftX()
										                                             * SwerveConstants.MaxSpeed * swerveSpeedMultiplier)
								                              .withRotationalRate(-driverController.getRightX() *
										                                                  SwerveConstants.MaxAngularRate * swerveSpeedMultiplier)),
						new RepeatCommand(
								new InstantCommand(this::printCurrentStickValues))));

				if (Utils.isSimulation()) {
						drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
								Rotation2d.fromDegrees(90)));
				}

				drivetrain
						.registerTelemetry(
								logger::telemeterize);


		}

		private void printCurrentStickValues() {
				SmartDashboard.putNumber("Driver Left X", driverController.getLeftX());
				SmartDashboard.putNumber("Driver Left Y", driverController.getLeftY());
				SmartDashboard.putNumber("Driver Right X", driverController.getRightX());
				SmartDashboard.putNumber("Driver Right Y", driverController.getRightY());
		}

		private void configureDriverController() {
				// reset the field-centric heading on left bumper press
				// LOAD BUTTON


		}

		public void configureOperatorController() {


		}

		private void setupErrorTriggers() {
				// There should be some feedback for an "failure mode" but rumbling the
				// controller continuously was obnoxious lol.
		}

		public Command getAutonomousCommand() {
				return autonChooser.getSelected(); // runAuto;
				// return null;
		}

}