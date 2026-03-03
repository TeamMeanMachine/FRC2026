package frc.team2471.frc2026

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.control.MeanCommandXboxController
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.parallelCommand
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.control.commands.toCommand
import org.team2471.frc.lib.control.commands.waitCommand
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.math.normalize
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.rotationsPerSecond

object OI: SubsystemBase("OI") {
    val driverController = MeanCommandXboxController(0, false)
    val operatorController = MeanCommandXboxController(1)

    val deadbandDriver = 0.08
    val deadbandOperator = 0.1

    val driveTranslationX: Double
        get() = driverController.leftY.deadband(deadbandDriver)

    val driveTranslationY: Double
        get() = driverController.leftX.deadband(deadbandDriver)

    val rawDriveTranslation: Translation2d
        get() {
            val translation = Translation2d(driveTranslationX, driveTranslationY)
            return if (translation.norm > 1.0) {
                translation.normalize()
            } else {
                translation
            }
        }

    val driveRotation: Double
        get() = -driverController.rightX.deadband(deadbandDriver)

    val driveLeftTrigger: Double
        get() = driverController.leftTriggerAxis

    val driveLeftTriggerFullPress: Boolean
        get() = driverController.leftTriggerAxis > 0.95

    val driveRightTrigger: Double
        get() = driverController.rightTriggerAxis

    val driveRightTriggerFullPress: Boolean
        get() = driverController.rightTriggerAxis > 0.95

    val operatorLeftTrigger: Double
        get() = operatorController.leftTriggerAxis

    val operatorLeftY: Double
        get() = operatorController.leftY.deadband(deadbandOperator)

    val operatorLeftX: Double
        get() = operatorController.leftX.deadband(deadbandOperator)

    val operatorRightTrigger: Double
        get() = operatorController.rightTriggerAxis

    val operatorRightX: Double
        get() = operatorController.rightX.deadband(deadbandOperator)

    val operatorRightY: Double
        get() = operatorController.rightY.deadband(deadbandOperator)

    private val driverNotConnectedAlert: Alert = Alert("DRIVER JOYSTICK DISCONNECTED", Alert.AlertType.kError)
    private val operatorNotConnectedAlert: Alert = Alert("OPERATOR JOYSTICK DISCONNECTED", Alert.AlertType.kError)
    private val driverDebouncer = Debouncer(0.05)
    private val operatorDebouncer = Debouncer(0.05)



    init {
        println("inside OI init")
        // Default command, normal field-relative drive
        Drive.defaultCommand = Drive.joystickDrive()

        Turret.defaultCommand = Turret.aimAtTarget()

        Shooter.defaultCommand = runCommand(Shooter) {
            Shooter.hoodAngleSetpoint = Shooter.HOOD_STOW_SETPOINT.degrees
            Shooter.shooterVelocitySetpoint = 0.0.rotationsPerSecond
        }

        // Zero Gyro
        driverController.back().onTrue({
                println("zero gyro")
                Drive.zeroGyro()
            }.toCommand(Drive).ignoringDisable(true))

        // Reset Odometry Position
        driverController.start().onTrue( {
            Drive.pose = Pose2d(Translation2d(3.0, 3.0), Drive.heading)
        }.toCommand(Drive).ignoringDisable(true))

        driverController.a().onTrue(runOnceCommand { Shooter.hoodAngleSetpoint = 0.0.degrees })
        driverController.y().onTrue(Intake.home())

//        driverController.rightTrigger(0.1).whileTrue(runCommand {
//                Spindexer.currentState = Spindexer.State.ON
//        }.finallyRun { Spindexer.currentState = Spindexer.State.OFF })
        driverController.rightTrigger(0.1).or(driverController.rightBumper()).whileTrue(
            Shooter.shootOrRamp()
        )
//        driverController.rightTrigger(0.1).whileTrue(Shooter.shoot())
//        driverController.rightBumper().whileTrue(runCommand { Shooter.rampUpLoop() })

        driverController.leftTrigger(0.05).whileTrue(parallelCommand( runCommand {
            Intake.intakeState = Intake.IntakeState.INTAKING
        }.finallyRun { Intake.intakeState = Intake.IntakeState.OFF },
//            Drive.snakeMode()
        ))
        driverController.leftStick().whileTrue(runCommand {
            Intake.intakeState = Intake.IntakeState.SPITTING
        }.finallyRun { Intake.intakeState = Intake.IntakeState.OFF })

        driverController.leftBumper().whileTrue(waitCommand(1.0).finallyRun { wasSuspended ->
            if (wasSuspended) {
                if (Intake.isDeployed) {
                    Intake.stow()
                } else {
                    Intake.deploy()
                }
            } else {
                Intake.deepStow()
            }
        })
    }

    override fun periodic() {
        LoopLogger.record("b4 OI piodc")
        driverNotConnectedAlert.set(driverDebouncer.calculate(!driverController.isConnected))
        operatorNotConnectedAlert.set(operatorDebouncer.calculate(!operatorController.isConnected))


        if (Intake.intakeState == Intake.IntakeState.INTAKING && Robot.isTeleopEnabled) {
            driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0.013)
        } else {
            driverController.setRumble(GenericHID.RumbleType.kRightRumble, 0.0)
        }

        LoopLogger.record("OI piodc")
    }
}