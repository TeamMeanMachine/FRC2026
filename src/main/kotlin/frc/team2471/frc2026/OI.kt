package frc.team2471.frc2026

import org.team2471.frc.lib.commands.onCancel
import org.team2471.frc.lib.commands.periodic
import org.team2471.frc.lib.commands.use
//import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.control.isConnected
import org.team2471.frc.lib.math.applyDeadband
//import org.team2471.frc.lib.control.commands.finallyRun
//import org.team2471.frc.lib.control.commands.parallelCommand
//import org.team2471.frc.lib.control.commands.runCommand
//import org.team2471.frc.lib.control.commands.runOnceCommand
//import org.team2471.frc.lib.control.commands.toCommand
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.util.demoMode
import org.wpilib.command3.Scheduler
import org.wpilib.command3.button.CommandXboxController
import org.wpilib.driverstation.Alert
import org.wpilib.math.filter.Debouncer
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.networktables.NetworkTableInstance
import org.wpilib.opmode.PeriodicOpMode
import org.wpilib.opmode.Teleop

object OI {
    private val table = NetworkTableInstance.getDefault().getTable("OI")

    val rotationMultiplierEntry = table.getEntry("Rotation Multiplier")
    val rotationMultiplier = rotationMultiplierEntry.getDouble(0.8)

    val driverController = CommandXboxController(0)
    val operatorController = CommandXboxController(1)

    val deadbandDriver = 0.08
    val deadbandOperator = 0.1

    val rawDriveTranslationX: Double
        get() = driverController.leftY

    val rawDriveTranslationY: Double
        get() = driverController.leftX

    val driveTranslation: Translation2d
        get() {
            return Translation2d(rawDriveTranslationX, rawDriveTranslationY).applyDeadband(deadbandDriver)
        }

    val driveTranslationX: Double get() = driveTranslation.x
    val driveTranslationY: Double get() = driveTranslation.y

    val driveRotation: Double
        get() = -driverController.rightX.deadband(deadbandDriver) * rotationMultiplier

    val driveLeftTrigger: Double
        get() = driverController.leftTrigger

    val driveLeftTriggerFullPress: Boolean
        get() = driverController.leftTrigger > 0.95

    val driveRightTrigger: Double
        get() = driverController.rightTrigger

    val driveRightTriggerFullPress: Boolean
        get() = driverController.rightTrigger > 0.95

    val operatorLeftTrigger: Double
        get() = operatorController.leftTrigger

    val operatorLeftY: Double
        get() = operatorController.leftY.deadband(deadbandOperator)

    val operatorLeftX: Double
        get() = operatorController.leftX.deadband(deadbandOperator)

    val operatorRightTrigger: Double
        get() = operatorController.rightTrigger

    val operatorRightX: Double
        get() = operatorController.rightX.deadband(deadbandOperator)

    val operatorRightY: Double
        get() = operatorController.rightY.deadband(deadbandOperator)

    private val driverNotConnectedAlert: Alert = Alert("DRIVER JOYSTICK DISCONNECTED", Alert.Level.HIGH)
    private val operatorNotConnectedAlert: Alert = Alert("OPERATOR JOYSTICK DISCONNECTED", Alert.Level.HIGH)
    private val driverDebouncer = Debouncer(0.05)
    private val operatorDebouncer = Debouncer(0.05)



    init {
        println("inside OI init")

        if (!rotationMultiplierEntry.exists()) rotationMultiplierEntry.setDouble(rotationMultiplier)
        rotationMultiplierEntry.setPersistent()

        /** DEFAULT JOYSTICK BINDINGS. (Will be active by default in every OpMode unless overridden) */


        driverController.menu().onTrue(Drive.zeroGyroCommand()) // Zero Gyro
        driverController.view().onTrue(use { Drive.pose = Pose2d(Translation2d(3.0, 3.0), Drive.heading) }) // Reset Odometry Position

        (driverController.y().and(driverController.dpadDown().negate())).onTrue(Intake.home())

        driverController.rightTrigger(0.1)
            .or(driverController.rightBumper())
            .or(driverController.rightStick())
            .whileTrue(Shooter.shootOrRamp())

        driverController.leftBumper().and(driverController.dpadDown().negate()).whileTrue(use {
            this.periodic {
                Intake.deploy()
                Intake.intakeState = Intake.IntakeState.INTAKING
            }
        }.onCancel {
            Intake.intakeState = Intake.IntakeState.OFF
        })

        driverController.leftStick().onTrue(use {
            if (Intake.isDeployed) {
                Intake.stow()
            } else {
                Intake.deploy()
            }
        })

        driverController.a().whileTrue(
            Drive.snakeMode()
        )
        driverController.x().whileTrue(use(Drive) {
            this.periodic {
                Drive.xPose()
            }
        })

        driverController.a().onTrue(use {
            Intake.deploySetpoint = Intake.DEEP_STOW_POSE
        })
        driverController.x().onTrue(use {
            Intake.deploySetpoint = Intake.STOW_POSE
        })
        driverController.b().onTrue(use {
            Intake.deploySetpoint = Intake.DEPLOY_POSE
        })

        (driverController.dpadDown().and(driverController.y())).onTrue(Intake.homeDeploy())
        (driverController.dpadDown().and(driverController.leftBumper())).onTrue(use { Intake.deepStow() })

        driverController.dpadLeft().whileTrue(Turret.staticAimAtTarget())
        driverController.dpadRight().whileTrue(FieldManager.disableAutoHoodRetractionCommand())

        driverController.dpadUp().onTrue(use { Turret.offset -= 2.0.degrees})
        driverController.dpadDown().and(driverController.y().negate()).and(driverController.leftBumper().negate()).onTrue(use { Turret.offset += 2.0.degrees})



        Scheduler.getDefault().addPeriodic {
            LoopLogger.record("b4 OI piodc")
            driverNotConnectedAlert.set(driverDebouncer.calculate(!driverController.isConnected))
            operatorNotConnectedAlert.set(operatorDebouncer.calculate(!operatorController.isConnected))
            LoopLogger.record("OI piodc")
        }
    }

    @Teleop(name = "Country Roads!")
    class TeleopMode: PeriodicOpMode() {
        init {
            println("TeleopMode selected")

            if (demoMode) {
                println("Demo mode enabled")
                // Demo mode bindings go here
            }

        }

        override fun disabledPeriodic() {}

        override fun start() {
            println("Teleop Mode start!")
        }

        override fun periodic() {}

        override fun end() {
            println("Teleop Mode end!")
        }
    }
}