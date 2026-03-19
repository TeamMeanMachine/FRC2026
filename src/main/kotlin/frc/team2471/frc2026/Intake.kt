package frc.team2471.frc2026

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ControlModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.frc2026.Robot.powerTracker
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.control.CurrentLimits
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.onlyRunWhileFalse
import org.team2471.frc.lib.control.commands.onlyRunWhileTrue
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.control.commands.waitCommand
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.motionMagic
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.units.asAmps
import org.team2471.frc.lib.units.asVolts
import org.team2471.frc.lib.util.isSim
import kotlin.math.absoluteValue

object Intake: SubsystemBase("Intake") {
    private val table = NetworkTableInstance.getDefault().getTable("Intake")

    val deployPoseEntry = table.getEntry("deployPose")
    val stowPoseEntry = table.getEntry("stowPose")
    val deepStowPoseEntry = table.getEntry("deepStowPose")
    val intakePowerEntry = table.getEntry("intakePower")

    val DEPLOY_POSE get() = deployPoseEntry.getDouble(25.75)
    val STOW_POSE get() = stowPoseEntry.getDouble(2.0)
    val DEEP_STOW_POSE get() = deepStowPoseEntry.getDouble(0.0)

    val INTAKE_POWER get() = intakePowerEntry.getDouble(75.0)
    const val HOMING_POWER = 0.1

    val rollerMotor = TalonFX(Falcons.INTAKE_ROLLER_0)
    val rollerMotorFollower = TalonFX(Falcons.INTAKE_ROLLER_1)
    val deployMotor = TalonFX(Falcons.INTAKE_DEPLOY)
    val stopSensor = DigitalInput(DigitalSensors.INTAKE_STOP_SENSOR)

    @get:AutoLogOutput(key = "Intake/Intake state")
    var intakeState: IntakeState = IntakeState.OFF
    var prevIntakeState = intakeState

    @get:AutoLogOutput(key = "Intake/Hit Hard Stop")
    val hitHardStop get() = !stopSensor.get()

    @get:AutoLogOutput(key = "Intake/Roller Motor Temp")
    val rollerTemp get() = rollerMotor.deviceTemp.valueAsDouble

    @get:AutoLogOutput(key = "Intake/Velocity Setpoint")
    var velocitySetpoint: Double = 0.0
        set(value) {
            field = value.coerceIn(-100.0, 100.0)
            if (field == 0.0) {
                rollerMotor.setControl(NeutralOut())
            } else {
                rollerMotor.setControl(DutyCycleOut(field / 100.0).withEnableFOC(true))
            }
        }

    @get:AutoLogOutput(key = "Intake/Deploy Setpoint")
    var deploySetpoint: Double = 0.0
        set(value) {
            field = value
            deployMotor.setControl(MotionMagicVoltage(field))
        }

    @get:AutoLogOutput(key = "Intake/Roller Velocity")
    val rollerVelocity: Double
        get() = rollerMotor.velocity.valueAsDouble

    @get:AutoLogOutput(key = "Intake/Roller Current")
    val rollerCurrent: Double
        get() = rollerMotor.supplyCurrent.valueAsDouble

    @get:AutoLogOutput(key = "Intake/Deploy Position")
    val deployPosition:Double
        get() = deployMotor.position.valueAsDouble

    @get:AutoLogOutput(key = "Intake/Deploy Current")
    val deployCurrent: Double
        get() = deployMotor.supplyCurrent.valueAsDouble

    @get:AutoLogOutput(key = "Intake/Deploy Motor Position")
    val deployMotorPosition: Double
        get() = deployMotor.position.valueAsDouble

    @get:AutoLogOutput(key = "Intake/Deploy Motor Error")
    val deployMotorError: Double
        get() = deployMotorPosition - deploySetpoint//deployMotor.closedLoopError.valueAsDouble

    var finishedHoming: Boolean = false
    var isDeployed: Boolean = false

    var goingToSetpoint: Boolean = false

    val autoCurrentLimits = CurrentLimits(30.0, 40.0, 1.0)
    val teleopCurrentLimits = CurrentLimits(15.0, 40.0, 0.2)


    init {
        if (!deployPoseEntry.exists()) deployPoseEntry.setDouble(DEPLOY_POSE)
        if (!stowPoseEntry.exists()) stowPoseEntry.setDouble(STOW_POSE)
        if (!deepStowPoseEntry.exists()) deepStowPoseEntry.setDouble(DEEP_STOW_POSE)
        if (!intakePowerEntry.exists()) intakePowerEntry.setDouble(INTAKE_POWER)

        deployPoseEntry.setPersistent()
        stowPoseEntry.setPersistent()
        deepStowPoseEntry.setPersistent()
        intakePowerEntry.setPersistent()



        deployMotor.applyConfiguration {
            currentLimits(5.0, 25.0, 0.25)
//            statorCurrentLimit(35.0)
            coastMode()
            p(1.5)
            s(0.25, StaticFeedforwardSignValue.UseClosedLoopSign)

            p(0.05, 1)
            s(0.25, StaticFeedforwardSignValue.UseClosedLoopSign, 1)

            motionMagic(750.0, 1500.0)
        }
        rollerMotor.applyConfiguration {
            currentLimits(autoCurrentLimits.peakLimit, autoCurrentLimits.continuousLimit, autoCurrentLimits.peakDuration)
//            currentLimits(15.0, 40.0, 0.2)
            p(7.0)
            s(10.0, StaticFeedforwardSignValue.UseVelocitySign)
            coastMode()
        }
        rollerMotor.addFollower(rollerMotorFollower)

        deployMotor.setPosition(0.0)

        if (!isSim) {
            powerTracker.addMotors("Intake Roller", { rollerMotor.getSupplyCurrent(true).value.asAmps }, 2, {rollerMotor.supplyVoltage.value.asVolts})
            powerTracker.addMotors("Intake Deploy", { rollerMotor.getSupplyCurrent(true).value.asAmps })
        }

        this.defaultCommand = default()
    }

    fun deploy() {
        goingToSetpoint = true
        deploySetpoint = DEPLOY_POSE
        isDeployed = true
    }

    fun stow() {
        goingToSetpoint = true
        deploySetpoint = STOW_POSE
        isDeployed = false
    }

    fun deepStow() {
        goingToSetpoint = true
        deploySetpoint = DEEP_STOW_POSE
        isDeployed = false
    }

    fun home(): Command = sequenceCommand(
        runCommand(this) {
            println("going in?")
            deployMotor.setControl(DutyCycleOut(HOMING_POWER))
        }.onlyRunWhileTrue { hitHardStop },
        runCommand(this) {
            println("going out?")
            deployMotor.setControl(DutyCycleOut(-HOMING_POWER))
        }.onlyRunWhileFalse { hitHardStop }.withTimeout(6.0).finallyRun {
            deployMotor.setControl(DutyCycleOut(0.0))
            println("Deploy Pos: ${deployMotor.position}")
            deployMotor.setPosition(0.13)
            finishedHoming = true
//            stow()
        }
    )

    fun pulse(): Command = sequenceCommand(
        runOnceCommand { stow() },
        waitCommand(0.25),
        runOnceCommand { deploy() },
        waitCommand(0.25),
    ).repeatedly().onlyRunWhileTrue { Robot.isAutonomous || OI.driverController.rightTriggerAxis > 0.75 }.finallyRun {
        stow()
    }


    fun homeDeploy(): Command = runOnce {
        deployMotor.setPosition(deployPosition)
    }


    private fun default(): Command = runCommand(this) {
        LoopLogger.record("b4 Intake default")
        when (intakeState) {
            IntakeState.OFF -> {
                velocitySetpoint = 0.0
            }
            IntakeState.INTAKING -> {
                velocitySetpoint = if (Robot.isAutonomous) 100.0 else INTAKE_POWER
                if (!Shooter.isShooting) {
                    Spindexer.currentState = Spindexer.State.AGITATING
                }
            }
            IntakeState.SPITTING -> {
                velocitySetpoint = -INTAKE_POWER
            }
        }
        LoopLogger.record("Intake default when")
        if (prevIntakeState == IntakeState.INTAKING && intakeState != IntakeState.INTAKING) {
            Spindexer.currentState = Spindexer.State.OFF
        }
        prevIntakeState = intakeState

        if (goingToSetpoint && deployMotorError.absoluteValue < 1.0) {
            goingToSetpoint = false
        }
        LoopLogger.record("Intake default b4 controlMode")

        if (!goingToSetpoint) {
            if (deployMotorError < -0.5 && deployMotor.controlMode.value != ControlModeValue.NeutralOut) {
                deployMotor.setControl(DutyCycleOut(0.05))
            } else {
                deployMotor.setControl(NeutralOut())
            }
        }
        LoopLogger.record("Intake default")
    }

    enum class IntakeState {
        INTAKING,
        OFF,
        SPITTING,
    }
}