package frc.team2471.frc2026

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.NetworkButton
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.onlyRunWhileFalse
import org.team2471.frc.lib.control.commands.onlyRunWhileTrue
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.motionMagic
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
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
    val deployMotor = TalonFX(Falcons.INTAKE_DEPLOY)
    val stopSensor = DigitalInput(DigitalSensors.INTAKE_STOP_SENSOR)

    @get:AutoLogOutput(key = "Intake/Intake state")
    var intakeState: IntakeState = IntakeState.OFF
    var prevIntakeState = intakeState

    @get:AutoLogOutput(key = "Intake/Hit Hard Stop")
    val hitHardStop get() = !stopSensor.get()

    @get:AutoLogOutput(key = "Intake/Velocity Setpoint")
    var velocitySetpoint: Double = 0.0
        set(value) {
            field = value.coerceIn(-100.0, 100.0)
            if (field == 0.0) {
                rollerMotor.setControl(NeutralOut())
            } else {
                rollerMotor.setControl(DutyCycleOut(field / 100.0))
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
            currentLimits(20.0, 30.0, 1.0)
//            statorCurrentLimit(35.0)
            coastMode()
            p(1.5)
            s(0.25, StaticFeedforwardSignValue.UseClosedLoopSign)

            p(0.05, 1)
            s(0.25, StaticFeedforwardSignValue.UseClosedLoopSign, 1)

            motionMagic(750.0, 1500.0)
        }
        rollerMotor.applyConfiguration {
            currentLimits(40.0, 50.0, 1.0)
            p(7.0)
            s(10.0, StaticFeedforwardSignValue.UseVelocitySign)
            coastMode()
        }
        rollerMotor.addFollower(Falcons.INTAKE_ROLLER_1)

        deployMotor.setPosition(0.0)

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
        }.onlyRunWhileFalse { hitHardStop }.withTimeout(10.0).finallyRun {
            deployMotor.setControl(DutyCycleOut(0.0))
            println("Deploy Pos: ${deployMotor.position}")
            deployMotor.setPosition(0.13)
            finishedHoming = true
//            stow()
        }
    )


    fun homeDeploy(): Command = runOnce {
        deployMotor.setPosition(deployPosition)
    }


    private fun default(): Command = runCommand(this) {
        when (intakeState) {
            IntakeState.OFF -> {
                velocitySetpoint = 0.0
            }
            IntakeState.INTAKING -> {
                velocitySetpoint = INTAKE_POWER
                Spindexer.currentState = Spindexer.State.AGITATING
            }
            IntakeState.SPITTING -> {
                velocitySetpoint = -INTAKE_POWER
            }
        }
        if (prevIntakeState == IntakeState.INTAKING && intakeState != IntakeState.INTAKING) {
            Spindexer.currentState = Spindexer.State.OFF
        }
        prevIntakeState = intakeState

        if (goingToSetpoint && deployMotorError.absoluteValue < 1.0) {
            goingToSetpoint = false
        }

        if (!goingToSetpoint) {
            if (deployMotorError < -0.5) {
                deployMotor.setControl(DutyCycleOut(0.05))
            } else {
                deployMotor.setControl(NeutralOut())
            }
        }
    }

    enum class IntakeState {
        INTAKING,
        OFF,
        SPITTING,
    }
}