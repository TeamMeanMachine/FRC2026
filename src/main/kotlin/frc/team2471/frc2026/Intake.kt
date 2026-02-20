package frc.team2471.frc2026

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
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

object Intake: SubsystemBase("Intake") {
    private val table = NetworkTableInstance.getDefault().getTable("Intake")
    val deployPoseEntry = table.getEntry("deployPose")
    val stowPoseEntry = table.getEntry("stowPose")
    val intakePowerEntry = table.getEntry("intakePower")

    val DEPLOY_POSE get() = deployPoseEntry.getDouble(25.75)
    val STOW_POSE get() = stowPoseEntry.getDouble(0.0)

    val INTAKE_POWER get() = intakePowerEntry.getDouble(0.84)
    const val HOMING_POWER = 0.1

    val rollerMotor = TalonFX(Falcons.INTAKE_ROLLER_0)
    val deployMotor = TalonFX(Falcons.INTAKE_DEPLOY)
    val stopSensor = DigitalInput(DigitalSensors.INTAKE_STOP_SENSOR)

    @get:AutoLogOutput(key = "Intake/Intake state")
    var intakeState: IntakeState = IntakeState.OFF

    @get:AutoLogOutput(key = "Intake/Hit Hard Stop")
    val hitHardStop get() = !stopSensor.get()

    @get:AutoLogOutput(key = "Intake/Roller Setpoint")
    var rollerSetpoint: Double = 0.0
        set(value) {
            field = value.coerceIn(-1.0, 1.0)
            rollerMotor.setControl(DutyCycleOut(field))
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


    init {
        if (!deployPoseEntry.exists()) deployPoseEntry.setDouble(DEPLOY_POSE)
        if (!stowPoseEntry.exists()) stowPoseEntry.setDouble(STOW_POSE)
        if (!intakePowerEntry.exists()) intakePowerEntry.setDouble(INTAKE_POWER)

        deployPoseEntry.setPersistent()
        stowPoseEntry.setPersistent()
        intakePowerEntry.setPersistent()



        deployMotor.applyConfiguration {
            currentLimits(20.0, 30.0, 1.0)
            coastMode()
            p(1.5)
            s(0.25, StaticFeedforwardSignValue.UseClosedLoopSign)
            motionMagic(750.0, 1500.0)
        }
        rollerMotor.applyConfiguration {
            currentLimits(20.0, 30.0, 1.0)
            coastMode()
        }
        rollerMotor.addFollower(Falcons.INTAKE_ROLLER_1)

        deployMotor.setPosition(0.0)

        this.defaultCommand = default()
    }

    fun deploy() {
        deploySetpoint = DEPLOY_POSE
    }

    fun stow() {
        deploySetpoint = STOW_POSE
    }

    fun home(): Command = sequenceCommand(
        runCommand(this) {
            deployMotor.setControl(DutyCycleOut(HOMING_POWER))
        }.onlyRunWhileTrue { hitHardStop },
        runCommand(this) {
            deployMotor.setControl(DutyCycleOut(-HOMING_POWER))
        }.onlyRunWhileFalse { hitHardStop }.finallyRun {
            deployMotor.setControl(DutyCycleOut(0.0))
            deployMotor.setPosition(0.0)
            stow()
        }
    )


    private fun default(): Command = runCommand(this) {
        when (intakeState) {
            IntakeState.OFF -> {
                rollerSetpoint = 0.0
            }
            IntakeState.INTAKING -> {
                rollerSetpoint = INTAKE_POWER
            }
            IntakeState.SPITTING -> {
                rollerSetpoint = -INTAKE_POWER
            }
        }
    }

    enum class IntakeState {
        INTAKING,
        OFF,
        SPITTING,
    }
}