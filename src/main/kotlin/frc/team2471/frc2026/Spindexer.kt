package frc.team2471.frc2026

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.commands.MechanismBase
import org.team2471.frc.lib.commands.command
import org.team2471.frc.lib.logging.LoopLogger
import org.team2471.frc.lib.hardware.ctre.addFollower
import org.team2471.frc.lib.hardware.ctre.applyConfiguration
import org.team2471.frc.lib.hardware.ctre.coastMode
import org.team2471.frc.lib.hardware.ctre.currentLimits
import org.team2471.frc.lib.hardware.ctre.inverted
import org.team2471.frc.lib.hardware.ctre.p
import org.team2471.frc.lib.hardware.ctre.s
import org.team2471.frc.lib.energy.BatteryLogger
import org.team2471.frc.lib.logging.getTunable
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.math.linearMap
import org.wpilib.command3.Command
import org.wpilib.telemetry.Telemetry
import org.wpilib.units.measure.AngularVelocity
import org.wpilib.units.measure.Current

object Spindexer: MechanismBase("Spindexer") {
    val table = Telemetry.getTable("Spindexer")

    val spinMotor = TalonFX(Falcons.SPIN_0, CANBus.systemcore(1))
    val spinMotorFollower = TalonFX(Falcons.SPIN_1, CANBus.systemcore(1))
    val sidetakeMotor = TalonFX(Falcons.SIDETAKE, CANBus.systemcore(1))
    val uptakeMotor = TalonFX(Falcons.UPTAKE, CANBus.systemcore(1))

    @get:AutoLogOutput(key = "Spindexer/Current State")
    var currentState = State.OFF

    val spinVelocityEntry = table.getTunable("Spin Velocity", 78.0, true)
    val spinLowerVelocityEntry = table.getTunable("Spin Lower Velocity", 40.0, true)
    val sidetakeVelocityEntry = table.getTunable("Sidetake Velocity", 115.0, true)
    val uptakeVelocityEntry = table.getTunable("Uptake Velocity", 129.0, true)
    val agitateVelocityEntry = table.getTunable("Agitate Velocity", 30.0, true)

    val sidetakeSpitVelocityEntry = table.getTunable("Sidetake Spit Velocity", -50.0, true)
    val uptakeSpitVelocityEntry = table.getTunable("Uptake Spit Velocity", -50.0, true)

    val spinSlowdownTimeEntry = table.getTunable("Spin Slowdown Time", 3.0, true)
    val spinSlowdownDelayTimeEntry = table.getTunable("Spin Slowdown Delay Time", 3.0, true)
    val doSpinSlowdownEntry = table.getTunable("Do Spin Slowdown", false, true)
    val doSineSpinSlowdownEntry = table.getTunable("Do Sin Spin Slowdown", false, true)

    val SPIN_VELOCITY: Double get() = spinVelocityEntry.get()
    val SPIN_LOWER_VELOCITY: Double get() = spinLowerVelocityEntry.get()
    val SIDETAKE_VELOCITY: Double get() = sidetakeVelocityEntry.get()
    val UPTAKE_VELOCITY: Double get() = uptakeVelocityEntry.get()
    val AGITATE_VELOCITY: Double get() = agitateVelocityEntry.get()

    val SIDETAKE_SPIT_VELOCITY: Double get() = sidetakeSpitVelocityEntry.get()
    val UPTAKE_SPIT_VELOCITY: Double get() = uptakeSpitVelocityEntry.get()

    val spinSlowdownDelayTime: Double get() = spinSlowdownDelayTimeEntry.get()
    val spinSlowdownTime: Double get() = spinSlowdownTimeEntry.get()
    val doSpinSlowdown: Boolean get() = doSpinSlowdownEntry.get()
    val doSineSpinSlowdown: Boolean get() = doSineSpinSlowdownEntry.get()

    @get:AutoLogOutput(key = "Spindexer/Spin Velocity")
    val spinVelocity: AngularVelocity get() = spinMotor.velocity.value
    @get:AutoLogOutput(key = "Spindexer/Uptake Velocity")
    val uptakeVelocity: AngularVelocity get() = uptakeMotor.velocity.value
    @get:AutoLogOutput(key = "Spindexer/Sidetake Velocity")
    val sidetakeVelocity: AngularVelocity get() = sidetakeMotor.velocity.value

    @get:AutoLogOutput(key = "Spindexer/Spindexer Current")
    val spindexerCurrent: Current get() = spinMotor.supplyCurrent.value
    @get:AutoLogOutput(key = "Spindexer/Uptake Current")
    val uptakeCurrent: Current get() = uptakeMotor.supplyCurrent.value
    @get:AutoLogOutput(key = "Spindexer/Sidetake Current")
    val sidetakeCurrent: Current get() = sidetakeMotor.supplyCurrent.value

    @get:AutoLogOutput(key = "Spindexer/Spindexer TorqueCurrent")
    val spindexerTorqueCurrent: Current get() = spinMotor.torqueCurrent.value
    @get:AutoLogOutput(key = "Spindexer/Uptake TorqueCurrent")
    val uptakeTorqueCurrent: Current get() = uptakeMotor.torqueCurrent.value
    @get:AutoLogOutput(key = "Spindexer/Sidetake TorqueCurrent")
    val sidetakeTorqueCurrent: Current get() = sidetakeMotor.torqueCurrent.value

    private val spinMotorControl = MotionMagicVelocityTorqueCurrentFOC(0.0)
    private val sidetakeMotorControl = VelocityTorqueCurrentFOC(0.0)
    private val uptakeMotorControl = VelocityTorqueCurrentFOC(0.0)


    @get:AutoLogOutput(key = "Spindexer/spinMotorVelocitySetpoint")
    var spinMotorVelocitySetpoint: Double = 0.0
        set(value) {
            spinMotor.setControl(
                if (value == 0.0 || !Intake.finishedHoming) NeutralOut() else spinMotorControl.withVelocity(value)//MotionMagicVelocityTorqueCurrentFOC(value)
            )
            field = value
        }

    @get:AutoLogOutput(key = "Spindexer/sidetakeMotorVelocitySetpoint")
    var sidetakeMotorVelocitySetpoint: Double = 0.0
        set(value) {
            sidetakeMotor.setControl(
                if (value == 0.0) NeutralOut() else sidetakeMotorControl.withVelocity(value)//VelocityTorqueCurrentFOC(value)
            )
            field = value
        }

    @get:AutoLogOutput(key = "Spindexer/uptakeMotorVelocitySetpoint")
    var uptakeMotorVelocitySetpoint: Double = 0.0
        set(value) {
            uptakeMotor.setControl(
                if (value == 0.0) NeutralOut() else uptakeMotorControl.withVelocity(value)//VelocityTorqueCurrentFOC(value)
            )
            field = value
        }

    var disableReversingAuto = false

    init {
        println("Spindexer initialization")

        spinMotor.applyConfiguration {
            currentLimits(10.0, 20.0, 0.5)
            inverted(false)
            coastMode()
            s(2.0, StaticFeedforwardSignValue.UseVelocitySign)
            p(6.0)
            MotionMagic.MotionMagicAcceleration = 120.0


            OpenLoopRamps.TorqueOpenLoopRampPeriod = 10.0
        }
        spinMotor.addFollower(spinMotorFollower)

        uptakeMotor.applyConfiguration {
            currentLimits(28.0, 40.0, 0.2)
            coastMode()
            inverted(true)

            p(7.0)
            s(2.0, StaticFeedforwardSignValue.UseVelocitySign)
        }

        sidetakeMotor.applyConfiguration {
            currentLimits(30.0, 30.0, 1.0)
            coastMode()

            p(7.0)
            s(2.0, StaticFeedforwardSignValue.UseVelocitySign)
        }
    }

    override fun periodic() {
        BatteryLogger.recordCurrent("Dye Rotor Spin", spinMotor.supplyCurrent.value * 2.0)
        BatteryLogger.recordCurrent("Dye Rotor Uptake", uptakeMotor.supplyCurrent.value)
        BatteryLogger.recordCurrent("Dye Rotor Sidetake", sidetakeMotor.supplyCurrent.value)
    }

    override fun defaultCommand(): Command = command(this) {
        LoopLogger.record("Spindexer default")
        when (currentState) {
            State.OFF -> {
                spinMotorVelocitySetpoint = 0.0
                sidetakeMotorVelocitySetpoint = 0.0
                uptakeMotorVelocitySetpoint = 0.0
            }

            State.ON -> {
                if (Robot.isAutonomous) {
                    spinMotorVelocitySetpoint = SPIN_VELOCITY
                } else {
                    spinMotorVelocitySetpoint =
                        SPIN_VELOCITY * linearMap(0.0, 1.0, 0.40, 1.0, OI.driveRightTrigger.deadband(0.1))
                }
                sidetakeMotorVelocitySetpoint = SIDETAKE_VELOCITY
                uptakeMotorVelocitySetpoint = UPTAKE_VELOCITY
            }

            State.SPITTING -> {
                spinMotorVelocitySetpoint = 0.0
                sidetakeMotorVelocitySetpoint = SIDETAKE_SPIT_VELOCITY
                uptakeMotorVelocitySetpoint = UPTAKE_SPIT_VELOCITY
            }

            State.AGITATING -> {
                if (Robot.isAutonomous && disableReversingAuto) {
                    spinMotorVelocitySetpoint = 0.0
                } else {
                    spinMotorVelocitySetpoint = -AGITATE_VELOCITY
                }
                sidetakeMotorVelocitySetpoint = 0.0
                uptakeMotorVelocitySetpoint = 0.0
            }
        }

        BatteryLogger.recordCurrent("Dye Rotor Spin", spinMotor.supplyCurrent.value * 2.0)
        BatteryLogger.recordCurrent("Dye Rotor Uptake", uptakeMotor.supplyCurrent.value)
        BatteryLogger.recordCurrent("Dye Rotor Sidetake", sidetakeMotor.supplyCurrent.value)

        LoopLogger.record("spindexer periodic")
    }

    enum class State {
        OFF,
        ON,
        SPITTING,
        AGITATING
    }
}