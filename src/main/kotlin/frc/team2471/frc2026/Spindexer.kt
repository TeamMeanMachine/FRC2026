package frc.team2471.frc2026

import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.math.linearMap
import kotlin.math.cos

object Spindexer: SubsystemBase("Spindexer") {
    val table = NetworkTableInstance.getDefault().getTable("Spindexer")

    val spinMotor = TalonFX(Falcons.SPIN_0)
    val sidetakeMotor = TalonFX(Falcons.SIDETAKE)
    val uptakeMotor = TalonFX(Falcons.UPTAKE)

    @get:AutoLogOutput(key = "Spindexer/Current State")
    var currentState = State.OFF

    val spinVelocityEntry = table.getEntry("Spin Velocity")
    val spinLowerVelocityEntry = table.getEntry("Spin Lower Velocity")
    val sidetakeVelocityEntry = table.getEntry("Sidetake Velocity")
    val uptakeVelocityEntry = table.getEntry("Uptake Velocity")
    val agitateVelocityEntry = table.getEntry("Agitate Velocity")

//    val spinSpitVelocityEntry = table.getEntry("Spin Spit Velocity")
    val sidetakeSpitVelocityEntry = table.getEntry("Sidetake Spit Velocity")
    val uptakeSpitVelocityEntry = table.getEntry("Uptake Spit Velocity")

    val spinSlowdownTimeEntry = table.getEntry("Spin Slowdown Time")
    val spinSlowdownDelayTimeEntry = table.getEntry("Spin Slowdown Delay Time")
    val doSpinSlowdownEntry = table.getEntry("Do Spin Slowdown")
    val doSineSpinSlowdownEntry = table.getEntry("Do Sin Spin Slowdown")

    val SPIN_VELOCITY: Double get() = spinVelocityEntry.getDouble(78.0)
    val SPIN_LOWER_VELOCITY: Double get() = spinLowerVelocityEntry.getDouble(40.0)
    val SIDETAKE_VELOCITY: Double get() = sidetakeVelocityEntry.getDouble(115.0)
    val UPTAKE_VELOCITY: Double get() = uptakeVelocityEntry.getDouble(129.0)
    val AGITATE_VELOCITY: Double get() = agitateVelocityEntry.getDouble(40.0)

//    val SPIN_SPIT_VELOCITY: Double get() = spinSpitVelocityEntry.getDouble(0.0)
    val SIDETAKE_SPIT_VELOCITY: Double get() = sidetakeSpitVelocityEntry.getDouble(-50.0)
    val UPTAKE_SPIT_VELOCITY: Double get() = uptakeSpitVelocityEntry.getDouble(-50.0)

    val spinSlowdownDelayTime: Double get() = spinSlowdownDelayTimeEntry.getDouble(3.0)
    val spinSlowdownTime: Double get() = spinSlowdownTimeEntry.getDouble(3.0)
    val doSpinSlowdown: Boolean get() = doSpinSlowdownEntry.getBoolean(false)
    val doSineSpinSlowdown: Boolean get() = doSineSpinSlowdownEntry.getBoolean(false)

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


    @get:AutoLogOutput(key = "Spindexer/spinMotorVelocitySetpoint")
    var spinMotorVelocitySetpoint: Double = 0.0
        set(value) {
            spinMotor.setControl(
                if (value == 0.0) NeutralOut() else MotionMagicVelocityTorqueCurrentFOC(value)
            )
            field = value
        }

    @get:AutoLogOutput(key = "Spindexer/sidetakeMotorVelocitySetpoint")
    var sidetakeMotorVelocitySetpoint: Double = 0.0
        set(value) {
            sidetakeMotor.setControl(
                if (value == 0.0) NeutralOut() else VelocityTorqueCurrentFOC(value)
            )
            field = value
        }

    @get:AutoLogOutput(key = "Spindexer/uptakeMotorVelocitySetpoint")
    var uptakeMotorVelocitySetpoint: Double = 0.0
        set(value) {
            uptakeMotor.setControl(
                if (value == 0.0) NeutralOut() else VelocityTorqueCurrentFOC(value)
            )
            field = value
        }

    val stateOnTimer = Timer()
    @get:AutoLogOutput(key = "Spindexer/stateOnTime")
    val stateOnTime: Double get() = stateOnTimer.get()

    init {
        if (!spinVelocityEntry.exists()) spinVelocityEntry.setDouble(SPIN_VELOCITY)
        if (!spinLowerVelocityEntry.exists()) spinLowerVelocityEntry.setDouble(SPIN_LOWER_VELOCITY)
        if (!sidetakeVelocityEntry.exists()) sidetakeVelocityEntry.setDouble(SIDETAKE_VELOCITY)
        if (!uptakeVelocityEntry.exists()) uptakeVelocityEntry.setDouble(UPTAKE_VELOCITY)
        if (!agitateVelocityEntry.exists()) agitateVelocityEntry.setDouble(AGITATE_VELOCITY)

//        if (!spinSpitVelocityEntry.exists()) spinSpitVelocityEntry.setDouble(SPIN_SPIT_VELOCITY)
        if (!sidetakeSpitVelocityEntry.exists()) sidetakeSpitVelocityEntry.setDouble(SIDETAKE_SPIT_VELOCITY)
        if (!uptakeSpitVelocityEntry.exists()) uptakeSpitVelocityEntry.setDouble(UPTAKE_SPIT_VELOCITY)

        if (!spinSlowdownTimeEntry.exists()) spinSlowdownTimeEntry.setDouble(spinSlowdownTime)
        if (!spinSlowdownDelayTimeEntry.exists()) spinSlowdownDelayTimeEntry.setDouble(spinSlowdownDelayTime)
        if (!doSpinSlowdownEntry.exists()) doSpinSlowdownEntry.setBoolean(doSpinSlowdown)
        if (!doSineSpinSlowdownEntry.exists()) doSineSpinSlowdownEntry.setBoolean(doSineSpinSlowdown)

        spinVelocityEntry.setPersistent()
        spinLowerVelocityEntry.setPersistent()
        sidetakeVelocityEntry.setPersistent()
        uptakeVelocityEntry.setPersistent()
        agitateVelocityEntry.setPersistent()

//        spinSpitVelocityEntry.setPersistent()
        sidetakeSpitVelocityEntry.setPersistent()
        uptakeSpitVelocityEntry.setPersistent()

        spinSlowdownTimeEntry.setPersistent()
        spinSlowdownDelayTimeEntry.setPersistent()
        doSpinSlowdownEntry.setPersistent()
        doSineSpinSlowdownEntry.setPersistent()


        spinMotor.applyConfiguration {
            currentLimits(30.0, 30.0, 1.0)
            inverted(false)
            coastMode()
            s(2.0, StaticFeedforwardSignValue.UseVelocitySign)
            p(6.0)
            MotionMagic.MotionMagicAcceleration = 120.0


            OpenLoopRamps.TorqueOpenLoopRampPeriod = 10.0
        }
        spinMotor.addFollower(Falcons.SPIN_1)

        uptakeMotor.applyConfiguration {
            currentLimits(30.0, 30.0, 1.0)
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
        LoopLogger.record("b4 spindexer periodic")
        when (currentState) {
            State.OFF -> {
                spinMotorVelocitySetpoint = 0.0
                sidetakeMotorVelocitySetpoint = 0.0
                uptakeMotorVelocitySetpoint = 0.0
                stateOnTimer.stop()
            }
            State.ON -> {
                if (doSpinSlowdown && stateOnTime > spinSlowdownDelayTime) {
                    if (doSineSpinSlowdown) {
                        // Sine periodic slowdown
                        spinMotorVelocitySetpoint =
                            (cos(2.0 * Math.PI * (stateOnTime - spinSlowdownDelayTime) / spinSlowdownTime) + 1) * 0.5 * (SPIN_VELOCITY - SPIN_LOWER_VELOCITY) + SPIN_LOWER_VELOCITY
                    } else {
                        // Linear slowdown
                        spinMotorVelocitySetpoint = ((SPIN_VELOCITY - SPIN_LOWER_VELOCITY) * (stateOnTime - spinSlowdownDelayTime) / spinSlowdownTime) + SPIN_LOWER_VELOCITY
                    }
                } else {
                    if (Robot.isAutonomous) {
                        spinMotorVelocitySetpoint = SPIN_VELOCITY
                    } else {
                        spinMotorVelocitySetpoint = SPIN_VELOCITY * linearMap(0.0, 1.0, 0.40, 1.0, OI.driveRightTrigger.deadband(0.1))
                    }
                }
                sidetakeMotorVelocitySetpoint = SIDETAKE_VELOCITY
                uptakeMotorVelocitySetpoint = UPTAKE_VELOCITY

                if (!stateOnTimer.isRunning) stateOnTimer.restart()
            }
            State.SPITTING -> {
                spinMotorVelocitySetpoint = 0.0//SPIN_SPIT_VELOCITY
                sidetakeMotorVelocitySetpoint = SIDETAKE_SPIT_VELOCITY
                uptakeMotorVelocitySetpoint = UPTAKE_SPIT_VELOCITY
                stateOnTimer.stop()
            }
            State.AGITATING -> {
                spinMotorVelocitySetpoint = -AGITATE_VELOCITY
                sidetakeMotorVelocitySetpoint = 0.0
                uptakeMotorVelocitySetpoint = 0.0
                stateOnTimer.stop()
            }
        }
        LoopLogger.record("spindexer periodic")
    }


    enum class State {
        OFF,
        ON,
        SPITTING,
        AGITATING
    }
}