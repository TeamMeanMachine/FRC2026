package frc.team2471.frc2026

import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s

object Spindexer: SubsystemBase("Spindexer") {
    val table = NetworkTableInstance.getDefault().getTable("Spindexer")

    val spinMotor = TalonFX(Falcons.SPIN_0)
    val sidetakeMotor = TalonFX(Falcons.SIDETAKE)
    val uptakeMotor = TalonFX(Falcons.UPTAKE)

    @get:AutoLogOutput(key = "Spindexer/Current State")
    var currentState = State.OFF

    val spinVelocityEntry = table.getEntry("Spin Velocity")
    val sidetakeVelocityEntry = table.getEntry("Sidetake Velocity")
    val uptakeVelocityEntry = table.getEntry("Uptake Velocity")

    val spinSpitVelocityEntry = table.getEntry("Spin Spit Velocity")
    val sidetakeSpitVelocityEntry = table.getEntry("Sidetake Spit Velocity")
    val uptakeSpitVelocityEntry = table.getEntry("Uptake Spit Velocity")

    val SPIN_VELOCITY: Double get() = spinVelocityEntry.getDouble(75.0)
    val SIDETAKE_VELOCITY: Double get() = sidetakeVelocityEntry.getDouble(105.0)
    val UPTAKE_VELOCITY: Double get() = uptakeVelocityEntry.getDouble(129.0)

    val SPIN_SPIT_VELOCITY: Double get() = spinSpitVelocityEntry.getDouble(0.0)
    val SIDETAKE_SPIT_VELOCITY: Double get() = sidetakeSpitVelocityEntry.getDouble(-50.0)
    val UPTAKE_SPIT_VELOCITY: Double get() = uptakeSpitVelocityEntry.getDouble(-50.0)

    @get:AutoLogOutput(key = "Spindexer/spindexerVelocity")
    val spindexerVelocity: Double
        get() = spinMotor.velocity.valueAsDouble

    @get:AutoLogOutput(key = "Spindexer/spindexerCurrent")
    val spindexerCurrent: Double
        get() = spinMotor.supplyCurrent.valueAsDouble

    @get:AutoLogOutput(key = "Spindexer/uptakeVelocity")
    val uptakeVelocity: Double
        get() = uptakeMotor.velocity.valueAsDouble

    @get:AutoLogOutput(key = "Spindexer/uptakeCurrent")
    val uptakeCurrent: Double
        get() = uptakeMotor.supplyCurrent.valueAsDouble

    @get:AutoLogOutput(key = "Spindexer/sidetakeMotorVelocity")
    val sidetakeVelocity: Double
        get() = sidetakeMotor.velocity.valueAsDouble

    @get:AutoLogOutput(key = "Spindexer/sidetakeCurrent")
    val sidetakeCurrent: Double
        get() = sidetakeMotor.supplyCurrent.valueAsDouble

    var spinMotorVelocitySetpoint: Double = 0.0
        set(value) {
            if (value == 0.0) {
                spinMotor.setControl(NeutralOut())
            } else {
                spinMotor.setControl(MotionMagicVelocityTorqueCurrentFOC(value))
            }
        }

    init {
        if (!spinVelocityEntry.exists()) spinVelocityEntry.setDouble(SPIN_VELOCITY)
        if (!sidetakeVelocityEntry.exists()) sidetakeVelocityEntry.setDouble(SIDETAKE_VELOCITY)
        if (!uptakeVelocityEntry.exists()) uptakeVelocityEntry.setDouble(UPTAKE_VELOCITY)

        if (!spinSpitVelocityEntry.exists()) spinSpitVelocityEntry.setDouble(SPIN_SPIT_VELOCITY)
        if (!sidetakeSpitVelocityEntry.exists()) sidetakeSpitVelocityEntry.setDouble(SIDETAKE_SPIT_VELOCITY)
        if (!uptakeSpitVelocityEntry.exists()) uptakeSpitVelocityEntry.setDouble(UPTAKE_SPIT_VELOCITY)

        spinVelocityEntry.setPersistent()
        sidetakeVelocityEntry.setPersistent()
        uptakeVelocityEntry.setPersistent()

        spinSpitVelocityEntry.setPersistent()
        sidetakeSpitVelocityEntry.setPersistent()
        uptakeSpitVelocityEntry.setPersistent()


        spinMotor.applyConfiguration {
            currentLimits(30.0, 30.0, 1.0)
            inverted(false)
            coastMode()
            s(2.0, StaticFeedforwardSignValue.UseVelocitySign)
            p(7.0)

            MotionMagic.MotionMagicAcceleration = 70.0

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
        when (currentState) {
            State.OFF -> {
                spinMotor.setControl(NeutralOut())
                sidetakeMotor.setControl(NeutralOut())
                uptakeMotor.setControl(NeutralOut())
            }
            State.ON -> {
                spinMotor.setControl(VelocityTorqueCurrentFOC(SPIN_VELOCITY))
                sidetakeMotor.setControl(VelocityTorqueCurrentFOC(SIDETAKE_VELOCITY))
                uptakeMotor.setControl(VelocityTorqueCurrentFOC(UPTAKE_VELOCITY))
            }
            State.SPITTING -> {
                spinMotor.setControl(VelocityTorqueCurrentFOC(SPIN_SPIT_VELOCITY))
                sidetakeMotor.setControl(VelocityTorqueCurrentFOC(SIDETAKE_SPIT_VELOCITY))
                uptakeMotor.setControl(VelocityTorqueCurrentFOC(UPTAKE_SPIT_VELOCITY))
            }
        }
    }


    enum class State {
        OFF,
        ON,
        SPITTING
    }
}