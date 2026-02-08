package frc.team2471.frc2026

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VelocityVoltage
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

    val spinVelocity = spinVelocityEntry.getDouble(100.0)
    val sidetakeVelocity = sidetakeVelocityEntry.getDouble(100.0)
    val uptakeVelocity = uptakeVelocityEntry.getDouble(100.0)

    val spinSpitVelocity = spinSpitVelocityEntry.getDouble(0.0)
    val sidetakeSpitVelocity = sidetakeSpitVelocityEntry.getDouble(-50.0)
    val uptakeSpitVelocity = uptakeSpitVelocityEntry.getDouble(-50.0)

    init {
        if (!spinVelocityEntry.exists()) spinVelocityEntry.setDouble(spinVelocity)
        if (!sidetakeVelocityEntry.exists()) sidetakeVelocityEntry.setDouble(sidetakeVelocity)
        if (!uptakeVelocityEntry.exists()) uptakeVelocityEntry.setDouble(uptakeVelocity)

        spinVelocityEntry.setPersistent()
        sidetakeVelocityEntry.setPersistent()
        uptakeVelocityEntry.setPersistent()

        if (!spinSpitVelocityEntry.exists()) spinSpitVelocityEntry.setDouble(spinSpitVelocity)
        if (!sidetakeSpitVelocityEntry.exists()) sidetakeSpitVelocityEntry.setDouble(sidetakeSpitVelocity)
        if (!uptakeSpitVelocityEntry.exists()) uptakeSpitVelocityEntry.setDouble(uptakeSpitVelocity)

        spinSpitVelocityEntry.setPersistent()
        sidetakeSpitVelocityEntry.setPersistent()
        uptakeSpitVelocityEntry.setPersistent()

        spinMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(true)
            coastMode()
            s(0.13, StaticFeedforwardSignValue.UseClosedLoopSign)
            p(0.0)

//            Feedback.SensorToMechanismRatio = 1.0 / (10.0 / 233.0)
//            motionMagic(2.1, 12.2)

            ClosedLoopGeneral.ContinuousWrap = true
            withOpenLoopRamps(OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(1.0))
        }
        spinMotor.addFollower(Falcons.SPIN_1)

        uptakeMotor.applyConfiguration {
            currentLimits(25.0, 30.0, 1.0)
            coastMode()

            p(0.0)
            s(0.0, StaticFeedforwardSignValue.UseVelocitySign)
        }

        sidetakeMotor.applyConfiguration {
            currentLimits(25.0, 30.0, 1.0)
            coastMode()

            p(0.0)
            s(0.0, StaticFeedforwardSignValue.UseVelocitySign)
        }
    }


    override fun periodic() {
        when (currentState) {
            State.OFF -> {
                spinMotor.setControl(VelocityVoltage(0.0))
                sidetakeMotor.setControl(VelocityVoltage(0.0))
                uptakeMotor.setControl(VelocityVoltage(0.0))
            }
            State.ON -> {
                spinMotor.setControl(VelocityVoltage(spinVelocity))
                sidetakeMotor.setControl(VelocityVoltage(sidetakeVelocity))
                uptakeMotor.setControl(VelocityVoltage(uptakeVelocity))
            }
            State.SPITTING -> {
                spinMotor.setControl(VelocityVoltage(spinSpitVelocity))
                sidetakeMotor.setControl(VelocityVoltage(sidetakeSpitVelocity))
                uptakeMotor.setControl(VelocityVoltage(uptakeSpitVelocity))
            }
        }
    }


    enum class State {
        OFF,
        ON,
        SPITTING
    }
}