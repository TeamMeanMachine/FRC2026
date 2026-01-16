package frc.team2471.frc2026

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.units.absoluteValue
import org.team2471.frc.lib.units.asInches
import org.team2471.frc.lib.units.asInchesPerSecond
import org.team2471.frc.lib.units.asMetersPerSecond
import org.team2471.frc.lib.units.asVolts
import org.team2471.frc.lib.units.cos
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.inchesPerSecond
import org.team2471.frc.lib.units.seconds
import org.team2471.frc.lib.units.sin
import org.team2471.frc.lib.units.volts
import org.team2471.frc.lib.units.voltsPerSecond

object Shooter: SubsystemBase("Shooter") {
    val table = NetworkTableInstance.getDefault().getTable("Shooter")

    val shooterMotor = TalonFX(Falcons.SHOOTER_0)

    val WHEEL_DIAMETER = 4.0.inches

    @get:AutoLogOutput(key = "Shooter/Shooter Setpoint")
    var shooterVelocitySetpoint: LinearVelocity = 0.0.inchesPerSecond
        set(value) {
            field = value
            shooterMotor.setControl(VelocityVoltage(field.asInchesPerSecond/(WHEEL_DIAMETER.asInches * Math.PI)))
        }
    @get:AutoLogOutput(key = "Shooter/Shooter Velocity")
    val shooterVelocity: LinearVelocity
        get() = (shooterMotor.velocity.valueAsDouble * WHEEL_DIAMETER.asInches * Math.PI).inchesPerSecond

    @get:AutoLogOutput(key = "Shooter/Shooter Current")
    val shooterCurrent: Double
        get() = shooterMotor.supplyCurrent.valueAsDouble

    var fuel: MutableList<FlyingFuel> = mutableListOf()

    init {
        shooterMotor.applyConfiguration {
            currentLimits(25.0, 30.0, 1.0)
            coastMode()

            p(0.0)
            s(0.0, StaticFeedforwardSignValue.UseVelocitySign)
        }
    }





    override fun periodic() {
        fuel.forEach { it.update() }
        logFuel("fuel", *fuel.toTypedArray())
        fuel.removeFuel()
    }

    fun shootSimulatedFuel(exitVelocity: Double, exitAngle: Angle) {
        val velocity2d = Translation2d(-exitVelocity * exitAngle.cos(), 0.0).rotateBy(Drive.heading)
        val posOffset2d =Translation2d(0.218, 0.0).rotateBy(Drive.heading)
        fuel.add(FlyingFuel(
            Translation3d(Drive.pose.translation.x + posOffset2d.x, Drive.pose.translation.y + posOffset2d.y, 0.4),
            Translation3d(velocity2d.x + Drive.velocity.x.asMetersPerSecond, velocity2d.y + Drive.velocity.y.asMetersPerSecond, exitVelocity * exitAngle.sin())
        ))
    }

    val sysIDShooterRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            1.0.voltsPerSecond,
            7.0.volts,
            5.0.seconds
        ) { state: SysIdRoutineLog.State ->
            SignalLogger.writeString("SysIdShooterLeft_State", state.toString())
            Logger.recordOutput("SysIdShooterLeft_State", state.toString())
            Logger.recordOutput("Shooter_Left_Position", shooterMotor.position.valueAsDouble)
            Logger.recordOutput("Shooter_Left_Velocity", shooterMotor.velocity.valueAsDouble)
        },
        SysIdRoutine.Mechanism({ output: Voltage ->
            shooterMotor.setControl(VoltageOut(output.asVolts))
            /* also log the requested output for SysId */
            Logger.recordOutput("Shooter_Left_Voltage", output.asVolts + 0.0001 * Math.random())
        }, null, this)
    )
}