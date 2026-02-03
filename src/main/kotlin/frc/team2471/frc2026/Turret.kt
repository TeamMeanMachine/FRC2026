package frc.team2471.frc2026

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.frc2026.AimUtils.aimTarget
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.math.toPose2d
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.meters
import org.team2471.frc.lib.units.rotations
import org.team2471.frc.lib.units.unWrap
import org.team2471.frc.lib.util.angleTo

object Turret: SubsystemBase("Turret") {
    val turretMotor = TalonFX(Falcons.TURRET_0)

    @get:AutoLogOutput(key = "Turret/fieldCentricAngle")
    val fieldCentricAngle: Angle
        get() = turretMotor.position.valueAsDouble.rotations + Drive.heading.measure

    @get:AutoLogOutput(key = "Turret/fieldCentricSetpoint")
    var fieldCentricSetpoint: Angle = 0.0.degrees
        set(value) {
            field = value.unWrap(fieldCentricAngle)

            turretMotor.setControl(PositionVoltage(field))
        }


    val turretOffsetFromCenter = Translation2d(0.0.inches, 0.725.inches)
    var turretHeight = 0.4.meters

    val turretPose: Translation2d
        get() = Drive.pose.translation + turretOffsetFromCenter.rotateBy(Drive.heading)


    init {
        turretMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(true)
            coastMode()
            s(0.13, StaticFeedforwardSignValue.UseClosedLoopSign)
            p(0.0)

//            Feedback.SensorToMechanismRatio = 1.0 / (10.0 / 233.0)
//            motionMagic(2.1, 12.2)

            ClosedLoopGeneral.ContinuousWrap = true
        }
        turretMotor.addFollower(Falcons.TURRET_1)
    }

    override fun periodic() {
        Logger.recordOutput("aim target", aimTarget.toPose2d())
        Logger.recordOutput("turret setpoint pose", turretPose.toPose2d(fieldCentricSetpoint.asRotation2d))
    }

    fun aimAtTarget(): Command = run {
        fieldCentricSetpoint =
            turretPose.angleTo(aimTarget)
    }
}