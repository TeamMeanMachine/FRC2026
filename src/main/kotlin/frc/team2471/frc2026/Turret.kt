package frc.team2471.frc2026

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
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
import kotlin.math.absoluteValue

object Turret: SubsystemBase("Turret") {

    const val turretRange = 600.0

    val turretMotor = TalonFX(Falcons.TURRET_0)

    val turretEncoder1 = CANcoder(CANCoders.TURRET_1)
    val turretEncoder2 = CANcoder(CANCoders.TURRET_2)

    const val encoder1GearRatio = 30.0/225.0
    const val encoder2GearRatio = encoder1GearRatio * 11.0/46.0

    @get:AutoLogOutput(key = "Turret/encoder1Angle")
    private val encoder1Angle = turretEncoder1.absolutePosition.valueAsDouble

    @get:AutoLogOutput(key = "Turret/encoder2Angle")
    private val encoder2Angle = turretEncoder2.absolutePosition.valueAsDouble

    // unwraps encoder 1 angle using encoder 2 angle
    @get:AutoLogOutput(key = "Turret/fusedEncoderAngle")
    val fusedEncoderAngle: Angle
        get() {
            // generate a list od all possible angles based off of encoder 1
            val validAngles: ArrayList<Double> = arrayListOf()
            var angle = encoder1Angle * encoder1GearRatio
            while (angle <= turretRange/2.0){
                println(angle)
                validAngles.add(angle)
                angle += 360.0 * encoder1GearRatio
            }
            angle = encoder1Angle * encoder1GearRatio - 360.0 * encoder1GearRatio
            while (angle >= -turretRange/2.0){
                println(angle)
                validAngles.add(angle)
                angle -= 360.0 * encoder1GearRatio
            }

//            println(validAngles)

            // using encoder 2 calculate errors of each valid angle
            var minError = Double.MAX_VALUE
            var bestAngle = 0.0
            for (angle in validAngles) {
                val estEncoder2Angle = (angle/encoder2GearRatio) % 360.0
                val error = kotlin.math.abs(encoder2Angle - estEncoder2Angle) % 360.0
//                println("angle: ${angle}, estAngle: ${estEncoder2Angle}, error: ${error}")
                if (error < minError){
                    minError = error
                    bestAngle = angle
                }
            }
            return bestAngle.degrees
        }

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
        Logger.recordOutput("aim target", AimUtils.aimTarget.toPose2d())
        Logger.recordOutput("turret setpoint pose", turretPose.toPose2d(fieldCentricSetpoint.asRotation2d))
    }

    fun aimAtTarget(): Command = run {
        fieldCentricSetpoint =
            turretPose.angleTo(AimUtils.aimTarget)
    }
}