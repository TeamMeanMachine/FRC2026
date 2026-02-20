package frc.team2471.frc2026

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.ctre.PhoenixUtil
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.alternateFeedbackSensor
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.d
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.loggedTalonFX.LoggedTalonFX
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.math.toPose2d
import org.team2471.frc.lib.units.absoluteValue
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asInches
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.asRotations
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.meters
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.units.rotations
import org.team2471.frc.lib.units.sin
import org.team2471.frc.lib.units.unWrap
import org.team2471.frc.lib.units.wrap
import org.team2471.frc.lib.util.angleTo
import org.team2471.frc.lib.util.isReal
import kotlin.math.abs
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.ctre.coastMode

object Turret: SubsystemBase("Turret") {
    private val table = NetworkTableInstance.getDefault().getTable("Turret")
    private val rawEncoder1AngleEntry = table.getEntry("Raw Encoder 1 Angle")

    const val turretRange = 600.0

    val turretMotor = LoggedTalonFX(Falcons.TURRET_0, CANivores.TURRET_CAN)
    val turretEncoder1 = CANcoder(CANCoders.TURRET_1, CANivores.TURRET_CAN)
    val turretEncoder2 = CANcoder(CANCoders.TURRET_2, CANivores.TURRET_CAN)
    val turretPigeon = Pigeon2(CANSensors.TURRET_PIGEON, CANivores.TURRET_CAN)

    val encoder1Offset = SmartDashboard.getEntry("Turret Encoder 1 Offset")
    val encoder2Offset = SmartDashboard.getEntry("Turret Encoder 2 Offset")

    const val ENCODER_1_DEFAULT_OFFSET = 0.0
    const val ENCODER_2_DEFAULT_OFFSET = 0.0

    const val encoder1GearRatio = 30.0/200.0
    const val encoder2GearRatio = encoder1GearRatio * 11.0/46.0

    @get:AutoLogOutput(key = "Turret/rawTurretMotorRotorAngle")
    val rawTurretMotorRotorAngle: Angle get() = turretMotor.rotorPosition.valueAsDouble.rotations / 27.88

    @get:AutoLogOutput(key = "Turret/turretMotorRotorAngleOffset")
    var turretMotorRotorPositionOffset: Angle = 0.0.degrees

    @get:AutoLogOutput(key = "Turret/turretMotorRotorAngle")
    val turretMotorRotorAngle: Angle
        get() = rawTurretMotorRotorAngle + turretMotorRotorPositionOffset

    @get:AutoLogOutput(key = "Turret/rawEncoder1Angle")
    val rawEncoder1Angle get() = turretEncoder1.absolutePosition.valueAsDouble.rotations

    @get:AutoLogOutput(key = "Turret/rawEncoder2Angle")
    val rawEncoder2Angle get() = turretEncoder2.absolutePosition.valueAsDouble.rotations

    @get:AutoLogOutput(key = "Turret/encoder1Angle")
    private val encoder1Angle get() = (turretEncoder1.absolutePosition.valueAsDouble.rotations - encoder1Offset.getDouble(ENCODER_1_DEFAULT_OFFSET).degrees).asDegrees.mod(360.0).degrees

    @get:AutoLogOutput(key = "Turret/encoder2Angle")
    private val encoder2Angle get() = (turretEncoder2.absolutePosition.valueAsDouble.rotations - encoder2Offset.getDouble(ENCODER_2_DEFAULT_OFFSET).degrees).asDegrees.mod(360.0).degrees

    // unwraps encoder 1 angle using encoder 2 angle
    @get:AutoLogOutput(key = "Turret/fusedEncoderAngle")
    val fusedEncoderAngle: Angle
        get() {
            // generate a list of all possible angles based off of encoder 1
            val validAngles: ArrayList<Double> = arrayListOf()
            var angle = encoder1Angle.asDegrees * encoder1GearRatio
            while (angle <= turretRange/2.0){
                validAngles.add(angle)
                angle += 360.0 * encoder1GearRatio
            }
            angle = encoder1Angle.asDegrees * encoder1GearRatio - 360.0 * encoder1GearRatio
            while (angle >= -turretRange/2.0){
                validAngles.add(angle)
                angle -= 360.0 * encoder1GearRatio
            }

            Logger.recordOutput("validAngles", validAngles.toDoubleArray())

            val errors: ArrayList<Double> = arrayListOf()

            // using encoder 2 calculate errors of each valid angle
            var minError = Double.MAX_VALUE
            var bestAngle = 0.0
            for (angle in validAngles) {
                val estEncoder2Angle = (angle/encoder2GearRatio) % 360.0
                // rounded because of floating point errors
                var error = kotlin.math.abs(encoder2Angle.asDegrees - estEncoder2Angle).round(3) % 360.0
                if (error > 180.0) error = 360.0 - error
                errors.add(error)
//                println("angle: ${angle}, estAngle: ${estEncoder2Angle}, error: ${error}")
                if (error < minError){
                    minError = error
                    bestAngle = angle
                }
            }
            Logger.recordOutput("errors", errors.toDoubleArray())
            Logger.recordOutput("lowestError", minError)
            Logger.recordOutput("best?", bestAngle)
            return bestAngle.degrees
        }
    @get:AutoLogOutput(key = "Turret/FieldCentricFusedEncoderAngle")
    val fieldCentricFusedEncoderAngle: Angle
        get() = (fusedEncoderAngle + Drive.heading.measure).wrap()

    @get:AutoLogOutput(key = "Turret/fieldCentricAngle")
    val fieldCentricAngle: Angle
        get() = if (isReal) {
            turretMotor.position.valueAsDouble.rotations
        } else {
            turretMotor.position.valueAsDouble.rotations + Drive.heading.measure
        }

    @get:AutoLogOutput(key = "Turret/turretFeedforward")
    val turretFeedforward: Double
        get() = -Drive.speeds.omegaRadiansPerSecond.radians.asRotations * 2.5

    val TURRET_TOP_LIMIT = 300.0.degrees
    val TURRET_BOTTOM_LIMIT = -300.0.degrees

    @get:AutoLogOutput(key = "Turret/isTurretWrapping")
    var isTurretWrapping = false

    @get:AutoLogOutput(key = "Turret/fieldCentricSetpoint")
    var fieldCentricSetpoint: Angle = fieldCentricAngle
        set(value) {
            if (isReal) {
                val turretMotorFieldCentricAngle = fieldCentricAngle
                val fieldCentricSetpoint = if (isTurretWrapping) {
                    value.unWrap(field)
                } else {
                    value.unWrap(turretMotorFieldCentricAngle)
                }
                val positionError = fieldCentricSetpoint - turretMotorFieldCentricAngle
                val robotCentricSetpoint = turretMotorRotorAngle + positionError

                field = if (robotCentricSetpoint > TURRET_TOP_LIMIT && !isTurretWrapping) {
                    fieldCentricSetpoint - 360.0.degrees
                } else if (robotCentricSetpoint < TURRET_BOTTOM_LIMIT && !isTurretWrapping) {
                    fieldCentricSetpoint + 360.0.degrees
                } else {
                    fieldCentricSetpoint
                }

                //Wrapping if pose error is more than half a rotation
                isTurretWrapping = (field - turretMotorFieldCentricAngle).absoluteValue() > 180.0.degrees

//                turretMotor.setControl(PositionVoltage(field.asRotations).withFeedForward(turretFeedforward))
            } else {
                field = value.unWrap(fieldCentricAngle)
//                turretMotor.setControl(PositionVoltage(field - Drive.heading.measure))
            }
        }

    @get:AutoLogOutput(key = "Turret/turretSetpointError")
    val turretSetpointError: Angle
        get() = fieldCentricSetpoint - fieldCentricAngle
    @get:AutoLogOutput(key = "Turret/turretSetpointErrorMotor")
    val turretSetpointErrorMotor: Angle
        get() = turretMotor.closedLoopError.valueAsDouble.rotations


    val turretOffsetFromCenter = Translation2d(0.0.inches, 0.725.inches)
    var turretHeight = 0.4.meters

    val turretTranslation: Translation2d
        get() = Drive.pose.translation + turretOffsetFromCenter.rotateBy(Drive.heading)


    @get:AutoLogOutput(key = "Turret/Turret error distance")
    val turretErrorDistance get() = abs(sin(turretMotor.closedLoopError.valueAsDouble.rotations) * AimUtils.distanceToGoal.asInches).inches

    var tempHeadingResetAngle: Angle? = null


    init {
        if (!encoder1Offset.exists()) encoder1Offset.setDouble(ENCODER_1_DEFAULT_OFFSET); encoder1Offset.setPersistent()
        if (!encoder2Offset.exists()) encoder2Offset.setDouble(ENCODER_2_DEFAULT_OFFSET); encoder2Offset.setPersistent()

        turretMotor.configSim(DCMotor.getKrakenX60(1), 0.01)

        turretMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(true)
//            brakeMode()
            coastMode()
            if (isReal) {
                s(0.0, StaticFeedforwardSignValue.UseClosedLoopSign)
                p(0.0)
                d(0.0)
            } else {
                s(0.13, StaticFeedforwardSignValue.UseClosedLoopSign)
                p(500.0)
                d(25.0)
            }

//            motionMagic(2.1, 12.2)
            alternateFeedbackSensor(turretPigeon.deviceID, FeedbackSensorSourceValue.RemotePigeon2Yaw, (10.0 / 58.0 / 234.0))

            ClosedLoopGeneral.ContinuousWrap = false
        }
        turretMotor.addFollower(Falcons.TURRET_1)

        turretMotor.setPosition(fusedEncoderAngle)
        setTurretOffset(Drive.heading.measure)

        //Loop that updates setpoint for constantly updating wrap limits and feedforward
        GlobalScope.launch {
            periodic {
                if (Robot.isDisabled) {
                    fieldCentricSetpoint = fieldCentricAngle
                } else {
                    if (turretMotor.controlMode.value in PhoenixUtil.positionControlModes) {
                        fieldCentricSetpoint = fieldCentricSetpoint
                    }
                }

                rawEncoder1AngleEntry.setDouble(rawEncoder1Angle.asDegrees)
            }
        }

        //Loop that updates the unwrapped robot heading also sets the turret pigeon offset.
        GlobalScope.launch {
            periodic {
                val tempResetAngle = tempHeadingResetAngle
                if (tempResetAngle != null) {
                    tempHeadingResetAngle = null
                    Drive.headingAngleUnwrapped = tempResetAngle
                    GlobalScope.launch {
                        println("setting turret pigeon yaw")
                        turretPigeon.setYaw(fieldCentricFusedEncoderAngle)
                        println("finished setting turret pigeon yaw")
                    }
                }
                Drive.headingAngleUnwrapped = Drive.heading.measure.unWrap(Drive.headingAngleUnwrapped)

                if (!turretPigeon.isConnected) {
//                    println("TURRET PIGEON DISCONNECTED!!!!")
                }
            }
        }

        turretMotorRotorPositionOffset = fusedEncoderAngle - rawTurretMotorRotorAngle
    }

    override fun periodic() {
        Logger.recordOutput("aim target", AimUtils.aimTarget.toPose2d())
        Logger.recordOutput("turret setpoint pose", turretTranslation.toPose2d(fieldCentricSetpoint.asRotation2d))
        Logger.recordOutput("turret pose", turretTranslation.toPose2d(fieldCentricAngle.asRotation2d))
    }

    fun aimAtTarget(): Command = run {
        fieldCentricSetpoint =
            turretTranslation.angleTo(AimUtils.aimTarget)
    }

    fun setTurretOffset(robotHeading: Angle) {
        tempHeadingResetAngle = robotHeading
    }
}