package frc.team2471.frc2026

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.ctre.PhoenixUtil
import org.team2471.frc.lib.ctre.addFollower
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.d
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.loggedTalonFX.LoggedTalonFX
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
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
import kotlin.collections.toDoubleArray

object Turret: SubsystemBase("Turret") {
    private val table = NetworkTableInstance.getDefault().getTable("Turret")
    val encoder1Offset = table.getEntry("Turret Encoder 1 Offset")
    val encoder2Offset = table.getEntry("Turret Encoder 2 Offset")

    val turretMotor = LoggedTalonFX(Falcons.TURRET_0, CANivores.TURRET_CAN)
    val turretEncoder1 = CANcoder(CANCoders.TURRET_1, CANivores.TURRET_CAN)
    val turretEncoder2 = CANcoder(CANCoders.TURRET_2, CANivores.TURRET_CAN)
    val turretPigeon = Pigeon2(CANSensors.TURRET_PIGEON, CANivores.TURRET_CAN)

    val TURRET_TOP_LIMIT = 300.0.degrees
    val TURRET_BOTTOM_LIMIT = -300.0.degrees
    val TURRET_RANGE = TURRET_TOP_LIMIT - TURRET_BOTTOM_LIMIT

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

    @get:AutoLogOutput(key = "Turret/rawEncoder1AngleCumulative")
    val rawEncoder1AngleCumulative get() = turretEncoder1.position.value
    @get:AutoLogOutput(key = "Turret/rawEncoder2AngleCumulative")
    val rawEncoder2AngleCumulative get() = turretEncoder2.position.value

    @get:AutoLogOutput(key = "Turret/offsetEncoder1AngleCumulative")
    val offsetEncoder1AngleCumulative get() = rawEncoder1AngleCumulative - encoder1Offset.getDouble(0.0).degrees
    @get:AutoLogOutput(key = "Turret/offsetEncoder2AngleCumulative")
    val offsetEncoder2AngleCumulative get() = rawEncoder2AngleCumulative - encoder2Offset.getDouble(0.0).degrees

    @get:AutoLogOutput(key = "Turret/multipliedEncoder2AngleCumulative")
    val multipliedEncoder2AngleCumulative get() = offsetEncoder2AngleCumulative * (11.0 / 46.0)

    @get:AutoLogOutput(key = "Turret/rawEncoder1AbsolutePosition")
    val rawEncoder1AbsolutePosition: Angle get() = turretEncoder1.absolutePosition.value
    @get:AutoLogOutput(key = "Turret/rawEncoder2AbsolutePosition")
    val rawEncoder2AbsolutePosition: Angle get() = turretEncoder2.absolutePosition.value

    @get:AutoLogOutput(key = "Turret/encoder1AbsolutePosition")
    val encoder1AbsolutePosition: Angle get() = (rawEncoder1AbsolutePosition - encoder1Offset.getDouble(ENCODER_1_DEFAULT_OFFSET).degrees).wrap()
    @get:AutoLogOutput(key = "Turret/encoder2AbsolutePosition")
    val encoder2AbsolutePosition: Angle get() = (rawEncoder2AbsolutePosition - encoder2Offset.getDouble(ENCODER_2_DEFAULT_OFFSET).degrees).wrap()

    @get:AutoLogOutput(key = "Turret/gyroAngle")
    val gyroAngle get() = turretPigeon.yaw.value

    var encoder1Angles = arrayListOf<Double>()
    var encoder2Angles = arrayListOf<Double>()
    var pigeonAngles = arrayListOf<Double>()

    // unwraps encoder 1 angle using encoder 2 angle
    @get:AutoLogOutput(key = "Turret/fusedEncoderAngle")
    val fusedEncoderAngle: Angle
        get() {
            // generate a list of all possible angles based off of encoder 1
            val validAngles: ArrayList<Angle> = arrayListOf()
            var angle = encoder1AbsolutePosition * encoder1GearRatio
            while (angle <= TURRET_RANGE / 2.0) {
                validAngles.add(angle)
                angle += 360.0.degrees * encoder1GearRatio
            }
            angle = (encoder1AbsolutePosition - 360.0.degrees) * encoder1GearRatio
            while (angle >= -TURRET_RANGE / 2.0) {
                validAngles.add(angle)
                angle -= 360.0.degrees * encoder1GearRatio
            }

            Logger.recordOutput("validAngles", validAngles.map { it.asDegrees }.toDoubleArray())

            val errors: ArrayList<Angle> = arrayListOf()

            // using encoder 2 calculate errors of each valid angle
            var minError = Double.MAX_VALUE.degrees
            var bestAngle = Double.MIN_VALUE.degrees
            for (angle in validAngles) {
                val estEncoder2Angle = (angle / encoder2GearRatio).wrap()
                val error = (encoder2AbsolutePosition - estEncoder2Angle).wrap().absoluteValue()
                errors.add(error)
//                println("angle: ${angle}, estAngle: ${estEncoder2Angle}, error: ${error}")
                if (error < minError){
                    minError = error
                    bestAngle = angle
                }
            }

            Logger.recordOutput("errors", errors.map { it.asDegrees }.toDoubleArray())
            Logger.recordOutput("lowestError", minError)
            Logger.recordOutput("best?", bestAngle)

            return bestAngle
        }
    @get:AutoLogOutput(key = "Turret/FieldCentricFusedEncoderAngle")
    val fieldCentricFusedEncoderAngle: Angle
        get() = (fusedEncoderAngle + Drive.heading.measure).wrap()

    @get:AutoLogOutput(key = "Turret/fieldCentricAngle")
    val fieldCentricAngle: Angle
        get() = if (isReal) {
            turretMotor.position.value
        } else {
            turretMotor.position.value + Drive.heading.measure
        }

    @get:AutoLogOutput(key = "Turret/turretFeedforward")
    val turretFeedforward: Double
        get() = -Drive.speeds.omegaRadiansPerSecond.radians.asRotations * 0.0

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
        println("Turret init")
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
//            alternateFeedbackSensor(turretPigeon.deviceID, FeedbackSensorSourceValue.RemotePigeon2Yaw, (10.0 / 58.0 / 234.0))

            ClosedLoopGeneral.ContinuousWrap = false
        }
        turretMotor.addFollower(Falcons.TURRET_1)

//        turretMotor.setPosition(fusedEncoderAngle)
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
        LoopLogger.record("b4 turret periodic")
        Logger.recordOutput("aim target", AimUtils.aimTarget.toPose2d())
        Logger.recordOutput("turret setpoint pose", turretTranslation.toPose2d(fieldCentricSetpoint.asRotation2d))
        Logger.recordOutput("turret pose", turretTranslation.toPose2d(fieldCentricAngle.asRotation2d))
        LoopLogger.record("turret periodic")
    }

    fun aimAtTarget(): Command = run {
        fieldCentricSetpoint =
            turretTranslation.angleTo(AimUtils.aimTarget)
    }

    fun setTurretOffset(robotHeading: Angle) {
        tempHeadingResetAngle = robotHeading
    }
}