package frc.team2471.frc2026

import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.frc2026.Robot.powerTracker
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.control.commands.onlyRunWhileFalse
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
import org.team2471.frc.lib.ctre.alternateFeedbackSensor
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.units.asAmps
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.rotationsPerSecond
import org.team2471.frc.lib.util.isSim
import kotlin.collections.toDoubleArray
import kotlin.math.IEEErem
import kotlin.math.absoluteValue

object Turret: SubsystemBase("Turret") {
    private val table = NetworkTableInstance.getDefault().getTable("Turret")
    val encoder1Offset = table.getEntry("Turret Encoder 1 Offset")
    val encoder2Offset = table.getEntry("Turret Encoder 2 Offset")
    val disableTurretEntry = table.getEntry("Disable Turret")
    val turretPigeonIsConnectedEntry = table.getEntry("Turret Pigeon IsConnected")

    val turretMotor = LoggedTalonFX(Falcons.TURRET_0, CANivores.TURRET_CAN)
    val turretEncoder1 = CANcoder(CANCoders.TURRET_1, CANivores.TURRET_CAN)
    val turretEncoder2 = CANcoder(CANCoders.TURRET_2, CANivores.TURRET_CAN)
    val turretPigeon = Pigeon2(CANSensors.TURRET_PIGEON, CANivores.TURRET_CAN)

    val TURRET_TOP_LIMIT = 270.0.degrees
    val TURRET_BOTTOM_LIMIT = -270.0.degrees
    val TURRET_RANGE = TURRET_TOP_LIMIT - TURRET_BOTTOM_LIMIT
    val TURRET_ENCODER_LIMIT = 720.0.degrees

    const val ENCODER_1_DEFAULT_OFFSET = 115.576
    const val ENCODER_2_DEFAULT_OFFSET = 106.084

    const val encoder1GearRatio = 30.0/200.0
    const val encoder2GearRatio = encoder1GearRatio * 83.0/32.0

    const val motorGearRatio = 30.0/200.0 * 11.0/46.0

    @AutoLogOutput(key = "Turret/offset")
    var offset: Angle = 0.0.degrees

    @get:AutoLogOutput(key = "Turret/rawTurretMotorRotorAngle")
    val rawTurretMotorRotorAngle: Angle get() = turretMotor.rotorPosition.valueAsDouble.rotations * motorGearRatio

    @get:AutoLogOutput(key = "Turret/turretMotorRotorAngleOffset")
    var turretMotorRotorPositionOffset: Angle = 0.0.degrees

    @get:AutoLogOutput(key = "Turret/turretMotorRotorAngle")
    val turretMotorRotorAngle: Angle
        get() = rawTurretMotorRotorAngle + turretMotorRotorPositionOffset

    @get:AutoLogOutput(key = "Turret/fieldCentricTurretMotorRotorAngle")
    val fieldCentricTurretMotorRotorAngle: Angle
        get() = ((turretMotorRotorAngle + 90.0.degrees) + Drive.headingAngleUnwrapped)

    @get:AutoLogOutput(key = "Turret/turretMotorVoltage")
    val turretMotorVoltage: Double get() = turretMotor.motorVoltage.valueAsDouble

    @get:AutoLogOutput(key = "Turret/rawEncoder1AbsolutePosition")
    val rawEncoder1AbsolutePosition: Angle get() = turretEncoder1.absolutePosition.value
    @get:AutoLogOutput(key = "Turret/rawEncoder2AbsolutePosition")
    val rawEncoder2AbsolutePosition: Angle get() = turretEncoder2.absolutePosition.value

    @get:AutoLogOutput(key = "Turret/encoder1AbsolutePosition")
    val encoder1AbsolutePosition: Angle get() = (rawEncoder1AbsolutePosition - encoder1Offset.getDouble(ENCODER_1_DEFAULT_OFFSET).degrees).wrap()
    @get:AutoLogOutput(key = "Turret/encoder2AbsolutePosition")
    val encoder2AbsolutePosition: Angle get() = (rawEncoder2AbsolutePosition - encoder2Offset.getDouble(ENCODER_2_DEFAULT_OFFSET).degrees).wrap()

    // unwraps encoder 1 angle using encoder 2 angle
    @get:AutoLogOutput(key = "Turret/fusedEncoderAngle")
    val fusedEncoderAngle: Angle
        get() {
            // generate a list of all possible angles based off of encoder 1
            val validAngles: ArrayList<Angle> = arrayListOf()
            var angle = encoder1AbsolutePosition * encoder1GearRatio
            while (angle <= TURRET_ENCODER_LIMIT / 2.0) {
                validAngles.add(angle)
                angle += 360.0.degrees * encoder1GearRatio
            }
            angle = (encoder1AbsolutePosition - 360.0.degrees) * encoder1GearRatio
            while (angle >= -TURRET_ENCODER_LIMIT / 2.0) {
                validAngles.add(angle)
                angle -= 360.0.degrees * encoder1GearRatio
            }

            Logger.recordOutput("Turret/ValidAngles", validAngles.map{it.asDegrees}.toDoubleArray())

            val errors = arrayListOf<Double>()


            // using encoder 2 calculate errors of each valid angle
            var minError = Double.MAX_VALUE.degrees
            var bestAngle = Double.MIN_VALUE.degrees

            for (angle in validAngles) {
                val estEncoder2Angle = (angle / encoder2GearRatio).wrap()
                val error = (encoder2AbsolutePosition - estEncoder2Angle).wrap().absoluteValue()
                errors.add(error.asDegrees)
                if (error < minError){
                    minError = error
                    bestAngle = angle
                }
            }

            Logger.recordOutput("Turret/Errors", errors.toDoubleArray())

            return bestAngle + offset
        }
    @get:AutoLogOutput(key = "Turret/FieldCentricFusedEncoderAngle")
    val fieldCentricFusedEncoderAngle: Angle
        get() = ((fusedEncoderAngle + 90.0.degrees) + Drive.heading.measure).wrap()

    @get:AutoLogOutput(key = "Turret/fieldCentricAngle")
    val fieldCentricAngle: Angle
        get() = if (isReal) {
            turretMotor.position.value
        } else {
            turretMotor.position.value + Drive.heading.measure
        }

    @get:AutoLogOutput(key = "Turret/fieldCentricAngleWrapped")
    val fieldCentricAngleWrapped: Angle get() = fieldCentricAngle.wrap()

    @get:AutoLogOutput(key = "Turret/turretFeedforward")
    val turretFeedforward: Double
        get() = -Drive.speeds.omegaRadiansPerSecond.radians.asRotations * 3.0

    @get:AutoLogOutput(key = "Turret/isTurretWrapping")
    var isTurretWrapping = false

    val useTurretGyro
        get() = (turretPigeonIsConnected && turretMotor.fault_RemoteSensorDataInvalid.value) || true

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

                if (disableTurret) {
                    turretMotor.setControl(NeutralOut())
                } else if (useTurretGyro) { // Use field-centric gyro
                    turretMotor.setControl(PositionVoltage(field.asRotations).withFeedForward(turretFeedforward))
                } else { // Use robot-centric motor
                    val robotCentricNoGyroSetpoint = (turretMotorRotorAngle + (value - fieldCentricTurretMotorRotorAngle.unWrap(value)))
                    val robotCentricNoGyroSetpointWrapped = robotCentricNoGyroSetpoint.asDegrees.IEEErem(TURRET_TOP_LIMIT.asDegrees.absoluteValue + TURRET_BOTTOM_LIMIT.asDegrees.absoluteValue).degrees
                    println("Turret Gyro Disconnect ${robotCentricNoGyroSetpointWrapped.asDegrees}")
                    turretMotor.setControl(PositionVoltage(robotCentricNoGyroSetpointWrapped.asRotations).withFeedForward(turretFeedforward))
                }
            } else {
                field = value.unWrap(fieldCentricAngle)
                turretMotor.setControl(PositionVoltage(field - Drive.heading.measure))
            }
        }

    @get:AutoLogOutput(key = "Turret/turretSetpointError")
    val turretSetpointError: Angle
        get() = fieldCentricSetpoint - fieldCentricAngle
    @get:AutoLogOutput(key = "Turret/turretSetpointErrorMotor")
    val turretSetpointErrorMotor: Angle
        get() = turretMotor.closedLoopError.valueAsDouble.rotations

    @get:AutoLogOutput(key = "Turret/turretVelocity")
    val turretVelocity: AngularVelocity
        get() = turretMotor.rotorVelocity.value

    @get:AutoLogOutput(key = "Turret/turretCurrent")
    val turretCurrent: Double
        get() = turretMotor.supplyCurrent.valueAsDouble

    val disableTurret: Boolean
        get() = disableTurretEntry.getBoolean(false)

    val turretOffsetFromCenter = Translation2d(0.0.inches, 0.725.inches)
    var turretHeight = 0.4.meters

    val turretTranslation: Translation2d
        get() = Drive.localizer.pose.translation + turretOffsetFromCenter.rotateBy(Drive.heading)


    @get:AutoLogOutput(key = "Turret/Turret error distance")
    val turretErrorDistance get() = abs(sin(turretMotor.closedLoopError.valueAsDouble.rotations) * AimUtils.distanceToTarget.asInches).inches

    var tempHeadingResetAngle: Angle? = null

    val turretPigeonIsConnected get() = turretPigeon.isConnected && isReal


    init {
        println("Turret init")
        if (!encoder1Offset.exists()) encoder1Offset.setDouble(ENCODER_1_DEFAULT_OFFSET); encoder1Offset.setPersistent()
        if (!encoder2Offset.exists()) encoder2Offset.setDouble(ENCODER_2_DEFAULT_OFFSET); encoder2Offset.setPersistent()
        disableTurretEntry.setBoolean(false)



        turretEncoder1.applyConfiguration {
            inverted(false)
        }
        turretEncoder2.applyConfiguration {
            inverted(false)
        }

        turretPigeon.applyConfiguration {
            MountPose.MountPoseYaw = 0.0
            MountPose.MountPosePitch = 0.0
            MountPose.MountPoseRoll = -90.0
        }

        turretMotor.configSim(DCMotor.getKrakenX60(1), 0.01)

        turretMotor.applyConfiguration {
            currentLimits(20.0, 20.0, 1.0)
            inverted(false)
//            brakeMode()
            coastMode()
            if (isReal) {
                s(0.2, StaticFeedforwardSignValue.UseClosedLoopSign)
                p(40.0)
                d(0.0)
            } else {
                s(0.13, StaticFeedforwardSignValue.UseClosedLoopSign)
                p(500.0)
                d(25.0)
            }

//            motionMagic(0.2, 12.2)
            alternateFeedbackSensor(turretPigeon.deviceID, FeedbackSensorSourceValue.RemotePigeon2Yaw, motorGearRatio)

            ClosedLoopGeneral.ContinuousWrap = false
        }
        turretMotor.addFollower(Falcons.TURRET_1)

        if (!isSim) {
            powerTracker.addMotors("Turret", { turretMotor.getSupplyCurrent(true).value.asAmps }, 2)
        }


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

                if ((fieldCentricAngle - fieldCentricTurretMotorRotorAngle.unWrap(fieldCentricAngle)).absoluteValue() > 1.0.degrees && turretVelocity.absoluteValue() < 3.0.rotationsPerSecond) {
                    GlobalScope.launch {
//                        println("setting turret pigeon yaw to motor angle")
//                        println("Detected Error. Trying to change gyro angle from ${fieldCentricAngle.asDegrees.round(3)} to ${fieldCentricTurretMotorRotorAngle.unWrap(fieldCentricAngle).asDegrees.round(3)}")

                        turretPigeon.setYaw(fieldCentricTurretMotorRotorAngle.unWrap(fieldCentricAngle))
//                        println("finished setting turret pigeon yaw")
                    }
                }

                val tempResetAngle = tempHeadingResetAngle
                if (tempResetAngle != null) {
                    tempHeadingResetAngle = null
                    Drive.headingAngleUnwrapped = tempResetAngle
                    GlobalScope.launch {
//                        println("setting turret pigeon yaw")
                        turretPigeon.setYaw(fieldCentricFusedEncoderAngle.unWrap(fieldCentricAngle))
//                        println("finished setting turret pigeon yaw")
                    }
                }
                Drive.headingAngleUnwrapped = Drive.heading.measure.unWrap(Drive.headingAngleUnwrapped)
            }
        }

        zeroTurretMotor()
    }

    override fun periodic() {
        LoopLogger.record("b4 turret periodic")
        val aimTarget = AimUtils.aimTarget
        val turretTranslation = turretTranslation
        val turretPigeonConnected = turretPigeonIsConnected
        Logger.recordOutput("aim target", aimTarget.toPose2d())
        Logger.recordOutput("Turret/turret setpoint pose", turretTranslation.toPose2d(fieldCentricSetpoint.asRotation2d))
        Logger.recordOutput("Turret/turret pose", turretTranslation.toPose2d(fieldCentricAngle.asRotation2d))
        Logger.recordOutput("Turret/distToGoalFeet", aimTarget.getDistance(Drive.localizer.pose.translation).meters.asFeet)
        Logger.recordOutput("Turret/turretPigeonIsConnected", turretPigeonConnected)
        turretPigeonIsConnectedEntry.setBoolean(turretPigeonConnected)

        LoopLogger.record("turret periodic")
    }

    fun aimAtTarget(): Command = run {
        fieldCentricSetpoint = turretTranslation.angleTo(AimUtils.aimTarget)
    }.onlyRunWhileFalse { Robot.isTestEnabled && Drive.useAprilTags }

    fun staticAimAtTarget(): Command = run {
        fieldCentricSetpoint = AimUtils.staticShotPos.angleTo(AimUtils.aimTarget)
    }

    fun setTurretOffset(robotHeading: Angle) {
        tempHeadingResetAngle = robotHeading
    }
    fun zeroTurretMotor() {
        turretMotorRotorPositionOffset = fusedEncoderAngle - rawTurretMotorRotorAngle
        println("Zeroing turret motor")
    }
}