package frc.team2471.frc2026

//import edu.wpi.first.wpilibj2.command.Command
//import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
//import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.MeanLogger
import org.team2471.frc.lib.commands.MechanismBase
import org.team2471.frc.lib.commands.periodic
import org.team2471.frc.lib.commands.use
import org.team2471.frc.lib.control.LoopLogger
//import org.team2471.frc.lib.control.commands.onlyRunWhileFalse
import org.team2471.frc.lib.math.toPose2d
import org.team2471.frc.lib.units.absoluteValue
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.asRotations
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.meters
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.units.unWrap
import org.team2471.frc.lib.units.wrap
import org.team2471.frc.lib.util.angleTo
import org.team2471.frc.lib.util.isReal
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.degreesPerSecond
import org.team2471.frc.lib.units.rotationsPerSecond
import org.team2471.frc.lib.util.PowerTracker
import org.team2471.frc.lib.util.isSim
import org.wpilib.command3.Command
import org.wpilib.math.geometry.Translation2d
import org.wpilib.networktables.NetworkTableInstance
import org.wpilib.units.measure.Angle
import org.wpilib.units.measure.AngularVelocity
import kotlin.math.IEEErem
import kotlin.math.absoluteValue

object Turret: MechanismBase("Turret") {
    private val table = NetworkTableInstance.getDefault().getTable("Turret")
    val encoder1OffsetEntry = table.getEntry("Encoder 1 Offset")
    val encoder2OffsetEntry = table.getEntry("Encoder 2 Offset")
    val disableTurretEntry = table.getEntry("Disable Turret")
    val turretPigeonIsConnectedEntry = table.getEntry("Turret Pigeon IsConnected")
    val rawEncoder1AbsolutePositionEntry = table.getEntry("Raw Encoder 1 Absolute Position")
    val rawEncoder2AbsolutePositionEntry = table.getEntry("Raw Encoder 2 Absolute Position")
    val encoder1AbsolutePositionEntry = table.getEntry("Encoder 1 Absolute Position")
    val encoder2AbsolutePositionEntry = table.getEntry("Encoder 2 Absolute Position")
    val fusedEncoderAngleEntry = table.getEntry("Fused Encoder Angle")
    val turetFeedforwardFactorEntry = table.getEntry("Feedforward Factor")

//    val turretMotor = LoggedTalonFX(Falcons.TURRET_0, CANivores.TURRET_CAN) // TODO: PHOENIX 6 2027
//    val turretEncoder1 = CANcoder(CANCoders.TURRET_0, CANivores.TURRET_CAN)
//    val turretEncoder2 = CANcoder(CANCoders.TURRET_1, CANivores.TURRET_CAN)
//    val turretPigeon = Pigeon2(CANSensors.TURRET_PIGEON, CANivores.TURRET_CAN)

    val TURRET_TOP_LIMIT = if (Robot.isCompBot) 190.0.degrees else 270.0.degrees
    val TURRET_BOTTOM_LIMIT = if (Robot.isCompBot) -190.0.degrees else -270.0.degrees
    val TURRET_RANGE = TURRET_TOP_LIMIT - TURRET_BOTTOM_LIMIT
    val TURRET_ENCODER_LIMIT = if (Robot.isCompBot) 600.0.degrees else 720.0.degrees

    val ENCODER_1_DEFAULT_OFFSET = if (Robot.isCompBot) 164.004 else 43.0664
    val ENCODER_2_DEFAULT_OFFSET = if (Robot.isCompBot) 144.229 else 76.2

    val encoder1GearRatio = if (Robot.isCompBot) 30.0/230.0 else 30.0/200.0
    val encoder2GearRatio = encoder1GearRatio * 83.0/32.0

    val turretZeroPositionOnRobot = if (Robot.isCompBot) 0.0.degrees else 90.0.degrees

    val motorGearRatio = if (Robot.isCompBot) 30.0/230.0 * 11.0/46.0 else 30.0/200.0 * 11.0/46.0

//    @AutoLogOutput(key = "Turret/offset") TODO
    var offset: Angle = 0.0.degrees

//    @get:AutoLogOutput(key = "Turret/rawTurretMotorRotorAngle") TODO
    val rawTurretMotorRotorAngle: Angle get() = 0.0.degrees//turretMotor.rotorPosition.valueAsDouble.rotations * motorGearRatio //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Turret/turretMotorRotorAngleOffset") TODO
    var turretMotorRotorPositionOffset: Angle = 0.0.degrees

//    @get:AutoLogOutput(key = "Turret/turretMotorRotorAngle") TODO
    val turretMotorRotorAngle: Angle
        get() = rawTurretMotorRotorAngle + turretMotorRotorPositionOffset

//    @get:AutoLogOutput(key = "Turret/fieldCentricTurretMotorRotorAngle") TODO
    val fieldCentricTurretMotorRotorAngle: Angle
        get() = ((turretMotorRotorAngle + turretZeroPositionOnRobot) + Drive.headingAngleUnwrapped)

//    @get:AutoLogOutput(key = "Turret/turretMotorVoltage") TODO
    val turretMotorVoltage: Double get() = 0.0//turretMotor.motorVoltage.valueAsDouble //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Turret/rawEncoder1AbsolutePosition") TODO
    val rawEncoder1AbsolutePosition: Angle get() = 0.0.degrees//turretEncoder1.absolutePosition.value //TODO: UNCOMMENT WHEN 2027 PHOENIX 6
//    @get:AutoLogOutput(key = "Turret/rawEncoder2AbsolutePosition") TODO
    val rawEncoder2AbsolutePosition: Angle get() = 0.0.degrees//turretEncoder2.absolutePosition.value //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Turret/encoder1AbsolutePosition") TODO
    val encoder1AbsolutePosition: Angle get() = (rawEncoder1AbsolutePosition - encoder1OffsetEntry.getDouble(ENCODER_1_DEFAULT_OFFSET).degrees).wrap()
//    @get:AutoLogOutput(key = "Turret/encoder2AbsolutePosition") TODO
    val encoder2AbsolutePosition: Angle get() = (rawEncoder2AbsolutePosition - encoder2OffsetEntry.getDouble(ENCODER_2_DEFAULT_OFFSET).degrees).wrap()

    // unwraps encoder 1 angle using encoder 2 angle
//    @get:AutoLogOutput(key = "Turret/fusedEncoderAngle")  TODO
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

//            Logger.recordOutput("Turret/ValidAngles", validAngles.map{it.asDegrees}.toDoubleArray())

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

//            Logger.recordOutput("Turret/Errors", errors.toDoubleArray())

            return bestAngle + offset
        }
//    @get:AutoLogOutput(key = "Turret/FieldCentricFusedEncoderAngle") TODO
    val fieldCentricFusedEncoderAngle: Angle
        get() = ((fusedEncoderAngle + turretZeroPositionOnRobot) + Drive.heading.measure).wrap()

//    @get:AutoLogOutput(key = "Turret/fieldCentricAngle") TODO
    val fieldCentricAngle: Angle
        get() = if (isReal) {
            0.0.degrees//turretMotor.position.value //TODO: UNCOMMENT WHEN 2027 PHOENIX 6
        } else {
            0.0.degrees//turretMotor.position.value + Drive.heading.measure //TODO: UNCOMMENT WHEN 2027 PHOENIX 6
        }

//    @get:AutoLogOutput(key = "Turret/fieldCentricAngleWrapped")  TODO
    val fieldCentricAngleWrapped: Angle get() = fieldCentricAngle.wrap()

    val turretFeedforwardFactor: Double
        get() = turetFeedforwardFactorEntry.getDouble(3.0)

//    @get:AutoLogOutput(key = "Turret/turretFeedforward") TODO
    val turretFeedforward: Double
        get() = -Drive.chassisVelocities.omega.radians.asRotations * 3.0

//    @get:AutoLogOutput(key = "Turret/isTurretWrapping") TODO
    var isTurretWrapping = false

    val useTurretGyro
        get() = true//(turretPigeonIsConnected && turretMotor.fault_RemoteSensorDataInvalid.value) || true

//    @get:AutoLogOutput(key = "Turret/fieldCentricSetpoint") TODO
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
//                    turretMotor.setControl(NeutralOut()) // TODO: PHOENIX 6 2027
                } else if (useTurretGyro) { // Use field-centric gyro
//                    turretMotor.setControl(PositionVoltage(field.asRotations).withFeedForward(turretFeedforward)) // TODO: PHOENIX 6 2027
                } else { // Use robot-centric motor
                    val fieldCentricTurretRotorAngle = fieldCentricTurretMotorRotorAngle
                    val noGyroError = (value.unWrap(fieldCentricTurretRotorAngle) - fieldCentricTurretRotorAngle)
                    val robotCentricNoGyroSetpoint = (turretMotorRotorAngle + noGyroError)
                    val robotCentricNoGyroSetpointWrapped = robotCentricNoGyroSetpoint.asDegrees.IEEErem(TURRET_TOP_LIMIT.asDegrees.absoluteValue + TURRET_BOTTOM_LIMIT.asDegrees.absoluteValue).degrees
                    println("Turret Gyro Disconnect ${robotCentricNoGyroSetpointWrapped.asDegrees}")
                    MeanLogger.recordOutput("Turret/testMotorCentricSetpointDeg", robotCentricNoGyroSetpointWrapped.asDegrees)
//                    turretMotor.setControl(PositionVoltage(robotCentricNoGyroSetpointWrapped.asRotations).withFeedForward(turretFeedforward)) // TODO: PHOENIX 6 2027
                }
            } else {
                field = value.unWrap(fieldCentricAngle)
//                turretMotor.setControl(PositionVoltage((field - Drive.heading.measure).asRotations)) // TODO: PHOENIX 6 2027
            }
        }

//    @get:AutoLogOutput(key = "Turret/turretSetpointError") TODO
    val turretSetpointError: Angle
        get() = fieldCentricSetpoint - fieldCentricAngle
//    @get:AutoLogOutput(key = "Turret/turretSetpointErrorMotor") TODO
    val turretSetpointErrorMotor: Angle
        get() = 0.0.degrees//turretMotor.closedLoopError.valueAsDouble.rotations// TODO: PHOENIX 6 2027

//    @get:AutoLogOutput(key = "Turret/turretVelocity") TODO
    val turretVelocity: AngularVelocity
        get() = 0.0.degreesPerSecond//turretMotor.rotorVelocity.value //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Turret/turretCurrent") TODO
    val turretCurrent: Double
        get() = 0.0//turretMotor.supplyCurrent.valueAsDouble //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

    val disableTurret: Boolean
        get() = disableTurretEntry.getBoolean(false)

    val turretOffsetFromCenter = Translation2d(0.0, 0.725.inches.asMeters)
    var turretHeight = 0.4.meters

    val turretTranslation: Translation2d
        get() = Drive.localizer.pose.translation + turretOffsetFromCenter.rotateBy(Drive.heading)


//    @get:AutoLogOutput(key = "Turret/Turret error distance") TODO
    val turretErrorDistance get() = 0.0.inches//abs(sin(turretMotor.closedLoopError.valueAsDouble.rotations) * AimUtils.distanceToTarget.asInches).inches // TODO: PHOENIX 6 2027

    var tempHeadingResetAngle: Angle? = null

    val turretPigeonIsConnected get() = false//turretPigeon.isConnected && isReal// TODO: PHOENIX 6 2027
    val turretPigeonLatency get() = 0.0//turretPigeon.yaw.timestamp.latency //TODO: UNCOMMENT WHEN 2027 PHOENIX 6

//    @get:AutoLogOutput(key = "Turret/Look Forward Override") TODO
    var lookForwardOverride = false

//    @get:AutoLogOutput(key = "Turret/Resetting Gyro")
//    var resettingGyro = false

    init {
        println("Turret init")
        if (!encoder1OffsetEntry.exists()) encoder1OffsetEntry.setDouble(ENCODER_1_DEFAULT_OFFSET); encoder1OffsetEntry.setPersistent()
        if (!encoder2OffsetEntry.exists()) encoder2OffsetEntry.setDouble(ENCODER_2_DEFAULT_OFFSET); encoder2OffsetEntry.setPersistent()
        if (!turetFeedforwardFactorEntry.exists()) turetFeedforwardFactorEntry.setDouble(turretFeedforwardFactor); turetFeedforwardFactorEntry.setPersistent()
        if (!disableTurretEntry.exists()) disableTurretEntry.setBoolean(disableTurret); disableTurretEntry.setPersistent()

//        turretEncoder1.applyConfiguration { // TODO: PHOENIX 6 2027
//            if (Robot.isCompBot) {
//                inverted(false)
//            } else {
//                inverted(false)
//            }
//        }
//        turretEncoder2.applyConfiguration {
//            if (Robot.isCompBot) {
//                inverted(true)
//            } else {
//                inverted(false)
//            }
//        }
//
//        turretPigeon.applyConfiguration {
//            MountPose.MountPoseYaw = 0.0
//            MountPose.MountPosePitch = 0.0
//            MountPose.MountPoseRoll = if (Robot.isCompBot) 0.0 else -90.0
//        }
//
//        turretMotor.configSim(DCMotor.getKrakenX60(1), 0.01)
//
//        turretMotor.applyConfiguration {
//            currentLimits(20.0, 20.0, 1.0)
//            inverted(false)
//            brakeMode()
//            if (isReal) {
//                if (Robot.isCompBot) {
//                    s(0.1, StaticFeedforwardSignValue.UseClosedLoopSign)
//                    p(55.0)
////                    p(25.0)
//                    d(0.0)
//                } else {
//                    s(0.2, StaticFeedforwardSignValue.UseClosedLoopSign)
//                    p(50.0)
//                    d(0.0)
//                }
//            } else {
//                s(0.13, StaticFeedforwardSignValue.UseClosedLoopSign)
//                p(500.0)
//                d(25.0)
//            }
//
////            motionMagic(0.2, 12.2)
//            if (useTurretGyro) {
////                alternateFeedbackSensor(turretPigeon.deviceID, FeedbackSensorSourceValue.RemotePigeon2_Yaw, motorGearRatio)
//            }
//
//            ClosedLoopGeneral.ContinuousWrap = false
//        }
//        turretMotor.addFollower(Falcons.TURRET_1)

        if (!isSim) {
            PowerTracker.addMotors("Turret", { /*turretMotor.getSupplyCurrent(true).value.asAmps*/0.0 }, 2) //TODO: UNCOMMENT WHEN 2027 PHOENIX 6
        }


//        turretMotor.setPosition(fusedEncoderAngle)
        setTurretOffset(Drive.heading.measure)

        //Loop that updates setpoint for constantly updating wrap limits and feedforward
        GlobalScope.launch {
            periodic {
                if (Robot.isDisabled) {
                    fieldCentricSetpoint = fieldCentricAngle
                } else {
//                    if (turretMotor.controlMode.value in PhoenixUtil.positionControlModes) { // TODO: PHOENIX 6 2027
//                        fieldCentricSetpoint = fieldCentricSetpoint
//                    }
                }
            }
        }

        //Loop that updates the unwrapped robot heading also sets the turret pigeon offset.
        GlobalScope.launch {
            periodic {

                if ((fieldCentricAngle - fieldCentricTurretMotorRotorAngle.unWrap(fieldCentricAngle)).absoluteValue() > 1.0.degrees && turretVelocity.absoluteValue() < 3.0.rotationsPerSecond) {
//                    if (!resettingGyro) {
//                        resettingGyro = true
                    if (isReal) {
                        GlobalScope.launch {
//                            println("setting turret pigeon yaw to motor angle")
//                        println("Detected Error. Trying to change gyro angle from ${fieldCentricAngle.asDegrees.round(3)} to ${fieldCentricTurretMotorRotorAngle.unWrap(fieldCentricAngle).asDegrees.round(3)}")

//                            turretPigeon.setYaw(fieldCentricTurretMotorRotorAngle.unWrap(fieldCentricAngle).asDegrees) // TODO: PHOENIX 6 2027 <- just this line
//                            println("finished setting turret pigeon yaw, status ok: ${status.isOK}")
//                            resettingGyro = false
                        }
                    }
                }

                val tempResetAngle = tempHeadingResetAngle
                if (tempResetAngle != null) {
                    tempHeadingResetAngle = null
                    Drive.headingAngleUnwrapped = tempResetAngle
                    if (isReal) {
                        GlobalScope.launch {
//                        println("setting turret pigeon yaw")
//                            turretPigeon.setYaw(fieldCentricFusedEncoderAngle.unWrap(fieldCentricAngle).asDegrees) // TODO: PHOENIX 6 2027 <- just this line
//                        println("finished setting turret pigeon yaw")
                        }
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
//        Logger.recordOutput("aim target", aimTarget.toPose2d())
        MeanLogger.recordOutput("Turret/turret setpoint pose", turretTranslation.toPose2d(fieldCentricSetpoint.asRotation2d))
        MeanLogger.recordOutput("Turret/turret pose", turretTranslation.toPose2d(fieldCentricAngle.asRotation2d))
        MeanLogger.recordOutput("Turret/distToGoalFeet", aimTarget.getDistance(Drive.localizer.pose.translation).meters.asFeet)
        MeanLogger.recordOutput("Turret/turretPigeonLatency", turretPigeonLatency)
        MeanLogger.recordOutput("Turret/turretPigeonIsConnected", turretPigeonConnected)
        turretPigeonIsConnectedEntry.setBoolean(turretPigeonConnected)
        LoopLogger.record("turret logging")

        rawEncoder1AbsolutePositionEntry.setDouble(rawEncoder1AbsolutePosition.asDegrees)
        rawEncoder2AbsolutePositionEntry.setDouble(rawEncoder2AbsolutePosition.asDegrees)
        encoder1AbsolutePositionEntry.setDouble(encoder1AbsolutePosition.asDegrees)
        encoder2AbsolutePositionEntry.setDouble(encoder2AbsolutePosition.asDegrees)
        if (!Robot.isEnabled) {
            fusedEncoderAngleEntry.setDouble(fusedEncoderAngle.asDegrees)
        }

        LoopLogger.record("turret periodic")
    }

    override fun default() = defaultCommand {
        this.periodic {
            await(aimAtTarget())
        }
    }

    fun aimAtTarget(): Command = use("AimAtTarget", this) {
        if (lookForwardOverride) {
            if (Robot.isEnabled) {
                fieldCentricSetpoint = Drive.heading.measure
            }
        } else {
            val aimingAngle = turretTranslation.angleTo(AimUtils.aimTarget)
            if (Robot.isEnabled) {
                fieldCentricSetpoint = aimingAngle
            }
        }
    }

    fun staticAimAtTarget(): Command = use(this) {
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