package frc.team2471.frc2026

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.frc2026.OI.driverController
import gg.questnav.questnav.QuestNav
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.LoopLogger
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.ctre.PhoenixUtil
import org.team2471.frc.lib.localization.PoseLocalizer
import org.team2471.frc.lib.math.cube
import org.team2471.frc.lib.math.square
import org.team2471.frc.lib.swerve.SwerveDriveSubsystem
import org.team2471.frc.lib.units.asMetersPerSecondPerSecond
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.math.DynamicInterpolatingTreeMap
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.inchesPerSecond
import org.team2471.frc.lib.units.metersPerSecondPerSecond
import org.team2471.frc.lib.units.perSecond
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.units.unWrap
import org.team2471.frc.lib.util.demoSpeed
import org.team2471.frc.lib.util.isBlueAlliance
import org.team2471.frc.lib.util.isReal
import org.team2471.frc.lib.vision.Fiducial
import org.team2471.frc.lib.vision.PipelineConfig
import org.team2471.frc.lib.vision.QuixVisionCamera
import org.team2471.frc.lib.vision.photonVision.PhotonVisionCamera
import kotlin.math.atan2


object Drive: SwerveDriveSubsystem(TunerConstants.drivetrainConstants, *TunerConstants.moduleConfigs) {

    // To reset position use this, also add other pose sources that need reset here.
    override var pose: Pose2d
        get() = savedState.Pose
        set(value) {
            tempQuestPose = Pose3d(value).transformBy(robotToQuestTransformMeters)
            resetPose(value)
            localizer.resetPose(value) // Possibly not needed, but good for a quick response.
        }

    override var heading: Rotation2d
        get() = pose.rotation
        set(value) {
//            println("resting heading to ${value.degrees}")
            resetRotation(value)
            localizer.resetRotation(value) // Not needed and redundant but may prevent some heading bugs
            val tempQuestPose = tempQuestPose
            if (tempQuestPose != null) {
                quest.setPose(tempQuestPose)
                this.tempQuestPose = null
            } else {
                quest.setPose(Pose3d(questPose.translation, Rotation3d(value)).transformBy(robotToQuestTransformMeters))
            }
            resetPoseTime = Timer.getFPGATimestamp()
            Turret.setTurretOffset(value.measure)
        }

    var headingAngleUnwrapped: Angle = heading.measure
        get() = heading.measure.unWrap(field)

    val cameras: List<QuixVisionCamera> = listOf(
        /*  +x
         * front *
        +y   o   -y
         *  back *
             -x      */
        PhotonVisionCamera("FrontLeft", Transform3d(Translation3d(-12.5.inches.asMeters, 13.1.inches.asMeters, 21.0.inches.asMeters), Rotation3d(0.0, -25.0.degrees.asRadians, 45.0.degrees.asRadians)), arrayOf(PipelineConfig())),
        PhotonVisionCamera("FrontRight", Transform3d(Translation3d(-12.5.inches.asMeters, -13.1.inches.asMeters, 21.0.inches.asMeters), Rotation3d(0.0, -25.0.degrees.asRadians, -45.0.degrees.asRadians)), arrayOf(PipelineConfig())),
        PhotonVisionCamera("BackLeft", Transform3d(Translation3d(-13.7.inches.asMeters, 10.7.inches.asMeters, 21.0.inches.asMeters), Rotation3d(0.0, -25.0.degrees.asRadians, 130.0.degrees.asRadians)), arrayOf(PipelineConfig())),
        PhotonVisionCamera("BackRight", Transform3d(Translation3d(-13.7.inches.asMeters, -10.7.inches.asMeters, 21.0.inches.asMeters), Rotation3d(0.0, -25.0.degrees.asRadians, -130.0.degrees.asRadians)), arrayOf(PipelineConfig())),
    )

    val headingHistory: DynamicInterpolatingTreeMap<Double, Double> = DynamicInterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble(), 75)

    private var tempQuestPose: Pose3d? = null
    private var resetPoseTime = 0.0

    val quest = QuestNav()

    var simulateQuest = true
    @get:AutoLogOutput(key = "Drive/Quest/isConnected")
    val questConnected: Boolean
        get() = if (isReal) quest.isConnected else simulateQuest
    @get:AutoLogOutput(key = "Drive/Quest/isTracking")
    val questTracking: Boolean
        get() = if (isReal) quest.isTracking else simulateQuest
    @get:AutoLogOutput(key = "Drive/Quest/isTrackingMaybe")
    var questTrackingMaybe: Boolean = false
    val robotToQuestTransformMeters = Transform3d(-12.5.inches.asMeters, -12.5.inches.asMeters, 12.5.inches.asMeters, Rotation3d(90.0.degrees, 0.0.degrees, 180.0.degrees))

    var questPose: Pose3d = Pose3d()
        private set

    val DRIVE_STD_DEVS: Matrix<N3?, N1?> = VecBuilder.fill(0.1, 0.1, 0.05)

    // Trust down to 2 cm in XY and 2 degrees in rotational. Units in meters and radians.
    val QUEST_STD_DEVS: Matrix<N3?, N1?> = VecBuilder.fill(0.025, 0.025, 99999.9)

    // TODO: Check heading accuracy
    val localizer = PoseLocalizer(Fiducial.constructFiducialList(FieldManager.allAprilTags), cameras)

    private val translationRateTimer = Timer()
    private var prevTranslation = Translation2d()

    // Drive Feedback controllers
    override val autoPilot = createAPObject(Double.POSITIVE_INFINITY.inchesPerSecond, 100.0.metersPerSecondPerSecond, 2.0.metersPerSecondPerSecond.perSecond, 0.5.inches, 1.0.degrees)
    val fastAutoPilot = createAPObject(Double.POSITIVE_INFINITY.inchesPerSecond, 100.0.metersPerSecondPerSecond, 5.0.metersPerSecondPerSecond.perSecond, 0.5.inches, 1.0.degrees)
    val slowAutoPilot = createAPObject(Double.POSITIVE_INFINITY.inchesPerSecond, 100.0.metersPerSecondPerSecond, 0.5.metersPerSecondPerSecond.perSecond, 0.25.inches, 1.0.degrees)

    override val pathXController = PIDController(7.0, 0.0, 0.0)
    override val pathYController = PIDController(7.0, 0.0, 0.0)
    override val pathThetaController = PIDController(7.0, 0.0, 0.0)

    override val autoDriveToPointController = PIDController(3.0, 0.0, 0.1)
    override val teleopDriveToPointController = PIDController(3.0, 0.0, 0.1)

    override val driveAtAnglePIDController = PhoenixPIDController(7.7, 0.0, 0.072)

    override val isDisabledSupplier: () -> Boolean = { Robot.isDisabled }

    override val choreoPathsStartOnRed: Boolean = false // false=made on the blue side, true=made on the red side

    var latestAppliedQuestDataTimestamp = 0.0
    var latestAppliedQuestCTRETimestamp = 0.0
    var asyncPeriodicTime = 0.0

    init {
        println("inside Drive init")

        // MUST start inside the field on bootup for accurate heading measurements due to a PoseLocalizer bug.
        pose = Pose2d(3.0, 3.0, heading)

        setStateStdDevs(DRIVE_STD_DEVS)

//        zeroGyro()

        println("max acceleration ${TunerConstants.kMaxAcceleration.asMetersPerSecondPerSecond}")

        localizer.trackAllTags()

        finalInitialization()
        // Launch a new thread and run vision loop
        GlobalScope.launch {
            org.team2471.frc.lib.coroutines.periodic {
                try {
                    asyncPeriodicTime = LoopLogger.measureTimeFPGA {
                        // Apply quest measurements
//                    if (questConnected && questTracking && tempQuestPose == null) {
//                        if (isReal) {
//                            quest.allUnreadPoseFrames.forEach {
//                                questTrackingMaybe = it.isTracking
//                                if (resetPoseTime < it.dataTimestamp && it.isTracking) {
//                                    val pose = it.questPose3d.transformBy(robotToQuestTransformMeters.inverse())
//                                    val ctreTimestamp = Utils.fpgaToCurrentTime(it.dataTimestamp)
//
//                                    latestAppliedQuestDataTimestamp = it.dataTimestamp
//                                    latestAppliedQuestCTRETimestamp = ctreTimestamp
//                                    addVisionMeasurement(pose.toPose2d(), ctreTimestamp, QUEST_STD_DEVS)
//                                    questPose = pose
//                                }
//                            }
//                        } else {
//                            // Simulate quest data
//                            addVisionMeasurement(pose, stateTimestamp, QUEST_STD_DEVS)
//                            questPose = Pose3d(pose.x, pose.y, robotToQuestTransformMeters.z, robotToQuestTransformMeters.rotation)
//                        }
//                    }
//
//                    // Update Vision
                        cameras.forEach {
                            it.updateInputs()
                        }
//                    // Update poses with processed particle filter estimates.
                        localizer.updateWithLatestPoseEstimate()
//                    // Create an odom measurement with a timestamp converted from phoenix time to fpga time.
                        val poseMeasurement = PoseLocalizer.OdometryMeasurement(pose, PhoenixUtil.currentToFpgaTime(stateTimestamp))
//                    // Publish the latest camera data to NT and also update pose from swerve odometry measurements.
                        localizer.update(poseMeasurement, cameras.map { it.latestMeasurement }, speeds)
                    }
                } catch (e: Exception) {
                    println("Exception in drive async periodic")
                    println(e)
                }
            }
        }
    }

    override fun periodic() {
        LoopLogger.record("Inside Drive periodic")

        LoopLogger.record("b4 Drive piodc")
        super.periodic() // Must call this
        LoopLogger.record("super Drive piodc")

        quest.commandPeriodic()
        LoopLogger.record("Drive quest periodic")

        headingHistory.put(Timer.getFPGATimestamp(), heading.degrees)
        LoopLogger.record("Recorded HeadingHistory")

        // Log all the poses for debugging
        Logger.recordOutput("Drive/Quest/questPose", questPose)
        Logger.recordOutput("Swerve/Odometry", localizer.odometryPose)
        Logger.recordOutput("Swerve/InterpolatedOdometry", localizer.interpolatedOdometryPose)
        Logger.recordOutput("Swerve/InterpolatedPose", localizer.interpolatedPose)
        Logger.recordOutput("Swerve/Localizer Raw", localizer.rawPose)
        Logger.recordOutput("Swerve/Localizer", localizer.pose)
        Logger.recordOutput("Swerve/SingleTagPose", localizer.singleTagPose)
        Logger.recordOutput("Drive/Quest/DataTimestamp", latestAppliedQuestDataTimestamp)
        Logger.recordOutput("Drive/Quest/CtreTimestamp", latestAppliedQuestCTRETimestamp)
        Logger.recordOutput("Drive/AsyncPeriodicTime", asyncPeriodicTime)

        LoopLogger.record("Drive pirdc")
    }

    /**
     * Returns [ChassisSpeeds] with a percentage power from the driver controller.
     */
    override fun getJoystickPercentageSpeeds(): ChassisSpeeds {
        val rawJoystick = OI.rawDriveTranslation
        // Square drive input and apply demoSpeed
        val power = rawJoystick.norm.square() * demoSpeed * if (Shooter.isShooting) 0.3 else if (inSnakeMode) 0.8 else 1.0
        // Apply modified power to joystick vector and flip depending on alliance
        val joystickTranslation = rawJoystick * power * if (isBlueAlliance) -1.0 else 1.0

        val rawJoystickRotation = OI.driveRotation
        // Cube rotation input and apply demoSpeed
        val omega = rawJoystickRotation.cube() * demoSpeed

        return ChassisSpeeds(joystickTranslation.x, joystickTranslation.y, omega)
    }

    var inSnakeMode = false
    fun snakeMode(): Command = runCommand(Drive) {
        inSnakeMode = true
        if (OI.rawDriveTranslation.norm > 0.1) {
            driveAtAngle(
                atan2(
                    driverController.leftY,
                    -driverController.leftX
                ).radians.asRotation2d - Rotation2d(90.0.degrees)
            )
        } else {
            driveVelocity(getChassisSpeedsFromJoystick().apply { omegaRadiansPerSecond = 0.0 })
        }
    }.finallyRun {
        inSnakeMode = false
    }

    fun resetOdometryToAbsolute() {
        println("resetting odometry to localizer pose")
        val localizerPose = localizer.pose
        pose = Pose2d(localizerPose.translation, pose.rotation)
    }
}