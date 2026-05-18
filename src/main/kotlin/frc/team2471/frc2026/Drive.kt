package frc.team2471.frc2026

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController
import frc.team2471.frc2026.OI.driverController
//import edu.wpi.first.wpilibj2.command.Command
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.MeanLogger
import org.team2471.frc.lib.commands.onCancel
import org.team2471.frc.lib.commands.periodic
import org.team2471.frc.lib.commands.use
import org.team2471.frc.lib.control.CurrentLimits
import org.team2471.frc.lib.control.LoopLogger
//import org.team2471.frc.lib.control.commands.finallyRun
//import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.rightStickButton
import org.team2471.frc.lib.ctre.PhoenixUtil
import org.team2471.frc.lib.localization.PoseLocalizer
import org.team2471.frc.lib.math.cube
import org.team2471.frc.lib.math.square
import org.team2471.frc.lib.swerve.SwerveDriveSubsystem
import org.team2471.frc.lib.units.asMetersPerSecondPerSecond
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.math.DynamicInterpolatingTreeMap
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.inchesPerSecond
import org.team2471.frc.lib.units.metersPerSecondPerSecond
import org.team2471.frc.lib.units.perSecond
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.units.unWrap
import org.team2471.frc.lib.util.PowerTracker
import org.team2471.frc.lib.util.demoSpeed
import org.team2471.frc.lib.util.isBlueAlliance
import org.team2471.frc.lib.util.isSim
import org.team2471.frc.lib.vision.Fiducial
import org.team2471.frc.lib.vision.QuixVisionCamera
import org.wpilib.command3.Command
import org.wpilib.driverstation.RobotState
import org.wpilib.math.controller.PIDController
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.interpolation.Interpolator
import org.wpilib.math.interpolation.InverseInterpolator
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.networktables.NetworkTableInstance
import org.wpilib.system.Timer
import org.wpilib.units.measure.Angle
import kotlin.math.atan2


object Drive: SwerveDriveSubsystem(DriveConstants.drivetrainConstants, *DriveConstants.moduleConfigs) {
    private val table = NetworkTableInstance.getDefault().getTable("Drive")

    private val frontLeftConnectedEntry = table.getEntry("FrontLeftConnected")
    private val frontRightConnectedEntry = table.getEntry("FrontRightConnected")
    private val backLeftConnectedEntry = table.getEntry("BackLeftConnected")
    private val backRightConnectedEntry = table.getEntry("BackRightConnected")

    val increaseDriveCurrentEntry = table.getEntry("IncreaseDriveCurrent")
    val increaseDriveCurrent get() = increaseDriveCurrentEntry.getBoolean(false)
    var prevIncreaseDriveCurrent = increaseDriveCurrent

    val useAprilTagsEntry = table.getEntry("UseAprilTags")

    val useAprilTags: Boolean get() = useAprilTagsEntry.getBoolean(true)


    // To reset position use this, also add other pose sources that need reset here.
    override var pose: Pose2d
        get() = Pose2d()//savedState.Pose //TODO: UNCOMMENT WHEN PHOENIX 6 2027 RELEASES
        set(value) {
//            tempQuestPose = Pose3d(value).transformBy(robotToQuestTransformMeters)
            resetPose(value)
            localizer.resetPose(value) // Possibly not needed, but good for a quick response.
        }

    override var heading: Rotation2d
        get() = pose.rotation
        set(value) {
//            println("resting heading to ${value.degrees}")
            resetRotation(value)
            localizer.resetRotation(value) // Not needed and redundant but may prevent some heading bugs
            Turret.setTurretOffset(value.measure)
            resetPoseTime = Timer.getMonotonicTimestamp()
        }

    var headingAngleUnwrapped: Angle = heading.measure
        get() = heading.measure.unWrap(field)

    val cameras: List<QuixVisionCamera> = listOf(
        /*  +x
         * front *
        +y   o   -y
         *  back *
             -x      */
//        PhotonVisionCamera("FrontLeft", Transform3d(Translation3d(-12.5.inches.asMeters, 13.1.inches.asMeters, 21.0.inches.asMeters), Rotation3d(0.0, -25.0.degrees.asRadians, 45.0.degrees.asRadians)), arrayOf(PipelineConfig())),
//        PhotonVisionCamera("FrontRight", Transform3d(Translation3d(-12.5.inches.asMeters, -13.1.inches.asMeters, 21.0.inches.asMeters), Rotation3d(0.0, -25.0.degrees.asRadians, -45.0.degrees.asRadians)), arrayOf(PipelineConfig())),
//        PhotonVisionCamera("BackLeft", Transform3d(Translation3d(-13.7.inches.asMeters, 10.7.inches.asMeters, 21.0.inches.asMeters), Rotation3d(0.0, -25.0.degrees.asRadians, 130.0.degrees.asRadians)), arrayOf(PipelineConfig())),
//        PhotonVisionCamera("BackRight", Transform3d(Translation3d(-13.7.inches.asMeters, -10.7.inches.asMeters, 21.0.inches.asMeters), Rotation3d(0.0, -25.0.degrees.asRadians, -130.0.degrees.asRadians)), arrayOf(PipelineConfig())),
    )

    val cameraDisconnected: Boolean get() = cameras.any { !it.isConnected }

    val headingHistory: DynamicInterpolatingTreeMap<Double, Double> = DynamicInterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble(), 75)

    private var resetPoseTime = 0.0

    // TODO: Check heading accuracy
    val localizer: PoseLocalizer = PoseLocalizer(Fiducial.constructFiducialList(FieldManager.allAprilTags), cameras)

    // Drive Feedback controllers
    override val autoPilot = createAPObject(Double.POSITIVE_INFINITY.inchesPerSecond, 100.0.metersPerSecondPerSecond, 2.0.metersPerSecondPerSecond.perSecond, 0.5.inches, 1.0.degrees)
    val fastAutoPilot = createAPObject(Double.POSITIVE_INFINITY.inchesPerSecond, 100.0.metersPerSecondPerSecond, 5.0.metersPerSecondPerSecond.perSecond, 0.5.inches, 1.0.degrees)
    val slowAutoPilot = createAPObject(Double.POSITIVE_INFINITY.inchesPerSecond, 100.0.metersPerSecondPerSecond, 0.5.metersPerSecondPerSecond.perSecond, 0.25.inches, 1.0.degrees)

    override val pathXController = PIDController(7.0, 0.0, 0.0)
    override val pathYController = PIDController(7.0, 0.0, 0.0)
    override val pathThetaController = PIDController(8.0, 0.0, 0.0)

    override val autoDriveToPointController = PIDController(3.0, 0.0, 0.1)
    override val teleopDriveToPointController = PIDController(3.0, 0.0, 0.1)

    override val driveAtAnglePIDController = PhoenixPIDController(7.7, 0.0, 0.072)

    override val isDisabledSupplier: () -> Boolean = { Robot.isDisabled }

    /** false = paths made on the blue side, true = paths made on the red side */
    override val choreoPathsStartOnRed: Boolean = false

    init {
        println("inside Drive init")

        useAprilTagsEntry.setBoolean(true)
        increaseDriveCurrentEntry.setBoolean(false)

        // MUST start inside the field on bootup for accurate heading measurements due to a PoseLocalizer bug.
        pose = Pose2d(3.0, 3.0, heading)

//        zeroGyro()

        println("max acceleration ${DriveConstants.kMaxAcceleration.asMetersPerSecondPerSecond}")

        localizer.trackAllTags()
        localizer.disableSingleTagCalculation() // for loop times and we dont use it in 2026

        if (!isSim) {
            PowerTracker.addMotors("Drive", {
                var tempTotalDriveCurrent = 0.0
                var tempTotalSteerCurrent = 0.0
//                io.modules.forEach { //TODO: UNCOMMENT WHEN 2027 PHOENIX 6
//                    tempTotalDriveCurrent += it.driveMotor.supplyCurrent.valueAsDouble //TODO: UNCOMMENT WHEN PHOENIX 6 2027 RELEASES
//                    tempTotalSteerCurrent += it.steerMotor.supplyCurrent.valueAsDouble
//                }
                totalSteerCurrent = tempTotalSteerCurrent
                totalDriveCurrent = tempTotalDriveCurrent
                tempTotalDriveCurrent
            })
            PowerTracker.addMotors("Steer", { totalSteerCurrent })
        }


        finalInitialization()
    }

    override fun periodic() {
        LoopLogger.record("Inside Drive periodic")

        if (RobotState.isTeleopEnabled()) {
            if (increaseDriveCurrent != prevIncreaseDriveCurrent) {
                if (increaseDriveCurrent) {
                    setDriveCurrentLimits(DriveConstants.driveMaxCurrentLimits)
                } else {
                    setDriveCurrentLimits(DriveConstants.driveTeleCurrentLimits)
                }

                prevIncreaseDriveCurrent = increaseDriveCurrent
            }
        }

        LoopLogger.record("b4 Drive piodc")
        super.periodic()
        LoopLogger.record("super Drive piodc")

        // Update Vision
        cameras.forEach {
            it.updateInputs()
        }
        LoopLogger.record("Drive camera updateInputs")


//        frontLeftConnectedEntry.setBoolean(cameras[0].isConnected)
//        frontRightConnectedEntry.setBoolean(cameras[1].isConnected)
//        backLeftConnectedEntry.setBoolean(cameras[2].isConnected)
//        backRightConnectedEntry.setBoolean(cameras[3].isConnected)

        LoopLogger.record("Camera Connected Publisher")

        // Update poses with processed particle filter estimates.
        localizer.updateWithLatestPoseEstimate()
        LoopLogger.record("Drive updateWithLatestPose")
        // Create an odom measurement with a timestamp converted from phoenix time to fpga time.
        val poseMeasurement = PoseLocalizer.OdometryMeasurement(pose, PhoenixUtil.currentToFpgaTime(stateTimestamp))
        // Publish the latest camera data to NT and also update pose from swerve odometry measurements.
        localizer.update(poseMeasurement, cameras.map { it.latestMeasurement }, chassisVelocities)
        LoopLogger.record("Drive localizer")

        headingHistory.put(Timer.getMonotonicTimestamp(), heading.degrees)
        LoopLogger.record("Recorded HeadingHistory")

        if (cameras.isNotEmpty()) {
            cameras.forEach {
                table.getEntry("Cameras/${it.cameraName} isConnected").setBoolean(it.isConnected)
                Logger.recordOutput("Drive/Cameras/${it.cameraName} isConnected", it.isConnected)
            }
        }
        LoopLogger.record("Cameras isConnected publish")

        // Log all the poses for debugging
        MeanLogger.recordOutput("Swerve/Odometry", localizer.odometryPose)
        MeanLogger.recordOutput("Swerve/InterpolatedOdometry", localizer.interpolatedOdometryPose)
        MeanLogger.recordOutput("Swerve/InterpolatedPose", localizer.interpolatedPose)
        MeanLogger.recordOutput("Swerve/Localizer Raw", localizer.rawPose)
        MeanLogger.recordOutput("Swerve/Localizer", localizer.pose)
        MeanLogger.recordOutput("Swerve/SingleTagPose", localizer.singleTagPose)


        LoopLogger.record("Drive pirdc")
    }

    override fun default() = defaultCommand {
        await(joystickDrive())
    }

    /**
     * Sets all drive motor current limits to be the passed in [currentLimits].
     */
    fun setDriveCurrentLimits(currentLimits: CurrentLimits) {
        GlobalScope.launch {
//            io.modules.forEach { //TODO: UNCOMMENT WHEN 2027 PHOENIX 6
//                it.driveMotor.modifyConfiguration {
//                    currentLimits(
//                        currentLimits.continuousLimit,
//                        currentLimits.peakLimit,
//                        currentLimits.peakDuration
//                    )
//                }
//            }
        }
    }

    /**
     * Returns [ChassisSpeeds] with a percentage power from the driver controller.
     */
    override fun getJoystickPercentageVelocity(): ChassisVelocities {
        val rawJoystick = OI.rawDriveTranslation
        // Square drive input and apply demoSpeed
        val power = rawJoystick.norm.square() * demoSpeed * if ((Shooter.isShooting || OI.driverController.rightStickButton) && FieldManager.inScoringZone) 0.3 else if (inSnakeMode) 0.8 else 1.0
        // Apply modified power to joystick vector and flip depending on alliance
        val joystickTranslation = rawJoystick * power * if (isBlueAlliance) -1.0 else 1.0

        val rawJoystickRotation = OI.driveRotation
        // Cube rotation input and apply demoSpeed
        val omega = rawJoystickRotation.cube() * demoSpeed

        return ChassisVelocities(joystickTranslation.x, joystickTranslation.y, omega)
    }

    var inSnakeMode = false
    fun snakeMode(): Command = use(Drive) {
        periodic {
            println("snake mode")
            inSnakeMode = true
            if (OI.rawDriveTranslation.norm > 0.1) {
                driveAtAngle(
                    atan2(
                        driverController.leftY,
                        -driverController.leftX
                    ).radians.asRotation2d - Rotation2d(90.0.degrees)
                )
            } else {
                driveVelocity(getChassisSpeedsFromJoystick().apply { omega = 0.0 })
            }
        }
    }.onCancel {
        inSnakeMode = false
    }

    fun zeroGyroCommand() = use {
        println("zero gyro command")
        zeroGyro()
    }

    fun resetOdometryToAbsolute() {
        println("resetting odometry to localizer pose")
        val localizerPose = localizer.pose
        pose = Pose2d(localizerPose.translation, pose.rotation)
    }
}