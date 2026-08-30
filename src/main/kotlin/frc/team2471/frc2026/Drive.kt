package frc.team2471.frc2026

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController
import kotlinx.coroutines.DelicateCoroutinesApi
import frc.team2471.frc2026.OI.driveLeftTriggerFullPress
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.commands.onCancel
import org.team2471.frc.lib.commands.periodic
import org.team2471.frc.lib.commands.command
import org.team2471.frc.lib.control.CurrentLimits
import org.team2471.frc.lib.logging.LoopLogger
import org.team2471.frc.lib.control.rightStickButton
import org.team2471.frc.lib.hardware.ctre.currentLimits
import org.team2471.frc.lib.hardware.ctre.modifyConfiguration
import org.team2471.frc.lib.environment.demoMode
import org.team2471.frc.lib.environment.demoSpeed
import org.team2471.frc.lib.environment.isBlueAlliance
import org.team2471.frc.lib.localization.PoseLocalizer
import org.team2471.frc.lib.logging.SimpleLogger
import org.team2471.frc.lib.math.cube
import org.team2471.frc.lib.math.square
import org.team2471.frc.lib.swerve.SwerveDriveSubsystem
import org.team2471.frc.lib.units.asMetersPerSecondSquared
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.math.DynamicInterpolatingTreeMap
import org.team2471.frc.lib.math.normalize
import org.team2471.frc.lib.units.asRadiansPerSecond
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.inchesPerSecond
import org.team2471.frc.lib.units.metersPerSecondSquared
import org.team2471.frc.lib.units.perSecond
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.units.unWrap
import org.team2471.frc.lib.vision.Fiducial
import org.team2471.frc.lib.vision.QuixVisionCamera
import org.wpilib.command3.Command
import org.wpilib.driverstation.RobotState
import org.wpilib.math.controller.PIDController
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.interpolation.Interpolator
import org.wpilib.math.interpolation.InverseInterpolator
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.networktables.NetworkTableInstance
import org.wpilib.system.Timer
import org.wpilib.units.measure.Angle
import org.wpilib.math.kinematics.SwerveModuleVelocity
import kotlin.math.absoluteValue
import kotlin.math.atan2


object Drive: SwerveDriveSubsystem(DriveConstants.drivetrainConstants, *DriveConstants.moduleConfigs) {
    private val table = NetworkTableInstance.getDefault().getTable("Drive")

    val useAprilTagsEntry = table.getEntry("UseAprilTags")
    val increaseDriveCurrentEntry = table.getEntry("IncreaseDriveCurrent")

    val increaseDriveCurrent get() = increaseDriveCurrentEntry.getBoolean(false)
    var prevIncreaseDriveCurrent = increaseDriveCurrent
    val useAprilTags: Boolean get() = useAprilTagsEntry.getBoolean(true)

    // To reset position use this, also add other pose sources that need reset here.
    override var pose: Pose2d
        get() = savedState.Pose
        set(value) {
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
    val localizer = PoseLocalizer(Fiducial.constructFiducialList(FieldManager.allAprilTags), cameras)

    // Drive Feedback controllers
    override val autoPilot = createAPObject(Double.POSITIVE_INFINITY.inchesPerSecond, 100.0.metersPerSecondSquared, 2.0.metersPerSecondSquared.perSecond, 0.5.inches, 1.0.degrees)
    val fastAutoPilot = createAPObject(Double.POSITIVE_INFINITY.inchesPerSecond, 100.0.metersPerSecondSquared, 5.0.metersPerSecondSquared.perSecond, 0.5.inches, 1.0.degrees)
    val slowAutoPilot = createAPObject(Double.POSITIVE_INFINITY.inchesPerSecond, 100.0.metersPerSecondSquared, 0.5.metersPerSecondSquared.perSecond, 0.25.inches, 1.0.degrees)

    override val pathXController = PIDController(7.0, 0.0, 0.0)
    override val pathYController = PIDController(7.0, 0.0, 0.0)
    override val pathThetaController = PIDController(8.0, 0.0, 0.0)

    override val autoDriveToPointController = PIDController(3.0, 0.0, 0.1)
    override val teleopDriveToPointController = PIDController(3.0, 0.0, 0.1)

    override val driveAtAnglePIDController = PhoenixPIDController(7.7, 0.0, 0.072)

    /** false = paths made on the blue side, true = paths made on the red side */
    override val choreoPathsStartOnRed: Boolean = false

    override var centerOfRotation: Translation2d = Translation2d(0.0.inches, 0.0.inches)

    init {
        println("Drive initialization")

        useMapleSim = true

        useAprilTagsEntry.setBoolean(true)
        increaseDriveCurrentEntry.setBoolean(false)

        // MUST start inside the field on bootup for accurate heading measurements due to a Particle Filter bug.
        pose = Pose2d(3.0, 3.0, heading)

        println("max acceleration ${DriveConstants.kMaxAcceleration.asMetersPerSecondSquared}")

        localizer.trackAllTags()
        localizer.disableSingleTagCalculation() // for loop times and we dont use it in 2026
    }

    override fun periodic() {
        LoopLogger.record("Drive periodic")

        LoopLogger.record("super Drive periodic")
        super.periodic()
        LoopLogger.record("super Drive periodic")

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

        // Update Vision
        cameras.forEach {
            it.updateInputs()
        }
        LoopLogger.record("Drive camera updateInputs")

        // Update poses with processed particle filter estimates.
        localizer.updateWithLatestPoseEstimate()
        LoopLogger.record("Drive updateWithLatestPose")
        // Create an odom measurement with a timestamp converted from phoenix time to fpga time.
        val poseMeasurement = PoseLocalizer.OdometryMeasurement(pose, stateTimestamp)
        // Publish the latest camera data to NT and also update pose from swerve odometry measurements.
        localizer.update(poseMeasurement, cameras.map { it.latestMeasurement }, chassisVelocities, wheelSlipFactor)
        LoopLogger.record("Drive localizer")

        headingHistory.put(Timer.getMonotonicTimestamp(), heading.degrees)
        LoopLogger.record("Recorded HeadingHistory")

        if (cameras.isNotEmpty()) {
            cameras.forEach {
                table.getEntry("Cameras/${it.cameraName} isConnected").setBoolean(it.isConnected)
                SimpleLogger.recordOutput("Drive/Cameras/${it.cameraName} isConnected", it.isConnected)
            }
        }
        LoopLogger.record("Cameras isConnected publish")

        // Log all the poses for debugging
        SimpleLogger.recordOutput("Swerve/Odometry", localizer.odometryPose)
        SimpleLogger.recordOutput("Swerve/Localizer Raw", localizer.rawPose)
        SimpleLogger.recordOutput("Swerve/Localizer", localizer.pose)
        SimpleLogger.recordOutput("Swerve/SingleTagPose", localizer.singleTagPose)

        LoopLogger.record("Drive periodic")
    }


    override fun defaultCommand(): Command = command(this) {
        await(joystickPercentageDrive())
    }

    /**
     * Sets all drive motor current limits to be the passed in [currentLimits].
     */
    @OptIn(DelicateCoroutinesApi::class)
    fun setDriveCurrentLimits(currentLimits: CurrentLimits) {
        GlobalScope.launch {
            modules.forEach {
                it.driveMotor.modifyConfiguration {
                    currentLimits(
                        currentLimits.continuousLimit,
                        currentLimits.peakLimit,
                        currentLimits.peakDuration
                    )
                }
            }
        }
    }

    /**
     * Returns [ChassisVelocities] with a percentage power from the driver controller.
     */
    override fun getJoystickPercentageSpeed(): ChassisVelocities {
        val rawJoystick = OI.driveTranslation
        // Square drive input and apply demoSpeed
        val power = rawJoystick.norm.square() * demoSpeed * if ((Shooter.isShooting || OI.driverController.rightStickButton) && FieldManager.inScoringZone) 0.3 else if (inSnakeMode) 0.8 else 1.0
        // Modify input to center in trench
        val joystickWithTrenchAlign = (rawJoystick.normalize() + FieldManager.trenchAlignTranslationModifier * rawJoystick.x.absoluteValue).normalize()
        // Apply modified power to joystick vector and flip depending on alliance
        val joystickTranslation = joystickWithTrenchAlign * power * if (isBlueAlliance) -1.0 else 1.0

        val rawJoystickRotation = OI.driveRotation
        // Cube rotation input and apply demoSpeed
        val omega = if (!(demoMode && driveLeftTriggerFullPress)) (rawJoystickRotation.cube() + FieldManager.trenchAlignRotationModifier * rawJoystick.x.absoluteValue) * demoSpeed else 0.0

        return ChassisVelocities(joystickTranslation.x, joystickTranslation.y, omega)
    }

    var inSnakeMode = false
    fun snakeMode(): Command = command(Drive) {
        periodic {
            println("snake mode")
            inSnakeMode = true
            val driveTranslation = OI.driveTranslation
            if (driveTranslation.norm > 0.1) {
                driveAtAngle(
                    atan2(
                        driveTranslation.x,
                        -driveTranslation.y
                    ).radians.asRotation2d - Rotation2d(90.0.degrees)
                )
            } else {
                driveVelocity(getChassisVelocitiesFromJoystick().apply { omega = 0.0 })
            }
        }
    }.onCancel {
        inSnakeMode = false
    }

    val wheelSlipMin = 1.2
    val wheelSlipMax = 4.0

    /**
     * A value representing wheel slippage from 0.0 (not slipping) to 1.0 (very slippy, swerve odometry not trustworthy).
     */
    @get:AutoLogOutput(key = "Swerve/WheelsSlipFactor")
    val wheelSlipFactor: Double get() {
        return ((wheelSlipRatio - wheelSlipMin) / (wheelSlipMax - wheelSlipMin)).coerceIn(0.0, 1.0)
    }

    /**
     * How much the wheels are slipping, determined by the ratio between the largest and smallest translation component of the wheels.
     */
    @get:AutoLogOutput(key = "Swerve/WheelsSlipRatio")
    val wheelSlipRatio: Double get() {
        val moduleRotationComponents = Array(moduleStates.size) {
            val state = SwerveModuleVelocity()
            state.velocity = gyroYawRate.asRadiansPerSecond * moduleLocations[it].norm
            state.angle = moduleLocations[it].angle.get() + 90.0.degrees.asRotation2d
            return@Array state
        }

        val moduleTranslationNorms = Array(moduleStates.size) { i ->
            (Translation2d(moduleStates[i].velocity, moduleStates[i].angle) - Translation2d(moduleRotationComponents[i].velocity, moduleRotationComponents[i].angle)).norm
        }.apply{ sort() }

        val mad = moduleTranslationNorms.map {(moduleTranslationNorms.average() - it).absoluteValue}.average()

        val accelerationDiff = (acceleration - Translation2d(pigeon2.accelerationX.value.asMetersPerSecondSquared, pigeon2.accelerationY.value.asMetersPerSecondSquared)).norm
        SimpleLogger.recordOutput("Swerve/DriveGyroAccelerationDifference", accelerationDiff)

        val minMaxRatio = moduleTranslationNorms.last() / (moduleTranslationNorms.first() + 0.001) // add fudge to prevent division by 0

//        Logger.recordOutput("Swerve/ModuleTranslationNorms/0", moduleTranslationNorms[0])
//        Logger.recordOutput("Swerve/ModuleTranslationNorms/1", moduleTranslationNorms[1])
//        Logger.recordOutput("Swerve/ModuleTranslationNorms/2", moduleTranslationNorms[2])
//        Logger.recordOutput("Swerve/ModuleTranslationNorms/3", moduleTranslationNorms[3])
//
//        Logger.recordOutput("Swerve/ModuleRotationComponents/0", moduleRotationComponents[0])
//        Logger.recordOutput("Swerve/ModuleRotationComponents/1", moduleRotationComponents[1])
//        Logger.recordOutput("Swerve/ModuleRotationComponents/2", moduleRotationComponents[2])
//        Logger.recordOutput("Swerve/ModuleRotationComponents/3", moduleRotationComponents[3])

        SimpleLogger.recordOutput("Swerve/ModuleTranslationsMinMaxRatio", minMaxRatio)
        SimpleLogger.recordOutput("Swerve/ModuleTranslationsMAD", mad)

//        val threshold = 10.0 // acc diff
//        return accelerationDiff.asMetersPerSecondPerSecond > threshold

//        val threshold = 0.09 // mad
//        return mad > threshold

        return minMaxRatio
    }

    @get:AutoLogOutput(key = "Swerve/WheelsSlipping")
    val wheelsSlipping: Boolean get() {
        return wheelSlipFactor > 0.0
    }


    /** Command to zero robot gyro */
    fun zeroGyroCommand() = command(Drive) {
        println("zero gyro command")
        zeroGyro()
    }

    /**
     * Resets swerve odometry ([Drive.pose]) to the vision [PoseLocalizer.pose].
     *
     * Useful if you want to use the swerve odometry for quick positioning or path following.
     *
     * Ex: Traveling somewhere in auto where you know the vision odometry will be unreliable. (human player station in 2025)
     */
    fun resetOdometryToAbsolute() {
        println("resetting odometry to localizer pose")
        val localizerPose = localizer.pose
        pose = Pose2d(localizerPose.translation, pose.rotation)
    }
}