package frc.team2471.frc2026

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.frc2026.FieldManager.reflectAcrossField
import frc.team2471.frc2026.FieldManager.rotateAroundField
import frc.team2471.frc2026.Robot.isAutonomous
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.math.toPose2d
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.angleTo
import org.team2471.frc.lib.util.demoMode
import org.team2471.frc.lib.util.isBlueAlliance
import org.team2471.frc.lib.util.isRedAlliance
import kotlin.math.absoluteValue
import kotlin.math.floor
import kotlin.math.sign

object FieldManager {
    private val table = NetworkTableInstance.getDefault().getTable("FieldManager")

    val aprilTagFieldLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded) //AprilTagFieldLayout(Filesystem.getDeployDirectory().path + "/2026Field.json")
    val allAprilTags = aprilTagFieldLayout.tags

    // x
    val fieldWidth = aprilTagFieldLayout.fieldWidth.meters
    // y
    val fieldLength = aprilTagFieldLayout.fieldLength.meters

    val fieldDimensions = UTranslation2d(fieldLength, fieldWidth)

    val fieldHalfWidth = fieldWidth / 2.0
    val fieldHalfLength = fieldLength / 2.0

    val fieldCenter = fieldDimensions / 2.0

    val redHubTags = allAprilTags.filter { it.ID in 2..5 || it.ID in 8..11 }
    val blueHubTags = allAprilTags.filter { it.ID in 18..21 || it.ID in 24..27 }
    val hubTags = redHubTags + blueHubTags

    val overrideAutoWinnerChooser: LoggedDashboardChooser<String?> =
        LoggedDashboardChooser<String?>("Override Auto Winner").apply {
            addDefaultOption("No Override", null)
            addOption("Red", "R")
            addOption("Blue", "B")
        }

    val preferredPassingSideChooser: LoggedDashboardChooser<PassingSide> =
        LoggedDashboardChooser<PassingSide>("Preferred Passing Side").apply {
            addDefaultOption("Both", PassingSide.BOTH)
            addOption("Outpost", PassingSide.OUTPOST)
            addOption("Depot", PassingSide.DEPOT)
        }

    val trenchAreaWidth = 50.0.inches
    val trenchAreaLength = 27.0.inches

    val towerAreaWidth = 62.0.inches
    val towerAreaLength = 47.0.inches

    val xRelativeToCenter: Distance
        get () = (Drive.localizer.pose.x.meters - fieldCenter.x)
    val yRelativeToCenter: Distance
        get() = Drive.localizer.pose.y.meters - fieldCenter.y



    val lowerBlueTrenchPosition = ((allAprilTags[21].pose.toPose2d().translation + allAprilTags[22].pose.toPose2d().translation)/2.0)
    val upperBlueTrenchPosition = ((allAprilTags[16].pose.toPose2d().translation + allAprilTags[27].pose.toPose2d().translation)/2.0)
    val lowerRedTrenchPosition = ((allAprilTags[0].pose.toPose2d().translation + allAprilTags[11].pose.toPose2d().translation)/2.0)
    val upperRedTrenchPosition = ((allAprilTags[5].pose.toPose2d().translation + allAprilTags[6].pose.toPose2d().translation)/2.0)

    val lowerBlueStaticShotPosition = lowerBlueTrenchPosition + Translation2d((-24.0).inches.asMeters, (-15.0).inches.asMeters)
    val upperBlueStaticShotPosition = upperBlueTrenchPosition + Translation2d((-24.0).inches.asMeters, (15.0).inches.asMeters)
    val lowerRedStaticShotPosition = lowerRedTrenchPosition + Translation2d((24.0).inches.asMeters, (-15.0).inches.asMeters)
    val upperRedStaticShotPosition = upperRedTrenchPosition + Translation2d((24.0).inches.asMeters, (15.0).inches.asMeters)

    val trenchPositions: Array<Translation2d> = arrayOf(lowerBlueTrenchPosition, lowerRedTrenchPosition, upperRedTrenchPosition, upperBlueTrenchPosition)

    @get:AutoLogOutput(key = "FieldManager/In Trench Area")
    val inTrenchArea: Boolean
        get () {
            for (pose in trenchPositions) {
                val relativePose = pose - Drive.localizer.pose.translation
                if (relativePose.y.absoluteValue.meters < (trenchAreaWidth/2.0)) {
                    val predictedPose = Drive.localizer.pose.translation + Drive.velocity * Shooter.HOOD_DOWN_TIME
                    Logger.recordOutput("Drive/predictedPose", predictedPose)
                    val predictedRelativePose = pose - predictedPose
                    if ((predictedRelativePose.x.sign != relativePose.x.sign) || (relativePose.x.absoluteValue.meters < (trenchAreaLength/2.0))) {
                        return true
                    }
                }
            }
            return false
        }

    @get:AutoLogOutput(key = "FieldManager/trenchAlignVector")
    val trenchAlignJoystickModifier: Translation2d
        get () {
            val areaWidth = 80.0.inches
            val areaLength = 300.0.inches
            for (pose in trenchPositions) {
                val relativePose = pose - Drive.localizer.pose.translation
                if (relativePose.y.absoluteValue.meters < (areaWidth * 0.5)) {
                    if (relativePose.x.absoluteValue.meters < (areaLength * 0.5)) {
                        if (relativePose.x.sign == Drive.velocity.x.asMetersPerSecond.sign) {
                            if (relativePose.y.sign == yRelativeToCenter.asMeters.sign) {
                                return Translation2d(0.0.inches, relativePose.y.meters) * if (isBlueAlliance) -trenchAssistStrength else trenchAssistStrength
                            }
                        }
                    }
                }
            }
            return Translation2d()
        }

    @get:AutoLogOutput(key = "FieldManager/In Tower Area")
    val inTowerArea: Boolean
        get() {
            val yRelativeToTower: Distance = (Drive.localizer.pose.y.meters - (fieldCenter.y.asMeters.meters + 11.46.inches * xRelativeToCenter.asInches.sign))

            if (xRelativeToCenter.absoluteValue() > (fieldHalfLength - towerAreaLength) && yRelativeToTower.absoluteValue() < towerAreaWidth / 2.0) {
                return true
            }

            return false
        }

    @get:AutoLogOutput(key = "FieldManager/In Opposing Alliance Zone")
    val inOpposingAllianceZone: Boolean
        get () = xRelativeToCenter.absoluteValue() > distanceFromMiddleToScore
                && if (isRedAlliance) xRelativeToCenter < 0.0.meters else xRelativeToCenter > 0.0.meters

    @get:AutoLogOutput(key = "FieldManager/In No Pass Area")
    val inOpposingNoPassArea: Boolean
        get() = if (!Drive.useAprilTags) false else inOpposingAllianceZone && yRelativeToCenter.absoluteValue() < 3.0.feet

    @get:AutoLogOutput(key = "FieldManager/In No Shoot Area")
    val inNoShootArea: Boolean
        get() = ((autoHoodRetraction && Drive.useAprilTags) && (inTowerArea || inTrenchArea)) || inOpposingNoPassArea


    val redTowerPose = (allAprilTags[14].pose.toPose2d().translation + Translation2d(-1.75, 0.0))
    val blueTowerPose = (allAprilTags[30].pose.toPose2d().translation + Translation2d(1.75, 0.0))

    val towerPose = Translation2d(11.0.feet.asMeters, 14.0.feet.asMeters)

    //1 3/4 3ft wide


    val redGoalPose = (allAprilTags[3].pose.toPose2d().translation + allAprilTags[9].pose.toPose2d().translation)/2.0
    val blueGoalPose = (allAprilTags[19].pose.toPose2d().translation + allAprilTags[25].pose.toPose2d().translation)/2.0

    @get:AutoLogOutput(key = "FieldManager/Goal Pose")
    val goalPose: Translation2d
        get () = if (isRedAlliance) redGoalPose else blueGoalPose

    @get:AutoLogOutput(key = "FieldManager/Pass Pose")
    val loggedPassPose get() = passPose.toPose2d()

    val passPose: Translation2d
        get() {
            var pose = if (Robot.isTeleop) Translation2d(4.0, 2.0) else Translation2d(2.0, 1.25)

            if (isRedAlliance) {
                pose = Translation2d(fieldLength.asMeters - pose.x, fieldWidth.asMeters - pose.y)
            } else {
                pose = Translation2d(pose.x, pose.y)
            }

            // meters
            if (
                if (preferredPassingSide == PassingSide.BOTH)
                    (Drive.localizer.pose.y.meters > fieldHalfWidth)
                else
                    (Drive.localizer.pose.translation.getDistance(pose) > 7.0 && (yRelativeToCenter.asMeters.sign == pose.y.sign * if (isRedAlliance xor (preferredPassingSide == PassingSide.DEPOT)) -1.0 else 1.0))
                ) {
                pose = Translation2d(pose.x, fieldWidth.asMeters - pose.y)
            }

            return pose
//
//            return if (Drive.localizer.pose.y.meters > FieldManager.fieldHalfWidth) {
//                goalPose + Translation2d(0.0.inches, 70.0.inches)
//            } else {
//                goalPose + Translation2d(0.0.inches, -70.0.inches)
//            } - AimUtils.calculateAimTargetOffset(AimUtils.PASS_AIRTIME)
        }

    val distanceFromMiddleToScore = fieldCenter.x.asFeet.feet - lowerRedTrenchPosition.x.feet - 5.0.feet

    @get:AutoLogOutput(key = "FieldManager/Pass Over Net")
    val passOverNet: Boolean get() {
        val angleToPassPoint = passPose.angleTo(Drive.localizer.pose.translation)
        val xOffset = if (isRedAlliance) -0.5.meters else 0.5.meters
        val angleToNet1 = passPose.angleTo(goalPose + Translation2d(xOffset, 1.0.meters))
        val angleToNet2 = passPose.angleTo(goalPose + Translation2d(xOffset, -1.0.meters))

        return angleToPassPoint in angleToNet1..angleToNet2 || angleToPassPoint in angleToNet2..angleToNet1
    }

    @get:AutoLogOutput(key = "FieldManager/In Scoring Zone")
    val inScoringZone: Boolean
        get () = xRelativeToCenter.absoluteValue() > distanceFromMiddleToScore
                && if (isRedAlliance) xRelativeToCenter > 0.0.meters else xRelativeToCenter < 0.0.meters

    const val HUB_PROCESSING_TIME = 1.0
    const val RAMP_TIME = 3.0

    @get:AutoLogOutput(key = "FieldManager/rawGameData")
    val rawGameData: String
        get() = DriverStation.getGameSpecificMessage()

    @get:AutoLogOutput(key = "FieldManager/gameData")
    val gameData: String
        get() = overrideAutoWinnerChooser.get() ?: rawGameData

    @get:AutoLogOutput(key = "FieldManager/gameData")
    val preferredPassingSide: PassingSide
        get() = preferredPassingSideChooser.get()

    @get:AutoLogOutput(key = "FieldManager/redWonAuto")
    val redWonAuto: Boolean
        get() = when (gameData) {
            "R" -> true
            "B" -> false
            else -> prevRedWonAuto
        }.also { prevRedWonAuto = it }

    private var prevRedWonAuto: Boolean = true //By default, we assume red won

    val blueWonAuto: Boolean
        get () = !redWonAuto

    @get:AutoLogOutput(key = "FieldManager/weWonAuto")
    val weWonAuto: Boolean
        get () = redWonAuto == isRedAlliance

    val weWonAutoEntry = table.getEntry("We Won Auto")

    @get:AutoLogOutput(key = "FieldManager/matchTime")
    val matchTime: Double
        get() = DriverStation.getMatchTime()

    val doShiftTimingEntry = table.getEntry("DoShiftTiming")
    val autoHoodRetractionEntry = table.getEntry("AutoHoodRetraction")

    val trenchAssistStrengthEntry = table.getEntry("TrenchAssistStrength")
    val trenchAssistStrength get() = trenchAssistStrengthEntry.getDouble(2.0)

    val hubCountdownEntry = table.getEntry("HubCountdown")
    val activeHubEntry = table.getEntry("ActiveHub")

    val doShiftTiming get() = doShiftTimingEntry.getBoolean(true) && !demoMode
    var autoHoodRetraction
        get() = autoHoodRetractionEntry.getBoolean(true)
        set(value) {
            autoHoodRetractionEntry.setBoolean(value)
        }

    val shouldShootStartTimes = arrayOf(130.0, 105.0, 80.0, 55.0).map { it + AimUtils.MEASURED_SHOT_AIRTIME + HUB_PROCESSING_TIME }
    val shouldRampStartTimes = shouldShootStartTimes.map { it + RAMP_TIME}
    val shouldShootEndTimes = arrayOf(105.0, 80.0, 55.0, 30.0)

    // this is offset by shoot time. for shooting /O\
    @get:AutoLogOutput(key = "FieldManager/shouldShoot")
    val shouldShoot: Boolean
        get () {
            if (!doShiftTiming) {
                return true
            }
            if (matchTime > 130.0 || matchTime < 30.0 + AimUtils.MEASURED_SHOT_AIRTIME + HUB_PROCESSING_TIME || isAutonomous) {
                return true
            }
            return if (matchTime in shouldShootEndTimes[1]..shouldShootStartTimes[1] || matchTime in shouldShootEndTimes[3]..shouldShootStartTimes[3])   {
                weWonAuto
            } else {
                !weWonAuto
            }
        }

    @get:AutoLogOutput(key = "FieldManager/shouldRamp")
    val shouldRamp: Boolean
        get () {
            if (!doShiftTiming || matchTime < 0.0) {
                return if (Drive.useAprilTags) AimUtils.isAimingAtGoal else false
            }
            if (matchTime > 130.0 || matchTime < 30.0 + AimUtils.MEASURED_SHOT_AIRTIME + HUB_PROCESSING_TIME + RAMP_TIME || isAutonomous) {
                return true
            }
            return if (matchTime in shouldShootEndTimes[1]..shouldRampStartTimes[1] || matchTime in shouldShootEndTimes[3]..shouldRampStartTimes[3])   {
                weWonAuto
            } else {
                !weWonAuto
            }
        }

    @get:AutoLogOutput(key = "FieldManager/hubIsActive")
    val hubIsActive: Boolean
        get () {
            if (matchTime !in 30.0..130.0 || isAutonomous) {
                return true
            }
            return if ((floor((matchTime - 30.0)/25.0)) % 2 == 0.0) {
                weWonAuto
            } else {
                !weWonAuto
            }
        }

    init {
        doShiftTimingEntry.setBoolean(true)
        autoHoodRetractionEntry.setBoolean(true)

        trenchAssistStrengthEntry.setDouble(trenchAssistStrength)
        trenchAssistStrengthEntry.setPersistent()

        val apriltagPositions = allAprilTags.map { it.pose }
        Logger.recordOutput("FieldManager/All apriltags", *apriltagPositions.toTypedArray())
        println("FieldManager init. Field dimensions: $fieldDimensions. ${allAprilTags.size} tags.")

        Logger.recordOutput("FieldManager/Trench Poses", *trenchPositions)

        Logger.recordOutput("FieldManager/TowerPoseRed", redTowerPose)
        Logger.recordOutput("FieldManager/TowerPoseBlue", blueTowerPose)

        GlobalScope.launch {
            periodic {
                weWonAutoEntry.setBoolean(weWonAuto)
                hubCountdownEntry.setDouble(if (matchTime > 130.0) matchTime - 130.0 else if (matchTime < 30.0 || (matchTime < 55.0 && weWonAuto)) matchTime else (matchTime - 5) % 25.0)
                activeHubEntry.setString(
                    if (isAutonomous || matchTime > 130.0 || matchTime < 30.0) {
                        "Both"
                    } else if (isRedAlliance == hubIsActive) {
                        "Red"
                    } else {
                        "Blue"
                    }
                )
            }
        }
    }

    fun disableAutoHoodRetractionCommand(): Command {
        var oldValue = autoHoodRetraction
        return sequenceCommand(
            runOnceCommand {
                oldValue = autoHoodRetraction
                autoHoodRetraction = false
                println("Disabling autoHoodRetraction. oldValue: $oldValue")
            },
            runCommand {
                autoHoodRetraction = false
            },
            runOnceCommand {
                autoHoodRetraction = oldValue
                println("Finished disabling autoHoodRetraction. autoHoodRetraction: $autoHoodRetraction")
            }
        ).withName("AutoHoodRetractionDisableCommand")
    }

    /**
     * Reflects [Translation2d] across the midline of the field. Useful for mirrored field layouts (2023, 2024).
     * Units must be meters
     * @param doReflect Supplier to perform reflection. Default: true
     * @see Translation2d.rotateAroundField
     */
    fun Translation2d.reflectAcrossField(doReflect: () -> Boolean = { true }): Translation2d {
        return if (doReflect()) Translation2d(fieldLength.asMeters - x, y) else this
    }

    /**
     * Reflects [Pose2d] across the midline of the field. Useful for mirrored field layouts (2023, 2024).
     * Units must be meters
     * @param doReflect Supplier to perform reflection. Default: true
     * @see Pose2d.rotateAroundField
     */
    fun Pose2d.reflectAcrossField(doReflect: () -> Boolean = { true }): Pose2d {
        return if (doReflect()) Pose2d(fieldLength.asMeters - x, y, (rotation - 180.0.degrees.asRotation2d).wrap()) else this
    }

    /**
     * Rotates the [Translation2d] 180 degrees around the center of the field. Useful for reflected field layouts (2022, 2025).
     * Units must be meters
     * @param doRotate Supplier to perform rotation. Default: true
     * @see Translation2d.reflectAcrossField
     */
    fun Translation2d.rotateAroundField(doRotate: () -> Boolean = { true }): Translation2d {
        return if (doRotate()) this.rotateAround(fieldCenter, 180.0.degrees.asRotation2d) else this
    }

    /**
     * Rotates the [Pose2d] 180 degrees around the center of the field. Useful for reflected field layouts (2022, 2025).
     * Units must be meters
     * @param doRotate Supplier to perform rotation. Default: true
     * @see Pose2d.reflectAcrossField
     */
    fun Pose2d.rotateAroundField(doRotate: () -> Boolean = { true }): Pose2d {
        return if (doRotate()) this.rotateAround(fieldCenter, 180.0.degrees.asRotation2d) else this
    }

    /**
     * Returns if the [Translation2d] is on the red alliance side of the field.
     */
    fun Translation2d.onRedSide(): Boolean = this.x > fieldCenter.x.asMeters
    /**
     * Returns if the [Translation2d] is on the blue alliance side of the field.
     */
    fun Translation2d.onBlueSide(): Boolean = !this.onRedSide()
    /**
     * Returns if the [Translation2d] is closer to your current alliance's side of the field.
     */
    fun Translation2d.onFriendlyAllianceSide() = this.onRedSide() == isRedAlliance
    /**
     * Returns if the [Translation2d] is closer to your opponent alliance's side of the field.
     */
    fun Translation2d.onOpposingAllianceSide() = !this.onFriendlyAllianceSide()

    /**
     * Returns if the [Pose2d] is on the red alliance side of the field.
     */
    fun Pose2d.onRedSide(): Boolean = this.translation.onRedSide()
    /**
     * Returns if the [Pose2d] is on the blue alliance side of the field.
     */
    fun Pose2d.onBlueSide(): Boolean = !this.onRedSide()
    /**
     * Returns if the [Pose2d] is closer to your current alliance's side of the field.
     */
    fun Pose2d.onFriendlyAllianceSide() = this.translation.onFriendlyAllianceSide()
    /**
     * Returns if the [Pose2d] is closer to your opponent alliance's side of the field.
     */
    fun Pose2d.onOpposingAllianceSide() = !this.onFriendlyAllianceSide()


    enum class PassingSide {
        BOTH,
        OUTPOST,
        DEPOT
    }
}
