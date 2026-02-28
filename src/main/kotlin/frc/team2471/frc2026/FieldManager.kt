package frc.team2471.frc2026

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import frc.team2471.frc2026.Robot.isAutonomous
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.meters
import org.team2471.frc.lib.units.UTranslation2d
import org.team2471.frc.lib.units.absoluteValue
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.wrap
import org.team2471.frc.lib.util.isRedAlliance
import kotlin.math.absoluteValue
import kotlin.math.floor

object FieldManager {
    val aprilTagFieldLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded) //AprilTagFieldLayout(Filesystem.getDeployDirectory().path + "/2026Field.json")
    val allAprilTags = aprilTagFieldLayout.tags

    val fieldWidth = aprilTagFieldLayout.fieldWidth.meters
    val fieldLength = aprilTagFieldLayout.fieldLength.meters

    val fieldDimensions = UTranslation2d(fieldLength, fieldWidth)

    val fieldHalfWidth = fieldWidth / 2.0
    val fieldHalfLength = fieldLength / 2.0

    val fieldCenter = fieldDimensions / 2.0

    val redHubTags = allAprilTags.filter { it.ID in 2..5 || it.ID in 8..11 }
    val blueHubTags = allAprilTags.filter { it.ID in 18..21 || it.ID in 24..27 }
    val hubTags = redHubTags + blueHubTags

    val overrideAutoWinner: LoggedDashboardChooser<String> =
        LoggedDashboardChooser<String>("Override Auto Winner").apply {
            addOption("None", null)
            addOption("Red", "R")
            addOption("Blue", "B")
        }

    val trenchAreaWidth = 75.0.inches
    val trenchAreaLength = 50.0.inches

    val lowerBlueTrenchPosition = ((allAprilTags[21].pose.toPose2d().translation + allAprilTags[22].pose.toPose2d().translation)/2.0)
    val upperBlueTrenchPosition = ((allAprilTags[16].pose.toPose2d().translation + allAprilTags[27].pose.toPose2d().translation)/2.0)
    val lowerRedTrenchPosition = ((allAprilTags[0].pose.toPose2d().translation + allAprilTags[11].pose.toPose2d().translation)/2.0)
    val upperRedTrenchPosition = ((allAprilTags[5].pose.toPose2d().translation + allAprilTags[6].pose.toPose2d().translation)/2.0)

    val trenchPositions: Array<Translation2d> = arrayOf(lowerBlueTrenchPosition, lowerRedTrenchPosition, upperRedTrenchPosition, upperBlueTrenchPosition)

    @get:AutoLogOutput(key = "FieldManager/In Trench Area")
    val inTrenchArea: Boolean
        get () {
            for (pose in trenchPositions) {
                val relativePose = pose - Drive.localizer.pose.translation
                if (relativePose.y.absoluteValue.meters < (trenchAreaLength/2.0) && relativePose.x.absoluteValue.meters < (trenchAreaWidth/2.0)) {
                    return true
                }
            }
            return false
        }

    val redGoalPose = (allAprilTags[3].pose.toPose2d().translation + allAprilTags[9].pose.toPose2d().translation)/2.0
    val blueGoalPose = (allAprilTags[19].pose.toPose2d().translation + allAprilTags[25].pose.toPose2d().translation)/2.0

    @get:AutoLogOutput(key = "FieldManager/Goal Pose")
    val goalPose: Translation2d
        get () = if (isRedAlliance) redGoalPose else blueGoalPose

    val passPose: Translation2d
        get() {
            var pose = Translation2d(3.0, 2.0)

            if (isRedAlliance) {
                pose = Translation2d(fieldLength.asMeters - pose.x, pose.y)
            }

            if (Drive.localizer.pose.y.meters > fieldHalfWidth) {
                pose = Translation2d(pose.x, fieldWidth.asMeters - pose.y)
            }

            return pose - AimUtils.calculateAimTargetOffset(AimUtils.PASS_AIRTIME)
//
//            return if (Drive.localizer.pose.y.meters > FieldManager.fieldHalfWidth) {
//                goalPose + Translation2d(0.0.inches, 70.0.inches)
//            } else {
//                goalPose + Translation2d(0.0.inches, -70.0.inches)
//            } - AimUtils.calculateAimTargetOffset(AimUtils.PASS_AIRTIME)
        }

    @get:AutoLogOutput(key = "FieldManager/Distance From Middle to Score")
    val distanceFromMiddleToScore = fieldCenter.x.asFeet.feet - lowerRedTrenchPosition.x.feet - 5.0.feet

    @get:AutoLogOutput(key = "FieldManager/Distance From Center")
    val xRelativeToCenter: Distance
        get () = (Drive.localizer.pose.x.meters - fieldCenter.x.asMeters.meters)

    @get:AutoLogOutput(key = "FieldManager/In Scoring Zone")
    val inScoringZone: Boolean
        get () = xRelativeToCenter.absoluteValue() > distanceFromMiddleToScore
                && if (isRedAlliance) xRelativeToCenter > 0.0.meters else xRelativeToCenter < 0.0.meters

    @get:AutoLogOutput(key = "FieldManager/gameData")
    val gameData: String
        get() = if (overrideAutoWinner.get() == null) DriverStation.getGameSpecificMessage() else overrideAutoWinner.get()

    @get:AutoLogOutput(key = "FieldManager/redWonAuto")
    val redWonAuto: Boolean
        get () = when (gameData) {
            "R" -> true
            "B" -> false
            else -> prevRedWonAuto
        }.also { prevRedWonAuto = it }

    private var prevRedWonAuto: Boolean = true

    val blueWonAuto: Boolean
        get () = !redWonAuto

    @get:AutoLogOutput(key = "FieldManager/weWonAuto")
    val weWonAuto: Boolean
        get () = redWonAuto == isRedAlliance

    @get:AutoLogOutput(key = "FieldManager/matchTime")
    val matchTime: Double
        get() = DriverStation.getMatchTime()

    @get:AutoLogOutput(key = "FieldManager/hubIsActive")
    val hubIsActive: Boolean
        get () {
            if (matchTime > 130.0 || matchTime < 30.0 || isAutonomous) {
                return true
            }
            if ((floor((matchTime - 30.0)/25.0)) % 2 == 0.0) {
                return weWonAuto
            } else {
                return !weWonAuto
            }
        }

    init {
        val apriltagPositions = allAprilTags.map { it.pose }
        Logger.recordOutput("All apriltags", *apriltagPositions.toTypedArray())
        println("FieldManager init. Field dimensions: $fieldDimensions. ${allAprilTags.size} tags.")
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

}
