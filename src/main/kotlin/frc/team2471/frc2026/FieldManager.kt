package frc.team2471.frc2026

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
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
    val aprilTagFieldLayout: AprilTagFieldLayout = AprilTagFieldLayout(Filesystem.getDeployDirectory().path + "/2026Field.json")
    val allAprilTags = aprilTagFieldLayout.tags

    val fieldWidth = aprilTagFieldLayout.fieldWidth.meters
    val fieldLength = aprilTagFieldLayout.fieldLength.meters

    val fieldDimensions = UTranslation2d(fieldLength, fieldWidth)

    val fieldHalfWidth = fieldWidth / 2.0
    val fieldHalfLength = fieldLength / 2.0

    val fieldCenter = fieldDimensions / 2.0

    val trenchAreaWidth = 75.0.inches
    val trenchAreaLength = 50.0.inches

    val lowerBlueTrenchPosition = ((allAprilTags[21].pose.toPose2d().translation + allAprilTags[22].pose.toPose2d().translation)/2.0)
    val upperBlueTrenchPosition = ((allAprilTags[16].pose.toPose2d().translation + allAprilTags[27].pose.toPose2d().translation)/2.0)
    val lowerRedTrenchPosition = ((allAprilTags[0].pose.toPose2d().translation + allAprilTags[11].pose.toPose2d().translation)/2.0)
    val upperRedTrenchPosition = ((allAprilTags[5].pose.toPose2d().translation + allAprilTags[6].pose.toPose2d().translation)/2.0)

    val trenchPositions: Array<Translation2d> = arrayOf(lowerBlueTrenchPosition, lowerRedTrenchPosition, upperRedTrenchPosition, upperBlueTrenchPosition)

    val inTrenchArea: Boolean
        get () {
            for (pose in trenchPositions) {
                val relativePose = pose - Drive.pose.translation
                if (relativePose.x.absoluteValue.meters < (trenchAreaWidth/2.0)) {
                    return true
                }
                if (relativePose.y.absoluteValue.meters < (trenchAreaLength/2.0)) {
                    return true
                }
            }
            return false
        }

    val redGoalPose = (allAprilTags[3].pose.toPose2d().translation + allAprilTags[9].pose.toPose2d().translation)/2.0
    val blueGoalPose = (allAprilTags[19].pose.toPose2d().translation + allAprilTags[25].pose.toPose2d().translation)/2.0
    val goalPose: Translation2d
        get () = if (isRedAlliance) redGoalPose else blueGoalPose

    val distanceFromMiddleToScore = fieldCenter.x - lowerBlueTrenchPosition.x.feet

    val inScoringZone: Boolean
        get () = (fieldCenter.x - Drive.pose.translation.x.feet).absoluteValue() > distanceFromMiddleToScore

    @get:AutoLogOutput(key = "FieldManager/redWonAuto")
    val redWonAuto: Boolean?
        get () = when (DriverStation.getGameSpecificMessage()) {
            "R" -> true
            "B" -> false
            else -> null
        }

    val blueWonAuto: Boolean?
        get () = redWonAuto?.not()

    val weWonAuto: Boolean?
        get () = redWonAuto?.equals(isRedAlliance)

    val hubIsActive: Boolean?
        get () {
            if (weWonAuto == null) {
                return null
            }

            if (DriverStation.getMatchTime() > 130.0) {
                return true
            }
            if (DriverStation.getMatchTime() < 30.0) {
                return true
            }
            if ((floor((DriverStation.getMatchTime() - 30.0)/25.0)) % 2 == 0.0) {
                return weWonAuto!!
            } else {
                return !weWonAuto!!
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
