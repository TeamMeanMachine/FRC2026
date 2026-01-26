package frc.team2471.frc2026

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.meters
import org.team2471.frc.lib.units.radians
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

object AimUtils {
    // seconds
    const val SHOT_AIRTIME = 0.85
    // m/s^2
    const val G = 9.80665
    // kg/m^3
    const val AIR_DENSITY = 1.2

    const val FUEL_DRAG_COEFFICIENT = 0.47
    // m
    const val FUEL_RADIUS = 0.075
    // m^2
    val FUEL_FRONTAL_AREA = FUEL_RADIUS.pow(2) * Math.PI
    // kg
    const val FUEL_MASS = 0.2172

    val aimTarget: Translation2d
        get() {
            if (FieldManager.inScoringZone) {
                return FieldManager.goalPose
            } else {
                // TODO: Use actual values for dumping positions
                if (Drive.pose.y.meters > FieldManager.fieldHalfWidth) {
                    return FieldManager.goalPose + Translation2d(0.0.inches, 70.0.inches)
                } else {
                    return FieldManager.goalPose + Translation2d(-0.0.inches, -70.0.inches)
                }
            }
        }


    val turretLookAheadPoint: Translation2d
        get() = Turret.turretPose + Drive.velocity * SHOT_AIRTIME


    /**
     * @return A pair of doubles, the first is the horizontal error from the target when the ball reaches a specific height, the second is the time error it took the ball to get there.
     */
    fun calcFuelError(vx: Double, vy: Double, goalPos: Translation2d, airTimeTarget: Double): Pair<Double, Double> {
        val fuel = FuelSim(Translation3d(), Translation3d(vx, 0.0, vy))
        var time = 0.0

        while (fuel.pos.z > goalPos.y || fuel.velocity.z >= 0.0) {
            fuel.update(0.02, 1)
            time += 0.02
        }

        return Pair(fuel.pos.x - goalPos.x, time - airTimeTarget)
    }


    //                              angle,                                    speed
    fun generateShooterCurve(): Pair<InterpolatingTreeMap<Double, Double>, InterpolatingTreeMap<Double, Double>> {
//                         meters,       m/s,  degrees
        val speedCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble())
        val angleCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble())
        for (i in 0..20) {
            val dist = i.toDouble().feet.asMeters
            val angleAndSpeed = getAngleAndSpeed(dist)
            angleCurve.put(dist, angleAndSpeed.first)
            speedCurve.put(dist, angleAndSpeed.second)
        }

        return Pair(angleCurve, speedCurve)
    }

    // Performs newtons method in 2 dimensions to estimate
    fun getAngleAndSpeed(distFromGoalM: Double): Pair<Double, Double> {
        val maxTError = 0.1
        val maxDError = 0.1

        val toTarget: Translation2d = Translation2d(distFromGoalM, 67.0.inches.asMeters - 0.4)

        // in vx and vy, will convert to angle and velocity at the end
        // this is derived from kinematic equations and assumes no drag
        var guess = Pair(toTarget.x / SHOT_AIRTIME, (toTarget.y + 0.5 * G * SHOT_AIRTIME.pow(2)) / SHOT_AIRTIME)
        var guessIncremented = Pair(guess.first + 0.1, guess.second + 0.1)


        var errors = calcFuelError(guess.first, guess.second, toTarget, SHOT_AIRTIME)
        var errorSum = errors.first.absoluteValue + errors.second.absoluteValue


        var errorsIncremented = calcFuelError(guessIncremented.first, guessIncremented.second, toTarget, SHOT_AIRTIME)
        var errorSumIncremented = errorsIncremented.first.absoluteValue + errorsIncremented.second.absoluteValue


        var num = 0
        while ((errors.first.absoluteValue > maxDError || errors.second.absoluteValue > maxTError) && num <= 5) {
            //guess is x1 & y1, errorsum is z1
            //guessincremented is x1 & y1, errorsumincremented is z1
            /*First, calculate a line between the two points in parametric form.
                x=x1+(x2-x1)t
                y=y1+(y2-y1)t
                z=z1+(z2-z1)t

                where 0<=t<=1

                I just need to store x and y
             */
            val x1 = guess.first
            val y1 = guess.second
            val x2 = guessIncremented.first
            val y2 = guessIncremented.second

            println("($x1,$y1,$errorSum)")
            println("($x2,$y2,$errorSumIncremented)")

            // Then find the t value for which z is 0
            //  t = -z1       / (           z2       -  z1      )
            val t = -errorSum / (errorSumIncremented - errorSum)

            guess = Pair(x1+(x2-x1) * t, y1+(y2-y1) * t)

            guessIncremented = Pair(guess.first + 0.1, guess.second + 0.1)

            errors = calcFuelError(guess.first, guess.second, toTarget, SHOT_AIRTIME)
            errorSum = errors.first.absoluteValue + errors.second.absoluteValue

            errorsIncremented = calcFuelError(guessIncremented.first, guessIncremented.second, toTarget, SHOT_AIRTIME)
            errorSumIncremented = errorsIncremented.first.absoluteValue + errorsIncremented.second.absoluteValue

            num++
        }
        println("Dist: ${distFromGoalM}, ErrorSum: ${errorSum}, iterations: ${num}")

        return Pair(atan2(guess.second, guess.first).radians.asDegrees, sqrt(guess.first.pow(2) + guess.second.pow(2)))
    }
}