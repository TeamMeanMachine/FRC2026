package frc.team2471.frc2026

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.units.measure.Distance
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asRadiansPerSecond
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
    const val PASS_AIRTIME = 1.0
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

    val HUB_HEIGHT = 65.0.inches

    @get:AutoLogOutput(key = "aim target")
    val aimTarget: Translation2d
        get() {
            val turretVelocity = Translation2d(Turret.turretOffsetFromCenter.x * Drive.gyroYawRate.asRadiansPerSecond, Turret.turretOffsetFromCenter.y * Drive.gyroYawRate.asRadiansPerSecond).rotateBy(Drive.heading) + Drive.velocity

            return if (FieldManager.inScoringZone) {
                FieldManager.goalPose - turretVelocity * SHOT_AIRTIME
            } else {
                // TODO: Use actual values for dumping positions
                if (Drive.pose.y.meters > FieldManager.fieldHalfWidth) {
                    // This is the stuff making the robot aim in the middle of the hump. Keeping it until we are sure it doesn't work.
                    FieldManager.goalPose + Translation2d(0.0.inches, 70.0.inches)
                } else {
                    FieldManager.goalPose + Translation2d(-0.0.inches, -70.0.inches)
                } - turretVelocity * PASS_AIRTIME
            }
        }


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


    //                                                                               angle,                                    speed
    fun generateShooterCurve(goalHeight: Distance, airTime: Double): Pair<InterpolatingTreeMap<Double, Double>, InterpolatingTreeMap<Double, Double>> {
//                         meters,       m/s,  degrees
        val speedCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble())
        val angleCurve = InterpolatingTreeMap(InverseInterpolator.forDouble(), Interpolator.forDouble())
        for (i in 0..40) {
            val dist = i.toDouble().feet
            val angleAndSpeed = getAngleAndSpeed(dist, goalHeight, airTime)
            angleCurve.put(dist.asMeters, angleAndSpeed.first)
            speedCurve.put(dist.asMeters, angleAndSpeed.second)
            println("Dist: ${dist.asFeet.round(3)} angle: ${angleAndSpeed.first.round(3)} Speed: ${angleAndSpeed.second.round(3)}")
        }

        return Pair(angleCurve, speedCurve)
    }

    // Performs newtons method in 2 dimensions to estimate
    fun getAngleAndSpeed(distFromGoal: Distance, goalHeight: Distance, airTime: Double): Pair<Double, Double> {
        val maxTError = 0.1
        val maxDError = 0.1

        val toTarget: Translation2d = Translation2d(distFromGoal, goalHeight)

        // in vx and vy, will convert to angle and velocity at the end
        // this is derived from kinematic equations and assumes no drag
        var guess = Pair(toTarget.x / airTime, (toTarget.y + 0.5 * G * airTime.pow(2)) / airTime)
        var guessIncremented = Pair(guess.first + 0.1, guess.second + 0.1)


        var errors = calcFuelError(guess.first, guess.second, toTarget, airTime)
        var errorSum = errors.first.absoluteValue + errors.second.absoluteValue


        var errorsIncremented = calcFuelError(guessIncremented.first, guessIncremented.second, toTarget, airTime)
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

//            println("($x1,$y1,$errorSum)")
//            println("($x2,$y2,$errorSumIncremented)")

            // Then find the t value for which z is 0
            //  t = -z1       / (           z2       -  z1      )
            val t = -errorSum / (errorSumIncremented - errorSum)

            guess = Pair(x1+(x2-x1) * t, y1+(y2-y1) * t)

            guessIncremented = Pair(guess.first + 0.1, guess.second + 0.1)

            errors = calcFuelError(guess.first, guess.second, toTarget, airTime)
            errorSum = errors.first.absoluteValue + errors.second.absoluteValue

            errorsIncremented = calcFuelError(guessIncremented.first, guessIncremented.second, toTarget, airTime)
            errorSumIncremented = errorsIncremented.first.absoluteValue + errorsIncremented.second.absoluteValue

            num++
        }
//        println("Dist: ${distFromGoal.asMeters}, ErrorSum: ${errorSum}, iterations: ${num}")

        return Pair(atan2(guess.second, guess.first).radians.asDegrees, sqrt(guess.first.pow(2) + guess.second.pow(2)))
    }
}