package frc.team2471.frc2026

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.ctre.a
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.d
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.ctre.v
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asMetersPerSecond
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.asVolts
import org.team2471.frc.lib.units.cos
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.meters
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.units.rotationsPerSecond
import org.team2471.frc.lib.units.seconds
import org.team2471.frc.lib.units.sin
import org.team2471.frc.lib.units.volts
import org.team2471.frc.lib.units.voltsPerSecond
import org.team2471.frc.lib.util.angleTo
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

// Unless otherwise specified every double here is in meters
object Shooter: SubsystemBase("Shooter") {
    var fuel: MutableList<FlyingFuel> = mutableListOf()

    val shooterCurves = generateShooterCurve()
    val speedCurve = shooterCurves.second
    val angleCurve = shooterCurves.first

    var isShooting = false
    var i = 0

    override fun periodic() {
        if (isShooting) {
            if (i > 1) {
                i = 0
                shoot()
            } else {
                i++
            }
        }
        fuel.forEach { it.update() }
        logFuel("fuel", *fuel.toTypedArray())
        fuel.removeFuel()
    }

    fun shoot() {
        val robotVelocity = Drive.velocity
        val newPos = Drive.pose.translation + Drive.velocity
        val dist = newPos.getDistance(FieldManager.redGoalPose).absoluteValue
        val exitVelocity = speedCurve.get(dist)
        val exitAngle = angleCurve.get(dist).degrees
        println("Hi ${exitVelocity}, ${exitAngle}")
        val angleToTarget = FieldManager.redGoalPose.angleTo(newPos)
        val velocity2d = Translation2d(-exitVelocity * exitAngle.cos(), 0.0).rotateBy(angleToTarget.asRotation2d)
        fuel.add(FlyingFuel(
            Translation3d(Drive.pose.translation.x, Drive.pose.translation.y, 0.4),
            Translation3d(velocity2d.x + Drive.velocity.x.asMetersPerSecond, velocity2d.y + Drive.velocity.y.asMetersPerSecond, exitVelocity * exitAngle.sin())
        ))
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

        val goalTime = 1.0

        val maxTError = 0.1
        val maxDError = 0.1

        // todo account for turret offset
        val toTarget: Translation2d = Translation2d(distFromGoalM, 67.0.inches.asMeters - 0.4)
//        println("ToTarger: ${toTarget}")

        // in vx and vy, will convert to angle and velocity at the end
        // this is derived from kinematic equations and assumes no drag
        var guess = Pair(toTarget.x / goalTime, (toTarget.y + 0.5 * g * goalTime.pow(2)) / goalTime)
        var guessIncremented = Pair(guess.first + 0.1, guess.second + 0.1)

//        println("Guesses")
//        println("Initial guess: ${guess.first.round(4)}, ${guess.second.round(4)}")

        var errors = calcFuelError(guess.first, guess.second, toTarget, goalTime)
        var errorSum = errors.first.absoluteValue + errors.second.absoluteValue

//        println("error 1")

        var errorsIncremented = calcFuelError(guessIncremented.first, guessIncremented.second, toTarget, goalTime)
        var errorSumIncremented = errorsIncremented.first.absoluteValue + errorsIncremented.second.absoluteValue


//        println("Calculating velocities")
//        println("Initial guess: ${guess.first.round(4)}, ${guess.second.round(4)}")
//        println("Initial error: ${errors.first.round(4)}, ${errors.second.round(4)}")
//        println("Errorsum: ${errorSum}\n")

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

            errors = calcFuelError(guess.first, guess.second, toTarget, goalTime)
            errorSum = errors.first.absoluteValue + errors.second.absoluteValue

            errorsIncremented = calcFuelError(guessIncremented.first, guessIncremented.second, toTarget, goalTime)
            errorSumIncremented = errorsIncremented.first.absoluteValue + errorsIncremented.second.absoluteValue

//            println("guess: ${guess.first.round(4)}, ${guess.second.round(4)}")
//            println("error: ${errors.first.round(4)}, ${errors.second.round(4)}")
//            println("Errorsum: ${errorSum}\n")

            num++
        }
        println("Dist: ${distFromGoalM}, ErrorSum: ${errorSum}, iterations: ${num}")

        return Pair(atan2(guess.second, guess.first).radians.asDegrees, sqrt(guess.first.pow(2) + guess.second.pow(2)))
    }
}