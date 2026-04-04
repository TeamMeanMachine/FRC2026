package frc.team2471.frc2026

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Velocity
import frc.team2471.frc2026.Robot.isCompBot
import frc.team2471.frc2026.Shooter.SHOOTER_GEAR_RATIO
import frc.team2471.frc2026.Shooter.floorSpeedCurve
import frc.team2471.frc2026.Shooter.hubSpeedCurve
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.asFeetPerSecond
import org.team2471.frc.lib.units.asInches
import org.team2471.frc.lib.units.asInchesPerSecond
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asMetersPerSecond
import org.team2471.frc.lib.units.asRadiansPerSecond
import org.team2471.frc.lib.units.asRotationsPerSecond
import org.team2471.frc.lib.units.cos
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.inchesPerSecond
import org.team2471.frc.lib.units.kilograms
import org.team2471.frc.lib.units.meters
import org.team2471.frc.lib.units.metersPerSecond
import org.team2471.frc.lib.units.radians
import org.team2471.frc.lib.units.rotationsPerSecond
import org.team2471.frc.lib.units.sin
import org.team2471.frc.lib.util.isRedAlliance
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

object AimUtils {
    // seconds
    const val TARGET_SHOT_AIRTIME = 1.25
    const val MEASURED_SHOT_AIRTIME = 1.0
    const val PASS_AIRTIME = 1.0

    // m/s^2
    const val G = 9.80665
    // kg/m^3
    const val AIR_DENSITY = 1.2

    const val FUEL_DRAG_COEFFICIENT = 0.47

    // m
    val FUEL_RADIUS = 0.075.meters

    // m^2
    val FUEL_FRONTAL_AREA = FUEL_RADIUS.asMeters.pow(2) * Math.PI

    val FUEL_MASS = 0.2172.kilograms

    val HUB_HEIGHT = 65.0.inches

    // Percent of surface speed of shooter that gets transferred into the ball
    val SHOOTER_EFFICIENCY = if (isCompBot) 0.69 else 0.69

    @get:AutoLogOutput(key = "aim target")
    val aimTarget: Translation2d
        get() {
            if (!Drive.useAprilTags) {
                FieldManager.goalPose
            }

            return if (isAimingAtGoal) {
                FieldManager.goalPose - calculateAimTargetOffset(FieldManager.goalPose, Shooter.hubTimeCurve)
            } else {
                FieldManager.passPose - calculateAimTargetOffset(FieldManager.passPose, Shooter.floorTimeCurve)
            }
        }

    val staticShotPos: Translation2d
        get() = if (Drive.heading.measure > 0.0.degrees) {
                if (isRedAlliance) {
                    FieldManager.upperRedStaticShotPosition
                } else {
                    FieldManager.upperBlueStaticShotPosition
                }
            } else {
                if (isRedAlliance) {
                    FieldManager.lowerRedStaticShotPosition
                } else {
                    FieldManager.lowerBlueStaticShotPosition
                }
            }

    // Calculates how fast the shooter should spin to make a shot.
    fun getShooterRPS(): AngularVelocity {
        return if (!Drive.useAprilTags) {
            hubSpeedCurve.get(staticShotPos.getDistance(aimTarget)).rotationsPerSecond
        } else if (Robot.isAutonomous) {
            (if (isAimingAtGoal) hubSpeedCurve.get(distanceToTarget.asFeet) else hubSpeedCurve.get(11.0)).rotationsPerSecond
        } else {
            (if (isAimingAtGoal || FieldManager.shouldRamp) hubSpeedCurve.get(distanceToTarget.asFeet) else floorSpeedCurve.get(distanceToTarget.asFeet)).rotationsPerSecond
        } / SHOOTER_GEAR_RATIO / SHOOTER_EFFICIENCY
    }


    // uses turret velocity to offset the aim target for sotm
    fun calculateAimTargetOffset(airTime: Double) : Translation2d {
        val turretVelocity = Translation2d(Turret.turretOffsetFromCenter.x * Drive.gyroYawRate.asRadiansPerSecond, Turret.turretOffsetFromCenter.y * Drive.gyroYawRate.asRadiansPerSecond).rotateBy(Drive.heading) + Drive.velocity

        return turretVelocity * airTime
    }

// iterative tof lut method.
    fun calculateAimTargetOffset(goalPos: Translation2d, timeCurve: InterpolatingTreeMap<Double, Double>) : Translation2d {
        val turretVelocity = Translation2d(Turret.turretOffsetFromCenter.x * Drive.gyroYawRate.asRadiansPerSecond, Turret.turretOffsetFromCenter.y * Drive.gyroYawRate.asRadiansPerSecond).rotateBy(Drive.heading) + Drive.velocity
        var offset : Translation2d = turretVelocity * timeCurve.get(Turret.turretTranslation.getDistance(
            goalPos).meters.asFeet)

        for (i in 1..11) {
            offset = turretVelocity * timeCurve.get(Turret.turretTranslation.getDistance(goalPos - offset).meters.asFeet)
        }

        return offset
    }

    val isAimingAtGoal get() = FieldManager.inScoringZone

    val distanceToTarget get() = Turret.turretTranslation.getDistance(aimTarget).absoluteValue.meters


    /**
     * Simulates a fuel shot with the given initial velocity, and calculates both the translational error and the time error
     * @return A pair of doubles, the first is the horizontal error from the target when the ball reaches a specific height, the second is the time error it took the ball to get there.
     */
    fun calcFuelErrors(vx: Double, vy: Double, goalPos: Translation2d, airTimeTarget: Double, printPoints: Boolean = false): Pair<Double, Double> {
        val fuel = FuelSim(Translation3d(), Translation3d(vx, 0.0, vy))
        var time = 0.0

        val points = mutableMapOf<Double, Double>()

        while (fuel.pos.z > goalPos.y || fuel.velocity.z >= 0.0) {
            fuel.update(0.02, 1)
            time += 0.02
            points[fuel.pos.x] = fuel.pos.z
        }

        if (printPoints) {
            points.forEach { (x, y) -> print("($x,$y),") }
            print("\n")
        }

        return Pair(fuel.pos.x - goalPos.x, time - airTimeTarget)
    }

    // Basically the same as calcFuelError, but for generating a shooter curve w/ constant exit velocity. Uses x value instead of y to tell if ball has reached hub, and calculates vertical error instead of horizontal.
    fun calcFuelHeightError(vx: Double, vy: Double, goalPos: Translation2d, printPoints: Boolean = false): Double {
        val fuel = FuelSim(Translation3d(), Translation3d(vx, 0.0, vy))
        var prevFuelPos = Translation2d()

        val points = mutableMapOf<Double, Double>()

        while (fuel.pos.x < goalPos.x) {
            prevFuelPos = Translation2d(fuel.pos.x, fuel.pos.z)
            fuel.update(0.02, 1)
            points[fuel.pos.x] = fuel.pos.z
        }
        if (printPoints) {
            var i = 1
            points.forEach { (x, y) -> print("(${x.round(4)},${y.round(4)})"); if (i <points.size) print(",");i++;}
            print("\n")
        }

        // draws a line between final 2 fuel position and calculates where line crosses goalpos
        val slope = (fuel.pos.z - prevFuelPos.y)/(fuel.pos.x - prevFuelPos.x)

        return (slope * (goalPos.x - prevFuelPos.x) + prevFuelPos.y) - goalPos.y
    }

    // calculates time it took fuel to get to the hub. Used to generate a time lut.
    fun calcFuelTime(vx: Double, vy: Double, goalPos: Translation2d): Double {
        val fuel = FuelSim(Translation3d(), Translation3d(vx, 0.0, vy))

        var t = 0.0

        while (fuel.pos.x < goalPos.x) {
            fuel.update(0.01, 1)
            t+=0.01
        }

        return t
    }

    // constant time method
    fun printShooterCurves(goalHeight: Distance, distRange: IntRange, airTime: Double, forCompBot: Boolean = true) {
        val angles = mutableMapOf<Double, Double>()
        val speeds = mutableMapOf<Double, Double>()

        for (i in distRange) {
            val dist = i.toDouble()
            val angleAndSpeed = calculateAngleAndSpeed(dist.feet, goalHeight, airTime)
            angles[dist] = angleAndSpeed.first
            speeds[dist] = angleAndSpeed.second.metersPerSecond.toWheelSpeed(forCompBot).asRotationsPerSecond
        }

        println("Angle Curve:")
        angles.forEach { (dist, angle) ->
            println("put(${dist.round(3)}, ${angle.round(3)})")
        }
        println("Speed Curve:")
        speeds.forEach { (dist, speed) ->
            println("put(${dist.round(3)}, ${speed.round(3)})")
        }
    }

    // constant(ish) exit velocity method. (lut)
    fun printShooterCurves(goalHeight: Distance, distRange: IntProgression, speedRange: Pair<Double, Double>, printTimeCurve: Boolean = true, forCompBot: Boolean = true) {

        val angles = mutableMapOf<Double, Double>()
        val speeds = mutableMapOf<Double, Double>()
        val times = mutableMapOf<Double, Double>()


        for (i in distRange) {

            val dist = i.toDouble()
            val speed = (((speedRange.second - speedRange.first)/(distRange.last.toDouble() - distRange.first.toDouble()))*(i.toDouble() - distRange.first.toDouble()) + speedRange.first).rotationsPerSecond.toExitVelocity(forCompBot).asMetersPerSecond
            val angleAndTime = calculateAngleAndTime(dist.feet, goalHeight, speed)

            angles[dist] = angleAndTime.first.asDegrees
            speeds[dist] = speed.metersPerSecond.toWheelSpeed(forCompBot).asRotationsPerSecond
            times[dist] = angleAndTime.second
        }
        println("Angle Curve:")
        angles.forEach { (dist, angle) ->
            println("put(${dist.round(3)}, ${angle.round(3)})")
        }
        println("Speed Curve:")
        speeds.forEach { (dist, speed) ->
            println("put(${dist.round(3)}, ${speed.round(3)})")
        }

        if (printTimeCurve) {
            println("Time Curve:")
            times.forEach { (dist, time) ->
                println("put(${dist.round(3)}, ${time.round(3)})")
            }
        }

    }

    // constant hood angle.
    fun printShooterCurves(goalHeight: Distance, distRange: IntProgression, shotAngle: Angle, printTimeCurve: Boolean = true, forCompBot: Boolean = true) {
        println("Angle: $shotAngle")
        val speeds = mutableMapOf<Double, Double>()
        val times = mutableMapOf<Double, Double>()

        for (i in distRange) {
            val dist = i.toDouble()

            val speedAndTime = calculateSpeedAndTime(dist.feet, goalHeight, shotAngle)

            speeds[dist] = speedAndTime.first.toWheelSpeed(forCompBot).asRotationsPerSecond
            times[dist] = speedAndTime.second
        }

        println("Speed Curve:")
        speeds.forEach { (dist, speed) ->
            println("put(${dist.round(3)}, ${speed.round(3)})")
        }

        if (printTimeCurve) {
            println("Time Curve:")
            times.forEach { (dist, time) ->
                println("put(${dist.round(3)}, ${time.round(3)})")
            }
        }

    }

    fun printPassTimes(distRange: IntProgression, shotSpeed: AngularVelocity, forCompBot: Boolean = true) {
        val angle = 45.0.degrees
        val v0 = shotSpeed.toExitVelocity(forCompBot).asFeetPerSecond
        for (i in distRange) {
            val dist = i.toDouble()
            val t = dist / (v0 * angle.cos())
            println("put(${dist}, ${t.round(3)})")
        }
    }

    // angle in degrees, speed in m/s
    // Performs newtons method in 2 dimensions to estimate the angle and exit velocity needed to make shot from the given distance with the given airtime
    fun calculateAngleAndSpeed(distFromGoal: Distance, goalHeight: Distance, airTime: Double): Pair<Double, Double> {
        val maxTError = 0.1
        val maxDError = 0.1

        val toTarget: Translation2d = Translation2d(distFromGoal, goalHeight)

        // in vx and vy, will convert to angle and velocity at the end
        // this is derived from kinematic equations and assumes no drag
        var guess = Pair(toTarget.x / airTime, (toTarget.y + 0.5 * G * airTime.pow(2)) / airTime)
        var guessIncremented = Pair(guess.first + 0.1, guess.second + 0.1)


        var errors = calcFuelErrors(guess.first, guess.second, toTarget, airTime)
        var errorSum = errors.first.absoluteValue + errors.second.absoluteValue


        var errorsIncremented = calcFuelErrors(guessIncremented.first, guessIncremented.second, toTarget, airTime)
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

            // Then find the t value for which z is 0
            //  t = -z1       / (           z2       -  z1      )
            val t = -errorSum / (errorSumIncremented - errorSum)

            guess = Pair(x1+(x2-x1) * t, y1+(y2-y1) * t)

            guessIncremented = Pair(guess.first + 0.1, guess.second + 0.1)

            errors = calcFuelErrors(guess.first, guess.second, toTarget, airTime)
            errorSum = errors.first.absoluteValue + errors.second.absoluteValue

            errorsIncremented = calcFuelErrors(guessIncremented.first, guessIncremented.second, toTarget, airTime)
            errorSumIncremented = errorsIncremented.first.absoluteValue + errorsIncremented.second.absoluteValue

            num++
        }

        return Pair(atan2(guess.second, guess.first).radians.asDegrees, sqrt(guess.first.pow(2) + guess.second.pow(2)))
    }

    // same as get angle and speed, except calculates angle for given exit velocity, and calculates time for tof lut
    fun calculateAngleAndTime(distFromGoal: Distance, goalHeight: Distance, speed: Double): Pair<Angle, Double> {

        val toTarget: Translation2d = Translation2d(distFromGoal, goalHeight)

        println("x=${toTarget.x}\ny=${toTarget.y}")

        var guess = if (distFromGoal > 7.0.feet) 75.0 else 84.0
        var guessIncremented = guess + 0.1

        var error = calcFuelHeightError(speed * guess.degrees.cos(), speed * guess.degrees.sin(), toTarget, true)
        var errorIncremented = calcFuelHeightError(speed * guessIncremented.degrees.cos(), speed * guessIncremented.degrees.sin(), toTarget, false)

        var num = 0
        while (error.absoluteValue >= 0.1 && num < 10 && guess != 0.0) {
            var slope = (errorIncremented - error)/(guessIncremented - guess)

            guess = (-error/slope + guess).coerceIn(0.0, 85.0)

            guessIncremented = guess + 0.1

            error = calcFuelHeightError(speed * guess.degrees.cos(), speed * guess.degrees.sin(), toTarget, true)
            errorIncremented = calcFuelHeightError(speed * guessIncremented.degrees.cos(), speed * guessIncremented.degrees.sin(), toTarget, false)

            num++
        }



        return Pair(guess.degrees, calcFuelTime(speed * guess.degrees.cos(), speed * guess.degrees.sin(), toTarget))
    }

    fun calculateSpeedAndTime(distFromGoal: Distance, goalHeight: Distance, exitAngle: Angle): Pair<LinearVelocity, Double> {
        println("x=${distFromGoal.asMeters}")

        val toTarget: Translation2d = Translation2d(distFromGoal, goalHeight)

        var guess = if (distFromGoal.asFeet < 40.0) 5.0 else 15.0
        var guessIncremented = guess + 0.1

        var error = calcFuelHeightError(guess * exitAngle.cos(), guess * exitAngle.sin(), toTarget, true)
        var errorIncremented = calcFuelHeightError(guessIncremented * exitAngle.cos(), guessIncremented * exitAngle.sin(), toTarget, false)

        var num = 0
        while (error.absoluteValue >= 0.05 && num < 10 && guess != 0.0) {
            val slope = (errorIncremented - error)/(guessIncremented-guess)

            guess = (-error/slope + guess)

            guessIncremented = guess + 0.1

            error = calcFuelHeightError(guess * exitAngle.cos(), guess * exitAngle.sin(), toTarget, true)
            errorIncremented = calcFuelHeightError(guessIncremented * exitAngle.cos(), guessIncremented * exitAngle.sin(), toTarget, false)

            num++
        }

        return Pair(guess.metersPerSecond, calcFuelTime(guess * exitAngle.cos(), guess * exitAngle.sin(), toTarget))
    }

    fun LinearVelocity.toWheelSpeed(forCompBot: Boolean = true): AngularVelocity {
        return (((if (forCompBot) 1.0 else 2.0) * this.asInchesPerSecond/(Shooter.WHEEL_DIAMETER.asInches * Math.PI)).rotationsPerSecond)
    }

    fun AngularVelocity.toExitVelocity(forCompBot: Boolean = true): LinearVelocity {
        return (this.asRotationsPerSecond * (Shooter.WHEEL_DIAMETER.asInches * Math.PI) / (if (forCompBot) 1.0 else 2.0)).inchesPerSecond
    }
}