package frc.team2471.frc2026

import frc.team2471.frc2026.AimUtils.AIR_DENSITY
import frc.team2471.frc2026.AimUtils.FUEL_DRAG_COEFFICIENT
import frc.team2471.frc2026.AimUtils.FUEL_FRONTAL_AREA
import frc.team2471.frc2026.AimUtils.FUEL_MASS
import frc.team2471.frc2026.AimUtils.G
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.units.asKilograms
import org.wpilib.math.geometry.Translation3d
import org.wpilib.system.Timer
import kotlin.math.pow

class FuelSim(val x0: Translation3d, val v0: Translation3d) {
    var pos = x0
    var velocity = v0
    var prevVelocity = v0
    var prevAcceleration = Translation3d(0.0, 0.0, -G)
    var prevT = Timer.getMonotonicTimestamp()

    fun update(numSteps: Int = 1) {
        // Time from last update
        val deltaT = Timer.getMonotonicTimestamp() - prevT


        update(deltaT, numSteps)
    }

    fun update(deltaT: Double, numSteps: Int) {
        // time between each step
        val dt = deltaT / numSteps

        for (i in 0..<numSteps) {
            val acceleration = Translation3d(0.0, 0.0, -G) + calcDragAccel(prevVelocity)

            velocity += ((prevAcceleration + acceleration) / 2.0).times(dt)

            pos += ((prevVelocity + velocity) / 2.0).times(dt)

            prevAcceleration = acceleration
            prevVelocity = velocity
        }

        prevT += deltaT
    }
}

fun logFuel(name: String, vararg fuel: FuelSim) {
    val fuelPositions = fuel.map { it.pos }.toTypedArray()
    Logger.recordOutput(name, *fuelPositions)
}

fun MutableList<FuelSim>.removeFuel() {
    this.removeIf { it.pos.z < 0.0 }
}
private fun calcDragAccel(v: Translation3d): Translation3d {
    val length = v.norm

    val newMagnitude = calcDragAccel(length)

    return v.times(-newMagnitude / length)

}
private fun calcDragAccel(speed: Double): Double {
    //      1/2       rho           v^2                          Cd          Frontal Area             mass
    return (0.5 * AIR_DENSITY * speed.pow(2) * FUEL_DRAG_COEFFICIENT * FUEL_FRONTAL_AREA) / FUEL_MASS.asKilograms
}
