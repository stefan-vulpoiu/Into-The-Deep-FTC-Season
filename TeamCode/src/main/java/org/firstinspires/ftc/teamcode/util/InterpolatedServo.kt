package org.firstinspires.ftc.teamcode.util

import androidx.core.math.MathUtils
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction
import java.util.stream.Stream

/**
 *
 * Servomotor wrapper that uses cubic spline interpolation to restrict itself to a relevant range.
 *
 * Can be used to synchronize servomotors together or to mimic real angles to account for inaccurate potentiometer measurements/angle conversions.
 */
class InterpolatedServo(
    hardwareMap: HardwareMap,
    servoName: String,
    private var maxAngle: Double = 1.0,
) {
    private val servo = hardwareMap.get(Servo::class.java, servoName)

    private lateinit var interpolation: PolynomialSplineFunction

    init {
        generatePositions(
            Pair(0.0, 0.0),
            Pair(maxAngle, maxAngle)
        )
    }

    var position: Double = 0.0
        set(value) {
            if (!interpolation.isValidPoint(value)) {
                val knots: DoubleArray = interpolation.getKnots()
                servo.position = MathUtils.clamp(
                    value,
                    knots[0],
                    knots[knots.size - 1]
                )
            } else {
                servo.position = interpolation.value(value) / maxAngle
                field = value
            }
        }

    /**
     *
     * Interpolates the servo's position to a relevant angle range using a cubic spline.
     *
     * Typical usage consists of restricting the servomotor to certain angles with a range from 0 to 1.
     *
     * @param controlPoints A list of input : output pairs for the generator to use (must use at least 2 pairs)
     */
    @SafeVarargs
    fun generatePositions(vararg controlPoints: Pair<Double, Double>) {
        val x = Stream.of(*controlPoints)
            .mapToDouble { p: Pair<Double, Double> -> p.first }
            .toArray()
        val y = Stream.of(*controlPoints)
            .mapToDouble { p: Pair<Double, Double> -> p.second }
            .toArray()

        interpolation = if (x.size == 2) LinearInterpolator().interpolate(x, y)
        else SplineInterpolator().interpolate(x, y)
    }

    fun setInverted(isInverted: Boolean) {
        servo.direction = if (isInverted) Servo.Direction.REVERSE else Servo.Direction.FORWARD
    }
}
