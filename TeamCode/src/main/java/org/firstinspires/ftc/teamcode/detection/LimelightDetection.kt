package org.firstinspires.ftc.teamcode.detection

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import com.rowanmcalpin.nextftc.core.units.deg
import org.opencv.core.Point
import org.opencv.core.Point3
import java.util.LinkedList
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt
import kotlin.math.tan

@Config
class LimelightDetection(hardwareMap: HardwareMap) {
    companion object {
        @JvmField
        var ASPECT_K: Double = 1.2

        @JvmField
        var CENTER_WX: Double = 2.0

        @JvmField
        var CENTER_WY: Double = 1.0

        @JvmField
        var MODE: Int = 2

        @JvmField
        var SECOND_MODE: Int = -1

        @JvmField
        var X_CAMERA: Double = -4.7

        @JvmField
        var Y_CAMERA: Double = -15.8

        @JvmField
        var Z_CAMERA: Double = 28.0

        @JvmField
        var PITCH_CAMERA: Double = 58.4

        @JvmField
        var MAX_FRAMES: Int = 5

        @JvmField
        var xViewWidth = 8.5

        @JvmField
        var yViewHeightTop = 8.5

        @JvmField
        var yViewHeightBottom = -4.5
    }

    var sampleYaw: Double = 0.0
    var aspectRatio: Double = 0.0

    private var vertices: List<List<Double>>? = null
    private var targetIndex: Int = -1
    var horizontalAngle: Double = 0.0
    var verticalAngle: Double = 0.0
    var resultExists: Boolean = false
    private var lastScore: Double = 0.0
    private var lastYaw: Int = 0
    private var wComp: Double = 0.0
    private var hComp: Double = 0.0

    private var w: Double = 0.0
    private var h: Double = 0.0

    private fun resetVariables() {
        targetIndex = -1
        resultExists = false
    }

    private var worldCoordinatesWithoutMedian: Point3? = null
    var worldCoordinates: Point3? = null

    private val detectionBuffer = LinkedList<Point3>()

    private var points: Array<Point?> = arrayOfNulls(4)

    private var limelight: Limelight3A = hardwareMap.get(Limelight3A::class.java, "limelight")

    init {
        limelight.pipelineSwitch(3)
        limelight.setPollRateHz(10)
        limelight.start()
    }

    private fun isGoodColor(target: DetectorResult): Boolean {
        return (target.classId == MODE || target.classId == SECOND_MODE)
    }

    private fun isWithinRange(tx: Double, ty: Double): Boolean {
        return ty < yViewHeightTop && ty > yViewHeightBottom && abs(tx) <= xViewWidth
    }

    // Smaller = closer to image center.
    // Uses degrees directly; weights let you prefer horizontal/vertical centering.
    private fun centerScore(txDeg: Double, tyDeg: Double): Double {
        return CENTER_WX * txDeg * txDeg + CENTER_WY * tyDeg * tyDeg
    }

    private fun getResult() {
        val result = limelight.latestResult
        resultExists = false
        targetIndex = -1

        if (result == null || !result.isValid) return

        val detectedTargets =
            result.detectorResults
        if (detectedTargets == null || detectedTargets.isEmpty()) return

        var bestScore = Double.POSITIVE_INFINITY
        var bestIdx = -1

        for (i in detectedTargets.indices) {
            val t = detectedTargets[i]

            val tx = t.targetXDegrees
            val ty = t.targetYDegrees

            if (!isGoodColor(t) || !isWithinRange(tx, ty)) continue

            // scor simplu: X cântărit mai tare decât Y (în centerScore)
            val score = centerScore(tx, ty)
            lastScore = score

            if (score < bestScore) {
                bestScore = score
                bestIdx = i

                // salvează unghiurile pentru restul pipeline-ului
                horizontalAngle = -Math.toRadians(tx)
                verticalAngle = Math.toRadians(ty)

                resultExists = true
            }
        }

        if (!resultExists || bestIdx == -1) return

        targetIndex = bestIdx

        // colțuri țintă
        vertices =
            detectedTargets[targetIndex].targetCorners
        if (vertices != null) {
            val n =
                min(vertices!!.size.toDouble(), 4.0)
                    .toInt()
            for (i in 0..<n) {
                points[i] = Point(
                    vertices!![i][0],
                    vertices!![i][1]
                )
            }
            // dacă vin mai puțin de 4 colțuri, protejează restul
            for (i in n..3) {
                points[i] = null
            }
        } else {
            for (i in 0..3) points[i] = null
        }
    }

    private fun getPositions() {
        val normalTargetAngle =
            PITCH_CAMERA.deg.inRad + verticalAngle
        val yDist =
            Z_CAMERA * tan(normalTargetAngle)
        val cameraTargetDist =
            sqrt(Z_CAMERA * Z_CAMERA + yDist * yDist)
        val xDist =
            cameraTargetDist * tan(horizontalAngle)

        // Compute current detection position
        val worldX = X_CAMERA + xDist
        val worldY = Y_CAMERA + yDist

        // Add to buffer
        detectionBuffer.add(Point3(worldX, worldY, 0.0))
        if (detectionBuffer.size > MAX_FRAMES) {
            detectionBuffer.removeFirst() // Keep buffer size fi// xed
        }

        // Compute rolling average
        var sumX = 0.0
        var sumY = 0.0
        for (p in detectionBuffer) {
            sumX += p.x
            sumY += p.y
        }

        worldCoordinates = Point3(
            sumX / detectionBuffer.size,
            sumY / detectionBuffer.size,
            0.0
        )

        worldCoordinatesWithoutMedian =
            Point3(worldX, worldY, 0.0)
    }

    private fun getYaw() {
        // Need 4 corners
        if (points[0] == null || points[1] == null || points[2] == null || points[3] == null
        ) {
            sampleYaw =
                if (lastYaw == 1) Math.PI / 2 else 0.0
            return
        }

        // Axis-aligned bbox in image pixels
        val minX = min(
            min(points[0]!!.x, points[1]!!.x),
            min(points[2]!!.x, points[3]!!.x)
        )
        val maxX = max(
            max(points[0]!!.x, points[1]!!.x),
            max(points[2]!!.x, points[3]!!.x)
        )
        val minY = min(
            min(points[0]!!.y, points[1]!!.y),
            min(points[2]!!.y, points[3]!!.y)
        )
        val maxY = max(
            max(points[0]!!.y, points[1]!!.y),
            max(points[2]!!.y, points[3]!!.y)
        )

        w = max(0.0, maxX - minX)
        h = max(0.0, maxY - minY)

        // --- Foreshortening compensation ---
        // Vertical squash depends on camera pitch + current vertical offset
        val cosV = max(
            0.2,
            abs(cos(PITCH_CAMERA.deg.inRad + verticalAngle))
        )
        // Horizontal squash depends on current horizontal offset
        val cosH = max(
            0.2,
            abs(cos(horizontalAngle))
        )

        wComp = w / cosH
        hComp = h / cosV

        // Aspect after compensation; ASPECT_K lets you nudge "straight" to ≈1.0
        val aspect =
            (wComp / max(1e-6, hComp)) * ASPECT_K
        aspectRatio = aspect

        if (lastYaw == 0) {                 // currently vertical (0 rad)
            if (aspect > 1) lastYaw =
                1 // flip to horizontal (π/2)
        } else {                              // currently horizontal
            if (aspect < 1) lastYaw = 0 // flip back to vertical
        }

        sampleYaw =
            if (lastYaw == 1) Math.PI / 2 else 0.0
    }

    fun runDetection() {
        resetVariables()
        getResult()

        if (resultExists) {
            getPositions()
            getYaw()
        }
    }
}
