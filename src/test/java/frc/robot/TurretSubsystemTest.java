package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.turret.TurretSubsystem;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

class TurretSubsystemTest {

    static final double EPSILON = 1e-6;
    static final double FORWARD_LIMIT = 720.0;
    static final double REVERSE_LIMIT = -720.0;

    @Test
    @DisplayName("turret")
    void setTurretTargetAngle() {

        double angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                90.0, Rotation2d.fromDegrees(0), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(90.0, angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                180.0, Rotation2d.fromDegrees(0), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(180.0, angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                270.0, Rotation2d.fromDegrees(0), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(-90.0, angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                360.0, Rotation2d.fromDegrees(0), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(0.0, angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                -90.0, Rotation2d.fromDegrees(0), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(-90.0, angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                -180.0, Rotation2d.fromDegrees(0), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(-180.0, angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                -270.0, Rotation2d.fromDegrees(0), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(90.0, angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                -360.0, Rotation2d.fromDegrees(0), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(0.0, angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                800.0, Rotation2d.fromDegrees(700), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals((700.0 - 260.0), angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                720.0, Rotation2d.fromDegrees(700), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(720.0, angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                -800.0, Rotation2d.fromDegrees(-700), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(-(700.0 - 260.0), angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                360.0, Rotation2d.fromDegrees(-360), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(0, angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                540, Rotation2d.fromDegrees(360), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(180, angle, EPSILON);

        angle = TurretSubsystem.convertToClosestBoundedTurretAngleDegrees(
                -540, Rotation2d.fromDegrees(-360), FORWARD_LIMIT, REVERSE_LIMIT);
        Assertions.assertEquals(-180, angle, EPSILON);
    }

    @Test
    @DisplayName("turret")
    void calculateTurretAngleFromCANCoderDegrees() {
        for (double i = -720.0; i < 720.0; i++) {
            var angle = TurretSubsystem.calculateTurretAngleFromCANCoderDegrees(
                    (i * 70.0 / 36.0) % 360.0, (i * 70.0 / 34.0) % 360.0);
            Assertions.assertEquals(i, angle, EPSILON);
        }
    }
}
