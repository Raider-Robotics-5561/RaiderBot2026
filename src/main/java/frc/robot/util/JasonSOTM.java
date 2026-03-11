package frc.robot.util;

public class JasonSOTM {
    /*@Override
public void execute() {
    ChassisSpeeds robotSpeed = fieldOrientedChassisSpeeds.get();
    Translation2d robotLocation = robotPose.get().getTranslation();
    Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    
    // 1. INITIAL GUESS
    // We start by assuming the robot is stationary to get a baseline distance
    double testDistance = robotLocation.getDistance(goalPosee.getTranslation());
    double predictedToF = 0.0;
    Translation2d futureTargetVec = new Translation2d();
    double requiredRPM = 0;

    // 2. ITERATIVE REFINEMENT (3 passes is usually plenty for FRC)
    for (int i = 0; i < 3; i++) {
        // Predict where the robot will be when the ball actually reaches the goal
        // (Base Latency + Calculated Flight Time)
        double totalLookAhead = latency + predictedToF;
        Translation2d futureRobotPos = robotLocation.plus(robotVelVec.times(totalLookAhead));
        
        // Calculate vector from future robot to goal
        futureTargetVec = goalPosee.getTranslation().minus(futureRobotPos);
        testDistance = futureTargetVec.getNorm();

        // Get the stationary RPM for this new distance
        double idealRPM = shooterTable.get(testDistance);
        double idealMs = calculateVelocityFromRPM(idealRPM);

        // Update ToF for the next iteration based on this speed
        predictedToF = testDistance / idealMs;
        requiredRPM = idealRPM; 
    }
    // 3. VECTOR COMPENSATION (Relative to the future position)
    // We need the ball to have a FIELD-RELATIVE velocity that matches the ideal shot
    double idealMs = calculateVelocityFromRPM(requiredRPM);
    Translation2d unitTargetVec = futureTargetVec.getAngle().getDegrees() != 0 ? 
                                  new Translation2d(1, futureTargetVec.getAngle()) : new Translation2d();
    
    // The "Shot Vector" is (Desired Field Velocity) - (Robot Velocity)
    Translation2d shotVec = unitTargetVec.times(idealMs).minus(robotVelVec);

    // 4. CONVERT TO ROBOT CONTROLS
    double fieldSpaceTurretAngle = shotVec.getAngle().getDegrees();
    double actualExitVelocityMs = shotVec.getNorm();
    double finalRPM = calculateRPMFromVelocity(actualExitVelocityMs);

    // 5. GYRO COMPENSATION & OUTPUT
    double robotHeading = robotPose.get().getRotation().getDegrees();
    double turretAngle = fieldSpaceTurretAngle - robotHeading - 180;
    
    // Normalize and Clamp (as in your original code)
    double normalizedAngle = MathUtil.inputModulus(turretAngle, -180, 180);
    double clampedAngle = -MathUtil.clamp(normalizedAngle, 
        TurretSubsystem.softLimitMin.in(Degrees), 
        TurretSubsystem.softLimitMax.in(Degrees));

    m_turret.setAngleSetpoint(Degrees.of(clampedAngle));
    m_launcher.setVelocitySetpoint(RPM.of(-finalRPM));
}
*/
/** Helper to reverse your RPM calculation */ /* 
private double calculateVelocityFromRPM(double rpm) {
    double flywheelRadiusMeters = FLYWHEEL_DIAMETER_METERS / 2;
    return (rpm * 2 * Math.PI * flywheelRadiusMeters) / 60.0;
} */
}
