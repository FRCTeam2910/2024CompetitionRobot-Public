package frc.robot.config;

public interface RobotConstants {
    DrivetrainConfiguration getDrivetrainConfiguration();

    PortConfiguration getPortConfiguration();

    Pigeon2Constants getPigeon2Constants();

    TurretConfiguration getTurretConfiguration();

    LimelightConfiguration getLimelightConfiguration();

    FollowPathConfiguration getFollowPathConfiguration();

    static RobotConstants getRobotConstants(RobotIdentity robot) {
        switch (robot) {
            case LOKI:
                return new Loki();
            case TYPHOON:
            case SIMULATION:
            default:
                // Something went wrong if this branch is reached, by default we will return our Comp Bot
                return new Typhoon();
        }
    }
}
