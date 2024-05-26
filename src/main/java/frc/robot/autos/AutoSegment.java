package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;

@SuppressWarnings("javadoc")
/**
 * Represents a pre-defined autonomous segment.
 *
 * @param start The segment's starting location
 * @param end   The segment's ending location
 * @param name  The segment's name
 * @param path  The {@link PathPlannerPath} for the segment
 */
public record AutoSegment(Location start, Location end, String name, PathPlannerPath path) {}
