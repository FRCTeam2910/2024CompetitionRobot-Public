package frc.robot.config;

/**
 * Configuration for {@link frc.robot.commands.FollowPathCommand}
 *
 * PID default values are all 0.0.
 * Feed-forward coefficient default values are all 1.0.
 */
public final class FollowPathConfiguration {
    private double translationKp;
    private double translationKi;
    private double translationKd;
    private double rotationKp;
    private double rotationKi;
    private double rotationKd;
    private double feedforwardVelocityCoefficient;
    private double feedforwardAccelerationCoefficient;

    /**
     * Create a new configuration.
     */
    public FollowPathConfiguration(
            double translationKp,
            double translationKi,
            double translationKd,
            double rotationKp,
            double rotationKi,
            double rotationKd,
            double feedforwardVelocityCoefficient,
            double feedforwardAccelerationCoefficient) {
        this.translationKp = translationKp;
        this.translationKi = translationKi;
        this.translationKd = translationKd;
        this.rotationKp = rotationKp;
        this.rotationKi = rotationKi;
        this.rotationKd = rotationKd;
        this.feedforwardVelocityCoefficient = feedforwardVelocityCoefficient;
        this.feedforwardAccelerationCoefficient = feedforwardAccelerationCoefficient;
    }

    /**
     * Create a new configuration.
     */
    public FollowPathConfiguration() {
        this(0, 0, 0, 0, 0, 0, 1.0, 1.0);
    }

    /**
     * Get the translation proportional gain.
     */
    public double getTranslationKp() {
        return translationKp;
    }

    /**
     * Set the translation proportional gain.
     *
     * @return This <code>FollowPathConfiguration</code>.
     */
    public FollowPathConfiguration withTranslationKp(final double translationKp) {
        this.translationKp = translationKp;

        return this;
    }

    /**
     * Get the translation integral gain.
     */
    public double getTranslationKi() {
        return translationKi;
    }

    /**
     * Set the translation integral gain.
     *
     * @return This <code>FollowPathConfiguration</code>.
     */
    public FollowPathConfiguration withTranslationKi(final double translationKi) {
        this.translationKi = translationKi;

        return this;
    }

    /**
     * Get the translation differential gain.
     */
    public double getTranslationKd() {
        return translationKd;
    }

    /**
     * Set the translation differential gain.
     *
     * @return This <code>FollowPathConfiguration</code>.
     */
    public FollowPathConfiguration withTranslationKd(final double translationKd) {
        this.translationKd = translationKd;

        return this;
    }

    /**
     * Get the rotation proportional gain.
     */
    public double getRotationKp() {
        return rotationKp;
    }

    /**
     * Set the rotation proportional gain.
     *
     * @return This <code>FollowPathConfiguration</code>.
     */
    public FollowPathConfiguration withRotationKp(final double rotationKp) {
        this.rotationKp = rotationKp;

        return this;
    }

    /**
     * Get the rotation integral gain.
     */
    public double getRotationKi() {
        return rotationKi;
    }

    /**
     * Set the rotation integral gain.
     *
     * @return This <code>FollowPathConfiguration</code>.
     */
    public FollowPathConfiguration withRotationKi(final double rotationKi) {
        this.rotationKi = rotationKi;

        return this;
    }

    /**
     * Get the rotation differential gain.
     */
    public double getRotationKd() {
        return rotationKd;
    }

    /**
     * Set the rotation differential gain.
     *
     * @return This <code>FollowPathConfiguration</code>.
     */
    public FollowPathConfiguration withRotationKd(final double rotationKd) {
        this.rotationKd = rotationKd;

        return this;
    }

    /**
     * Get the feed-forward velocity coefficient.
     */
    public double getFeedforwardVelocityCoefficient() {
        return feedforwardVelocityCoefficient;
    }

    /**
     * Set the feed-forward velocity coefficient.
     *
     * @return This <code>FollowPathConfiguration</code>
     */
    public FollowPathConfiguration withFeedforwardVelocityCoefficient(final double feedforwardVelocityCoefficient) {
        this.feedforwardVelocityCoefficient = feedforwardVelocityCoefficient;

        return this;
    }

    /**
     * Get the feed-forward acceleration coefficient.
     */
    public double getFeedforwardAccelerationCoefficient() {
        return feedforwardAccelerationCoefficient;
    }

    /**
     * Set the feed-forward acceleration coefficient.
     *
     * @return This <code>FollowPathConfiguration</code>
     */
    public FollowPathConfiguration withFeedforwardAccelerationCoefficient(
            final double feedforwardAccelerationCoefficient) {
        this.feedforwardAccelerationCoefficient = feedforwardAccelerationCoefficient;

        return this;
    }
}
