from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import TypeGuard

from choreo.util import DEFAULT_YEAR, get_flipper_for_year
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds


def lerp(a, b, t) -> float:
    return a + (b - a) * t


@dataclass
class EventMarker:
    """
    A marker for an event in a trajectory.
    """

    timestamp: float
    event: str

    def offset_by(self, timestamp_offset: float):
        """
        Returns a new EventMarker with the timestamp offset by the specified
        amount.

        Parameter ``timestamp_offset``:
            The amount to offset the timestamp by.

        Returns:
            A new EventMarker with the timestamp offset by the specified amount.
        """
        return EventMarker(self.timestamp + timestamp_offset, self.event)


def marker_is_not_none(marker: EventMarker | None) -> TypeGuard[EventMarker]:
    return marker is not None


def load_event_marker(event) -> EventMarker | None:
    try:
        offset = float(event["from"]["offset"]["val"])
        target_timestamp = float(event["from"]["targetTimestamp"])
        name = event["name"]
        if target_timestamp + offset < 0 or len(name) == 0:
            return None
        return EventMarker(target_timestamp + offset, name)
    except TypeError:
        return None


@dataclass
class DifferentialSample:
    """
    Constructs a DifferentialSample with the specified parameters.

    Parameter ``timestamp``:
        The timestamp of this state, relative to the beginning of the
        trajectory.

    Parameter ``x``:
        The X position of the state in meters.

    Parameter ``y``:
        The Y position of the state in meters.

    Parameter ``heading``:
        The heading of the state in radians, with 0 being in the +X
        direction.

    Parameter ``vl``:
        The left linear velocity of the state in m/s.

    Parameter ``vr``:
        The right linear velocity of the state in m/s.

    Parameter ``omega``:
        The chassis angular velocity of the state in rad/s.

    Parameter ``al``:
        The left linear acceleration of the state in m/s².

    Parameter ``ar``:
        The right linear acceleration of the state in m/s².

    Parameter ``fl``:
        The left force on the swerve modules in Newtons.

    Parameter ``fr``:
        The right force on the swerve modules in Newtons.
    """

    # The timestamp of this state, relative to the beginning of the trajectory.
    timestamp: float
    x: float
    y: float
    heading: float
    vl: float
    vr: float
    omega: float
    al: float
    ar: float
    fl: float
    fr: float

    def get_pose(self) -> Pose2d:
        """
        Returns the pose at this state.
        """
        return Pose2d(self.x, self.y, Rotation2d(value=self.heading))

    def get_chassis_speeds(self) -> ChassisSpeeds:
        """
        Returns the field-relative chassis speeds of this state.
        """

        return ChassisSpeeds((self.vl + self.vr) / 2.0, 0.0, self.omega)

    def interpolate(
        self, end_value: DifferentialSample, t: float
    ) -> DifferentialSample:
        """
        Interpolate between this state and the provided state.

        Parameter ``end_value``:
            The next state. It should have a timestamp after this state.

        Parameter ``t``:
            The timestamp of the interpolated state. It should be between this
            state and end_value.

        Returns:
            The interpolated state.
        """
        scale = (t - self.timestamp) / (end_value.timestamp - self.timestamp)

        return DifferentialSample(
            t,
            lerp(self.x, end_value.x, scale),
            lerp(self.y, end_value.y, scale),
            lerp(self.heading, end_value.heading, scale),
            lerp(self.vl, end_value.vl, scale),
            lerp(self.vr, end_value.vr, scale),
            lerp(self.omega, end_value.omega, scale),
            lerp(self.al, end_value.al, scale),
            lerp(self.ar, end_value.ar, scale),
            lerp(self.fl, end_value.fl, scale),
            lerp(self.fr, end_value.fr, scale),
        )

    def flipped(self, year: int = DEFAULT_YEAR) -> DifferentialSample:
        """
        Returns the current sample flipped based on the field year.

        Parameter ``year``:
            The field year (default: the current year).
        """
        flipper = get_flipper_for_year(year)
        if flipper.IS_MIRRORED:
            return DifferentialSample(
                self.timestamp,
                flipper.flip_x(self.x),
                flipper.flip_y(self.y),  # No-op for mirroring
                flipper.flip_heading(self.heading),
                self.vr,
                self.vl,
                -self.omega,
                self.ar,
                self.al,
                self.fr,
                self.fl,
            )
        else:
            return DifferentialSample(
                self.timestamp,
                flipper.flip_x(self.x),
                flipper.flip_y(self.y),
                flipper.flip_heading(self.heading),
                self.vl,
                self.vr,
                self.omega,
                self.al,
                self.ar,
                self.fl,
                self.fr,
            )


@dataclass
class DifferentialTrajectory:
    """
    Constructs a Trajectory with the specified parameters.

    Parameter ``name``:
        The name of the trajectory.

    Parameter ``samples``:
        A vector containing a list of Samples.

    Parameter ``splits``:
        The indices of the splits in the trajectory.

    Parameter ``events``:
        The events in the trajectory.
    """

    name: str
    samples: list[DifferentialSample]
    splits: list[int]
    events: list[EventMarker]

    def __sample_internal(self, timestamp: float) -> DifferentialSample | None:
        if len(self.samples) == 0:
            return None
        if len(self.samples) == 1:
            return self.samples[0]
        # Handle timestamps outside the trajectory range
        if timestamp < self.samples[0].timestamp:
            return self.samples[0]
        if timestamp > self.get_total_time():
            return self.samples[-1]

        # Binary search to find the two states on either side of the requested timestamps
        low = 0
        high = len(self.samples) - 1
        while low != high:
            mid = math.floor((low + high) / 2)
            if self.samples[mid].timestamp < timestamp:
                low = mid + 1
            else:
                high = mid

        # Handle case near start of trajectory
        if low == 0:
            return self.samples[0]

        # Find the states on either side of the requested time
        behind_state = self.samples[low - 1]
        ahead_state = self.samples[low]

        if ahead_state.timestamp - behind_state.timestamp < 1e-6:
            # meh states are so close, just give back one of them
            return ahead_state

        # Perform the actual interpolation
        return behind_state.interpolate(ahead_state, timestamp)

    def sample_at(
        self, timestamp: float, flip_for_red_alliance: bool = False
    ) -> DifferentialSample | None:
        """
        Return an interpolated sample of the trajectory at the given timestamp.

        Parameter ``timestamp``:
            The timestamp of this sample relative to the beginning of the
            trajectory.

        Parameter ``flip_for_red_alliance``:
            Whether or not to return the sample as flipped for the current year.

        Returns:
            The Sample at the given time.
        """
        tmp = self.__sample_internal(timestamp)
        if tmp is None:
            return None
        return tmp.flipped() if flip_for_red_alliance else tmp

    def get_samples(self) -> list[DifferentialSample]:
        """
        Returns the list of states for this trajectory.
        """
        return self.samples

    def get_initial_pose(self, flip_for_red_alliance: bool = False) -> Pose2d | None:
        """
        Returns the initial pose of the trajectory.

        Parameter ``flip_for_red_alliance``:
            Whether or not to return the Pose flipped.
        """

        if len(self.samples) == 0:
            return None
        return (
            self.samples[0].flipped().get_pose()
            if flip_for_red_alliance
            else self.samples[0].get_pose()
        )

    def get_final_pose(self, flip_for_red_alliance: bool = False) -> Pose2d | None:
        """
        Returns the final pose of the trajectory.

        Parameter ``flip_for_red_alliance``:
            Whether or not to return the Pose flipped.
        """
        if len(self.samples) == 0:
            return None
        return (
            self.samples[-1].flipped().get_pose()
            if flip_for_red_alliance
            else self.samples[-1].get_pose()
        )

    def get_total_time(self) -> float:
        """
        Returns the total time of the trajectory (the timestamp of the last
        sample).
        """
        if len(self.samples) == 0:
            return 0.0
        return self.samples[-1].timestamp

    def get_poses(self) -> list[Pose2d]:
        """
        Returns the array of poses corresponding to the trajectory.
        """
        return [x.get_pose() for x in self.samples]

    def flipped(self, year: int = DEFAULT_YEAR) -> DifferentialTrajectory:
        """
        Returns this trajectory flipped based on the field year.

        Parameter ``year``:
            The field year (default: the current year).
        """
        return DifferentialTrajectory(
            self.name, [x.flipped() for x in self.samples], self.splits, self.events
        )


@dataclass
class SwerveSample:
    """
    Constructs a SwerveSample with the specified parameters.

    Parameter ``timestamp``:
        The timestamp of this state, relative to the beginning of the
        trajectory.

    Parameter ``x``:
        The X position of the state in meters.

    Parameter ``y``:
        The Y position of the state in meters.

    Parameter ``heading``:
        The heading of the state in radians, with 0 being in the +X
        direction.

    Parameter ``vx``:
        The linear velocity of the state in the X direction in m/s.

    Parameter ``vy``:
        The linear velocity of the state in the Y direction in m/s.

    Parameter ``omega``:
        The angular velocity of the state in rad/s.

    Parameter ``ax``:
        The linear acceleration of the state in the X direction in m/s².

    Parameter ``ay``:
        The linear acceleration of the state in the Y direction in m/s².

    Parameter ``alpha``:
        The angular acceleration of the state in rad/s².

    Parameter ``fx``:
        The force on the swerve modules in the X direction in Newtons.

    Parameter ``fy``:
        The force on the swerve modules in the Y direction in Newtons.
    """

    timestamp: float
    x: float
    y: float
    heading: float
    vx: float
    vy: float
    omega: float
    ax: float
    ay: float
    alpha: float
    fx: list[float]
    fy: list[float]

    def get_pose(self) -> Pose2d:
        """
        Returns the pose at this state.
        """
        return Pose2d(self.x, self.y, Rotation2d(value=self.heading))

    def get_chassis_speeds(self) -> ChassisSpeeds:
        """
        Returns the field-relative chassis speeds of this state.
        """
        return ChassisSpeeds(self.vx, self.vy, self.omega)

    def interpolate(self, end_value: SwerveSample, t: float) -> SwerveSample:
        """
        Interpolate between this state and the provided state.

        Parameter ``end_value``:
            The next state. It should have a timestamp after this state.

        Parameter ``t``:
            The timestamp of the interpolated state. It should be between this
            state and end_value.

        Returns:
            The interpolated state.
        """
        scale = (t - self.timestamp) / (end_value.timestamp - self.timestamp)

        return SwerveSample(
            t,
            lerp(self.x, end_value.x, scale),
            lerp(self.y, end_value.y, scale),
            lerp(self.heading, end_value.heading, scale),
            lerp(self.vx, end_value.vx, scale),
            lerp(self.vy, end_value.vy, scale),
            lerp(self.omega, end_value.omega, scale),
            lerp(self.ax, end_value.ax, scale),
            lerp(self.ay, end_value.ay, scale),
            lerp(self.alpha, end_value.alpha, scale),
            [lerp(self.fx[i], end_value.fx[i], scale) for i in range(len(self.fx))],
            [lerp(self.fy[i], end_value.fy[i], scale) for i in range(len(self.fy))],
        )

    def flipped(self, year: int = DEFAULT_YEAR) -> SwerveSample:
        """
        Returns the current sample flipped based on the field year.

        Parameter ``year``:
            The field year (default: the current year).
        """
        flipper = get_flipper_for_year(year)
        if flipper.IS_MIRRORED:
            return SwerveSample(
                self.timestamp,
                flipper.flip_x(self.x),
                flipper.flip_y(self.y),
                flipper.flip_heading(self.heading),
                -self.vx,
                self.vy,
                -self.omega,
                -self.ax,
                self.ay,
                -self.alpha,
                [-self.fx[1], -self.fx[0], -self.fx[3], -self.fx[2]],
                [self.fy[1], self.fy[0], self.fy[3], self.fy[2]],
            )
        else:
            return SwerveSample(
                self.timestamp,
                flipper.flip_x(self.x),
                flipper.flip_y(self.y),
                flipper.flip_heading(self.heading),
                -self.vx,
                -self.vy,
                self.omega,
                -self.ax,
                -self.ay,
                self.alpha,
                [-x for x in self.fx],
                [-y for y in self.fy],
            )


@dataclass
class SwerveTrajectory:
    """
    Constructs a SwerveTrajectory with the specified parameters.

    Parameter ``name``:
        The name of the trajectory.

    Parameter ``samples``:
        A vector containing a list of Samples.

    Parameter ``splits``:
        The indices of the splits in the trajectory.

    Parameter ``events``:
        The events in the trajectory.
    """

    name: str
    samples: list[SwerveSample]
    splits: list[int]
    events: list[EventMarker]

    def __sample_internal(self, timestamp: float) -> SwerveSample | None:
        if len(self.samples) == 0:
            return None
        if len(self.samples) == 1:
            return self.samples[0]
        # Handle timestamps outside the trajectory range
        if timestamp < self.samples[0].timestamp:
            return self.samples[0]
        if timestamp > self.get_total_time():
            return self.samples[-1]

        # Binary search to find the two states on either side of the requested timestamps
        low = 0
        high = len(self.samples) - 1
        while low != high:
            mid = math.floor((low + high) / 2)
            if self.samples[mid].timestamp < timestamp:
                low = mid + 1
            else:
                high = mid

        # Handle case near start of trajectory
        if low == 0:
            return self.samples[0]

        # Find the states on either side of the requested time
        behind_state = self.samples[low - 1]
        ahead_state = self.samples[low]

        if ahead_state.timestamp - behind_state.timestamp < 1e-6:
            # meh states are so close, just give back one of them
            return ahead_state

        # Perform the actual interpolation
        return behind_state.interpolate(ahead_state, timestamp)

    def sample_at(
        self, timestamp: float, flip_for_red_alliance: bool = False
    ) -> SwerveSample | None:
        """
        Return an interpolated sample of the trajectory at the given timestamp.

        Parameter ``timestamp``:
            The timestamp of this sample relative to the beginning of the
            trajectory.

        Parameter ``flip_for_red_alliance``:
            Whether or not to return the sample as flipped for the current year.

        Returns:
            The Sample at the given time.
        """
        tmp = self.__sample_internal(timestamp)
        if tmp is None:
            return None
        return tmp.flipped() if flip_for_red_alliance else tmp

    def get_samples(self) -> list[SwerveSample]:
        """
        Returns the list of states for this trajectory.
        """
        return self.samples

    def get_initial_pose(self, flip_for_red_alliance: bool = False) -> Pose2d | None:
        """
        Returns the initial pose of the trajectory.

        Parameter ``flip_for_red_alliance``:
            Whether or not to return the Pose flipped.
        """

        if len(self.samples) == 0:
            return None
        return (
            self.samples[0].flipped().get_pose()
            if flip_for_red_alliance
            else self.samples[0].get_pose()
        )

    def get_final_pose(self, flip_for_red_alliance: bool = False) -> Pose2d | None:
        """
        Returns the final pose of the trajectory.

        Parameter ``flip_for_red_alliance``:
            Whether or not to return the Pose flipped.
        """

        if len(self.samples) == 0:
            return None
        return (
            self.samples[-1].flipped().get_pose()
            if flip_for_red_alliance
            else self.samples[-1].get_pose()
        )

    def get_total_time(self) -> float:
        """
        Returns the total time of the trajectory (the timestamp of the last
        sample).
        """

        if len(self.samples) == 0:
            return 0.0
        return self.samples[-1].timestamp

    def get_poses(self) -> list[Pose2d]:
        """
        Returns the array of poses corresponding to the trajectory.
        """
        return [x.get_pose() for x in self.samples]

    def flipped(self, year: int = DEFAULT_YEAR) -> SwerveTrajectory:
        """
        Returns this trajectory flipped based on the field year.

        Parameter ``year``:
            The field year (default: the current year).
        """
        return SwerveTrajectory(
            self.name, [x.flipped() for x in self.samples], self.splits, self.events
        )
