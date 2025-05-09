# Choreo

[![Discord](https://img.shields.io/discord/975739302933856277?color=%23738ADB&label=Join%20our%20Discord&logo=discord&logoColor=white)](https://discord.gg/ad2EEZZwsS)

![A screenshot of Choreo with an example path](./docs/media/readmeScreenshot.png)

Choreo (_Constraint-Honoring Omnidirectional Route Editor and Optimizer_, pronounced like choreography) is a graphical tool for planning time-optimized trajectories for autonomous mobile robots in the FIRST Robotics Competition.

## Download and Install

Grab the latest release for your platform on the [releases](https://github.com/SleipnirGroup/Choreo/releases) page.

## Usage and Documentation

Check out the [docs](https://choreo.autos), which covers installation, usage, building, and robot code integration with ChoreoLib.

## Repository directory structure

- [src](src) - Choreo frontend
- [src-tauri](src-tauri) - Choreo backend
- [src-cli](src-cli) - Choreo command-line generation tool
- [src-core](src-core) - Rust code shared between the app and the CLI
- [choreolib](choreolib) - ChoreoLib: robot-side library for loading and following Choreo paths
- [trajoptlib](trajoptlib) - TrajoptLib: library used by Choreo to generate time-optimal trajectories for FRC robots
- [docs](docs) - Source for the [documentation](https://choreo.autos)

## Authors

<a href="https://github.com/SleipnirGroup/Choreo/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=SleipnirGroup/Choreo" />
</a>

The 2024 field background was traced from the field renders provided by [MikLast](https://www.chiefdelphi.com/t/2024-crescendo-top-down-field-renders/447764).
