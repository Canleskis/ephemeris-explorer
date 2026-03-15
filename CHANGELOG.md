# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## Unreleased - 2025-10-08

### Added

- Experimental way to edit when a burn starts by dragging.
- Automatic in-SOI plotting.
- SOI visualisations.
- Dashed lines for trajectory segments where a burn is happening.

### Changed

- Improved performance and accuracy of integrators.
- Trajectory always display segments after a significant event like a burn or an SOI change.

### Fixed

- Flight plan recomputations inconsistencies when many changes are made in a short period.
- Wront results when the flight plan is changed.
- Auto-extend inconsistent behavior.

## [0.1.2] - 2025-03-01

### Changed

- Decrease RAM usage of loaded textures.

## [0.1.1] - 2025-02-25

### Fixed

- Fix crash on MacOS.

## 0.1.0 - 2025-02-25

- Initial release

[0.1.2]: https://github.com/Canleskis/ephemeris-explorer/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/Canleskis/ephemeris-explorer/compare/v0.1.0...v0.1.1
