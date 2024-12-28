# Ephemeris Explorer

<div align="center">
    <img src="ephemeris-explorer-preview.gif" alt="preview gif">
</div>

Ephemeris Explorer aims to be a multi-purpose interactive visualisation of gravitationally bound systems, be it stellar systems, planetary systems or satellite systems.
It uses concepts from various research, notably [Numerical Representation of Planetary Ephemerides](https://adsabs.harvard.edu/full/1989CeMec..45..305N) to generate accurate[^1] trajectories of celestial bodies and presents them in an interactive manner using the [Bevy Engine](https://bevyengine.org/) and [egui](https://github.com/emilk/egui).

[^1]: The accuracy of the current implementation is limited by the fact that the acceleration computation does not take into account general relativity and all bodies are considered to be spheres with an homogeneous mass distribution.

## User Interface and Key Bindings

On top of showing the bodies in the system and their trajectories, the user interface provides the following set of features:

- **Camera Controls**: The camera can be moved using the WASD keys, A and D for roll, as well as Space and Left Control to move up and down. The up and down arrow keys allow to change the FOV. The mouse can be used for pitch and yaw when the left mouse button is held down. You can also use orbit controls around the followed body by holding the right mouse button.

- **Hierarchy**: The left panel provides a hierarchical view of the celestial bodies within the current system. Clicking the ⌖ allows setting the reference frame of the camera. You can toggle trajectory plotting using the ○ button, and toggle trajectory plotting of the children of a body using the A button. Clicking a body's name or a body in the viewport selects it and allows you to read information about the body and configure plotting.

- **Body Information**: When a body is selected, information about the current state of the body is displayed. You can also configure the plotting of the body's trajectory. Changing the `Reference` of the body changes some of the displayed information and changes the reference frame of the body's trajectory. If the body is a ship, you can also export it and its flight plan to a file and delete it from the current system.

- **Flight Planning**: Allows to add manoeuvres to ships. Manoeuvres are defined by a start epoch, a duration and an acceleration in a reference frame. Using the same window that displays body information, you can add, remove and edit manoeuvres. When a manoeuvre is changed, the trajectory of the ship is recomputed asynchronously. If the flight plan is shorter than the current prediction bounds, the ship will freeze at the end of its trajectory.

- **Prediction Planner**: Allows to extend the ephemerides forward or backward in time independently and asynchronously by selecting start and end epochs. You can pause and cancel ongoing predictions. By default, predictions will auto-extend when the current time approaches the prediction bounds.

- **Trajectory Picking**: You can hover over or click on the trajectories to display information about the body at that point in time as well as adding manoeuvres for ships. Multiple points might be selected when picking a trajectory if the points are close together and belong to the same body.

- **Time Controls**: Allows to change the speed of the simulation, pause it or set the epoch.

- **Export**: Allows to export the current state of the system to a file, selecting which bodies are included.

- **Load**: Allows loading a system from a directory. The directory should contain four files: `ephemeris.json` for the ephemerides configuration, `hierarchy.json`, `state.json` for the initial state of the system, and a `skybox.png`. The initial state file should be a JSON containing the state of the system. See `export-solar-system` for a python program that generates such files using NASA's development ephemerides and [skyfield](https://rhodesmill.org/skyfield/). It should be formatted as follows:

```json
{
    "epoch": "2000-01-01 00:00:00 TAI",
    "bodies": [
        {
            "name": "Name",
            "mu": 100.0,
            "position": [1.0, -2.0, 2.0],
            "velocity": [3.0, -2.0, 1.0]
        }
    ]
}
```

- **Ship Spawner**: Allows to spawn ships in the system. Ships are defined by a name as well as a position and velocity within a reference frame. You can also import a ship and its flight plan from a file.

- **Prediction Debug**: Displays information about each trajectory.

## Technical overview of the ephemeris generation

Generating accurate ephemerides for a given system requires the configuration of the following parameters:

- An epoch at which is provided the state vectors of the bodies in the system, as well as the gravitational parameters of the bodies. This data is loaded with the `state.json` files in the provided systems.
- A time step or delta time `dt` and for each body the sample count `count` and the degree `deg` of the piecewise polynomial interpolation. This data is loaded with the `ephemeris.json` files in the provided systems. For now, the parameters need to be hand-tuned for each system to ensure the accuracy of the ephemerides. You can compute an estimate of the error from the interpolation in the `Ephemerides Debug` interface.

The initial state is used as the starting point for the generation of the ephemerides, whilst the other parameters define the accuracy of the ephemerides. More specifically, each polynomial of degree `deg` is interpolated using 9 samples from the numerical integration. The sample count `count` and the delta `dt` define the distance between each of these samples. For example, with `dt = 1 min`, the resulting trajectory of a body with `count = 2` will be made of polynomials that have been interpolated with 9 samples at `t = 0 min`, `t = 2 min`, ..., `t = 16 min`. As such, each polynomial will be valid for `8 * count * dt` and the delta between each sample will be `count * dt`.  
Currently, the interpolation is done using a least squares polynomial fit using the [poly_it](https://github.com/SkyeC0re/polyit-rs) crate.  

This implementation was custom made for this project and is not validated in any way. It is also better suited to stable trajectories and incorrect configuration of a system will result non-continuous polynomials within the trajectories.
