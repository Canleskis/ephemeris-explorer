# Ephemeris Explorer

Ephemeris Explorer aims to be a multi-purpose interactive experience of gravitationally bound systems, be it stellar systems, planetary systems or satellite systems.
It uses concepts from various research, notably [Numerical Representation of Planetary Ephemerides](https://adsabs.harvard.edu/full/1989CeMec..45..305N) to generate accurate[^1] trajectories of celestial bodies and presents them in an interactive manner using the [Bevy Engine](https://bevyengine.org/) and [egui](https://github.com/emilk/egui).

[^1]: The accuracy of the current implementation is limited by the fact that the acceleration computation does not take into account general relativity and all bodies are considered to be spheres with an homogeneous mass distribution.

## User Interface and Key Bindings

On top of showing the bodies in the system and their trajectories, the user interface allows for various ways to interact with the system:

- **Camera Controls**: The camera can be moved using the WASD keys, A and D for roll, as well as Space and Left Control to move up and down. The mouse can be used for pitch and yaw when the left mouse button is held down. You can also use orbit controls around the followed body by holding the right mouse button.

- **Hierarchy**: The left panel provides a hierarchical view of the celestial bodies within the current system. Clicking the 📌 allows setting the reference frame of the camera, which it will follow. You can toggle trajectory plotting using the ○ button, and toggle trajectory plotting of the children of a body using the A button. Clicking a body's name or a body in the viewport selects it and allows you to read information about the body and configure plotting.

- **Body information**: When a body is selected, information about the current state of the body is displayed. You can also configure the plotting of the body's trajectory. Changing the `Reference` of the body changes some of the displayed information and changes the reference frame of the body's trajectory.

- **Prediction Planner**: Allows to extend the ephemerides forward or backward in time independently and asynchronously by selecting a start epoch and an end epoch. You can pause and cancel ongoing predictions.

- **Time Controls**: Allows to change the speed of the simulation, pause it or set the epoch.

- **Export**: Allows to export the current state of the system to a file, selecting which bodies are included. The file can be loaded back into the system using the `Load` button. You can also export trajectories, but they are currently not importable.

- **Load**: Allows to load a system from a file. The file should be a JSON file containing the state of the system. See `export-solar-system` for a python program that allows the generation of such files using NASA's development ephemerides and [skyfield](https://rhodesmill.org/skyfield/). The file should be formatted as follows:

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

- **Prediction Debug**: Displays information about each trajectory.

## Technical overview of the ephemeris generation

Generating accurate ephemerides for a given system requires the configuration of the following parameters:

- An epoch at which is provided the state vectors of the bodies in the system, as well as the gravitational parameters of the bodies. This data is loaded with the `state.json` files in the provided systems.
- A time step or delta time `dt` and for each body the sample count `count` and the degree `deg` of the piecewise polynomial interpolation. This data is loaded with the `ephemeris.json` files in the provided systems. For now, the parameters need to be hand-tuned for each system to ensure the accuracy of the ephemerides. No tools are provided to help with this process yet, but hopefully this configuration won't be necessary in the future.  

The initial state is used as the starting point for the generation of the ephemerides, whilst the other parameters define the accuracy of the ephemerides. More specifically, each polynomial of degree `deg` is interpolated using 9 samples from the numerical integration. The sample count `count` and the delta `dt` define the distance between each of these samples. For example, with `dt = 1 min`, the resulting trajectory of a body with `count = 2` will be made of polynomials that have been interpolated with 9 samples at `t = 0 min`, `t = 2 min`, ..., `t = 16 min`. As such, each polynomial will be valid for `8 * count * dt` and the delta between each sample will be `count * dt`.  
Currently, the interpolation is done using a least squares polynomial fit using the [poly_it](https://github.com/SkyeC0re/polyit-rs) crate.  

This implementation was custom made for this project and is not validated in any way. It is also better suited to stable trajectories and incorrect configuration of a system will result non-continuous polynomials within the trajectories.
