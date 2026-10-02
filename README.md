# IVR-Localization

Monte Carlo Localization (MCL) for a VEX V5 robot. Four distance sensors are compared against a
model of the field to correct LemLib odometry drift during a match.

Built with PROS (kernel 4.2.2) and LemLib 0.5.6.

## How it works

Each MCL cycle (every 20 ms, in its own task):

1. **Read sensors** – the four `pros::Distance` sensors (front, right, back, left) are read and
   converted to inches. Out-of-range readings become `-1`.
2. **Update** – the odometry change since the last cycle is taken from LemLib, converted into the
   robot frame (forward / right), and applied to every particle using that particle's own heading.
3. **Weigh** – for each particle, `fieldRepresentation::simulateCast` predicts what the four sensors
   should read from that pose. Particles whose predictions match the real readings get higher weight.
   The weighted mean of the particles is the pose estimate.
4. **Correct** – the estimate is applied to LemLib as a correction relative to the pose sampled at the
   start of the cycle, so motion LemLib tracked during weighing isn't lost.
5. **Resample** – only when the effective particle count drops below half the particle count.

Each particle stores its heading as an offset from the IMU (`heading = imu rotation + headingOffset`),
so the filter can also correct IMU drift.

## Field model

The field is a set of axis-aligned boxes in inches. The origin is at field centre on top of the
foam tiles, +X points towards the blue alliance station and +Z is up. Headings are in degrees,
0° = +Y, clockwise positive, which matches LemLib and the IMU.

- **Walls** – one box spanning the inner faces of the perimeter, marked `interior` so a ray from
  inside the field hits where it exits.
- **Loaders and goals** – one box each.

Ray casting uses the slab method (ray vs. axis-aligned box), with:

- a per-sensor height filter built once in the constructor, since sensor heights don't change
- early exit when a box can't beat the closest hit found so far
- a guard for rays parallel to a box face
- the V5 distance sensor's 2000 mm (78.74") maximum range

The box coordinates come from the official field CAD. See [`tools/field/README.md`](tools/field/README.md)
for the scripts that extract them (`Resources/field-model/field_boxes.json`) and render the model.

## Layout

| File | Contents |
|---|---|
| `include/robot.h`, `src/robot.cpp` | Motors, IMU, tracking wheels, distance sensors, LemLib chassis |
| `include/fieldRepresentation.h`, `src/fieldRepresentation.cpp` | Field boxes, sensor layout, ray casting |
| `include/mcl.h`, `src/mcl.cpp` | MCL loop: read sensors, update, weigh, resample |
| `include/particle.h` | Particle pose (x, y, heading offset) and weight |
| `include/utilities.hpp` | `vector3`, `distanceSensor`, angle and unit conversions |
| `tools/field/` | Python scripts to build the field model from the CAD |

## Building

With the PROS CLI installed:

```
pros make
pros upload
```

## Setup

- Set the distance sensor ports in `src/robot.cpp` (currently placeholders) and each sensor's mounting
  angle and offset from the robot's centre in `fieldRepresentation::distanceSensors()`.
- Mount the sensors level and more than about 3.5" above the tiles so the beam doesn't hit the floor.
- Call `chassis.calibrate()` and set the starting pose before `MCL::init`.

## Status

Work in progress. Ray casting and the MCL loop are in place. The particle motion update
(`MCL::update`), weighting (`MCL::weigh`), resampling (`MCL::resample`) and initial particle spread
(`MCL::init`) are still TODO.
