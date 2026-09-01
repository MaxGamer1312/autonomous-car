# autonomous-car

A self-driving car that learns to navigate a scale road network through reinforcement learning in Unity, then drives a real vehicle on a physical replica of the same map. Built through UT Austin's ECLAIR club.

The policy is trained entirely in simulation with Unity ML-Agents and transfers to hardware without retraining, because the real car and the simulated one consume identical observations.

## Positioning without GPS

At this scale, GPS is useless, its error is larger than the entire map. Commercial indoor positioning systems were out of budget. The solution was a camera mounted a few feet above the center of the track, looking straight down, with AprilTags in each corner of the map and one on the roof of the car. The corner tags establish the map frame; the car's tag gives its position and heading within that frame, and the tracking is accurate enough to drive on.

This is why the car carries no camera of its own. It doesn't perceive anything. All sensing happens overhead, all computation happens on a laptop, and the Raspberry Pi on board exists only to receive commands and turn them into motor output.

## Navigation

Each road piece in the map carries a node, a marked point placed by hand, snapped together the way LEGO bricks connect. Wiring those nodes into a graph turns the road network into something searchable, and a breadth-first search returns the shortest valid path between any two nodes. That path is the car's route, and it stands in for the GPS the vehicle can't use.

The agent is never given a destination in world coordinates. It's given the next few waypoints along the BFS path, expressed relative to itself, and left to work out the driving.

## The learned policy

**Observations.** Everything the agent sees is relative and normalized, which is what lets one policy work anywhere on the map:

- Angle and distance to each of the next three path nodes
- Its own forward speed and current steering angle
- For the three nearest vehicles: distance, bearing, speed, and whether that vehicle is moving toward it or away from it

Empty slots are padded with unambiguous "nothing here" values, a missing car reads as maximum distance, stopped, traveling the same direction, so the network never confuses absence for a threat.

**Actions.** Two continuous outputs, throttle and steering. When hardware is attached, these are scaled to motor units and sent over TCP to the Pi.

**Reward.** Shaped rather than sparse, since a pure goal reward gives almost no gradient on a road network:

| Signal | Effect |
|---|---|
| Closing distance to the next node | Small continuous reward, paid on progress only |
| Touching the correct next node | Bonus, and the path advances |
| Touching a wrong lane node | Equal penalty |
| Reaching the destination | Full goal reward |
| Hitting a wall or another car | Full penalty, episode ends, respawn |
| Every timestep | Small penalty, scaled so a full episode costs about one death |
| Sitting still | Small penalty, escalating to early termination after 100 stalled steps |

**Continuous driving.** Reaching a destination normally ends the episode and resets the car. Here an explicit flag suppresses the reset, the car keeps its position, picks a new destination, and drives on. Episode boundaries are invisible from the outside, which is what makes the physical car look like it's simply driving rather than running trials.

## Emergent behavior

Two results came out of training that were never explicitly designed:

**Three-point turns.** The agent was given no turning maneuver, no reverse primitive, and no curriculum for tight spaces. Faced with a destination behind it on a road too narrow to turn around in, it learned to reverse, reposition, and continue, the maneuver falls out of throttle and steering control under a progress reward.

**Multi-car avoidance.** With several agents sharing one map, cars learned to avoid each other most of the time using only the relative distance, bearing, and closing-direction observations. This was validated in simulation only; the team had one physical vehicle.

## Repository layout

| Path | Contents |
|---|---|
| `Assets/` | Unity scenes, prefabs, and all C#, agent, controller, pathfinding, Pi communication |
| `ThreePointTurn/` | Turning maneuver scene and assets |
| `carray-headless/` | Headless Unity build the ML-Agents trainer runs against |
| `Config/` | Training configuration |
| `Demos/` | Recorded runs |
| `saveData/` | Captured run data |
| `notebook.ipynb` | Analysis |

## Built with

Unity · C# · Unity ML-Agents · Python · Raspberry Pi · AprilTags
