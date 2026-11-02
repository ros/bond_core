## Libraries

Bond is implemented in the following languages:

- C++ in [bondcpp](https://index.ros.org/p/bondcpp)
- Python in [bondpy](https://index.ros.org/p/bondpy)

## Motivation

Bond exists to ensure that two processes can monitor each other's
termination. This is useful in several situations.

**Spawning nodelets/controllers/robots in gazebo**

When spawning a nodelet (or anything else), two processes, the spawner
and the container, communicate to bring the nodelet up, however, the
current system does not cleanly deal with all termination possibilities
(). Creating a bond between the spawner and the container allows each to
know when the other crashes and to implement appropriate recovery
behaviors.

This applies to controllers (re-loading when the realtime_loop goes
down) and to gazebo (re-creating the robot if gazebo crashes).

**Resource ownership**

Bonds can be used to track resource ownership. The creation of a bond
indicates a lease on the resource, and breaking a bond indicates that
the lease is broken.

You could, for example, request that the laser scan at a particular
frequency. By passing a bond id along with the request, the laser
scanner controller will form a bond based on the request. When it
receives a new request, it breaks the bond. Whether the laser controller
receives a new request or the controller crashes, you will be informed
that the laser is no longer scanning as you desired.
