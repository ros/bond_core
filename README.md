# bond_core

[![Build Status](https://travis-ci.org/ros/bond_core.svg)](https://travis-ci.org/ros/bond_core)

A bond allows two processes, A and B, to know when the other has terminated, either cleanly or by crashing. The bond remains
connected until it is either broken explicitly or until a heartbeat times out.
