## Overview

bondcpp is an implementation of [bond](https://index.ros.org/p/bond) in
C++. To use bondcpp, please see the example below as well as the
[API documentation](http://www.ros.org/doc/api/bondcpp/html/).

## Example usage

**Process A:** 

```cplusplus
#include <bondcpp/bond.h>

std::string id = generateUniqueId();
// Sends id to B using a service or action
bond::Bond bond("example_bond_topic", id);
bond.start();
if (!bond.waitUntilFormed(ros::Duration(1.0)))
{
    ROS_ERROR("ERROR!");
    return false;
}
// ... do things with B ...
bond.waitUntilBroken(ros::Duration(-1.0));
printf("B has broken the bond\n");
```



**Process B:** 

```cplusplus
#include <bondcpp/bond.h>

// Receives id from A using a service or action
bond::Bond bond("example_bond_topic", id);
bond.start();
// ... do things ...
bond.breakBond();
```



## Report a Bug
