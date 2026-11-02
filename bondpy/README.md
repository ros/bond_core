## Overview

bondpy is an implementation of [bond](https://index.ros.org/p/bond) in
Python. To use bondpy, please see the example below as well as the
[API documentation](http://www.ros.org/doc/api/bondpy/html/).

## Example usage

**Process A:** 

```python
from bondpy import bondpy

id = generate_unique_id()
# Sends id to B using an action or a service
bond = bondpy.Bond("example_bond_topic", id)
bond.start()
if not bond.wait_until_formed(rospy.Duration(1.0)):
    raise Exception('Bond could not be formed')
# ... do things with B ... 
bond.wait_until_broken()
print "B has broken the bond"
```



**Process B:** 

```python
from bondpy import bondpy

# Receives id from A using a service or an action
bond = bondpy.Bond("example_bond_topic", id)
bond.start()
# ... do things ...
bond.break_bond()
```



## Report a Bug
