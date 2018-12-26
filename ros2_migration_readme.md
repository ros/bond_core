This document contains the overview of the bond-core package, test procedure to test package and the changes made with respect to migration of bond-core from ROS to ROS2. Later current limitations are also listed down.

Overview:
A bond allows two processes, A and B, to know when the other has terminated, either cleanly or by crashing. The bond remains connected until it is either broken explicitly or until a heartbeat times out.
Constructor of object is as follows,
bond::Bond::Bond(
  const std::string & topic, const std::string & id, rclcpp::Node::SharedPtr nh,
  std::function<void(void)> on_broken,
  std::function<void(void)> on_formed)

Parameters
    topic           :	The topic used to exchange the bond status messages.
    id	           :	The ID of the bond, which should match the ID used on the sister's end.
    nh               :	The node used to from bond
    on_broken  :	callback that will be called when the bond is broken.
    on_formed  :	callback that will be called when the bond is formed. 

Bond-core (CPP) is have following functionalities as per ROS.

 start() : Starts the bond and connects to the sister process.
 setFormedCallback(std::function<void(void)> on_formed) : Sets the formed callback.
 setBrokenCallback(std::function<void(void)> on_broken) : Sets the broken callback.
 waitUntilFormed(rclcpp::Duration timeout) : Blocks until the bond is formed for at most 'duration', timeout Maximum duration to wait.
 waitUntilBroken(rclcpp::Duration timeout) : Blocks until the bond is broken for at most 'duration', timeout Maximum duration to wait.
 isBroken() : Indicates if the bond is broken.
 breakBond() : Breaks the bond, notifying the other process.
 getTopic() : Gets topic name.
 getId() : Gets ID.
 getInstanceId() : Gets instant ID.
 getConnectTimeout() : Gets connect timeout in nanoseconds.
 setConnectTimeout(double dur) : Sets connect timeout in seconds.
 getDisconnectTimeout(): Gets disconnect timeout in nanoseconds.
 setDisconnectTimeout(double dur) : Sets disconnect timeout in seconds.
 getHeartbeatTimeout() : Gets heatrbeat timeout in nanoseconds.
 setHeartbeatTimeout(double dur) : Sets heartbeat timeout in seconds.
 getHeartbeatPeriod() : Gets heartbeat period (publishing period) in naoseconds.
 setHeartbeatPeriod(double dur) : Sets heartbeat period (publishing period) in seconds.
 getDeadPublishPeriod() : Gets dead publish period in nanoseconds.
 setDeadPublishPeriod(double dur) : Sets dead publish period in seconds.

Test procedure :

1. Clone and build the package
mkdir ros2_ws_bond
cd ros2_ws_bond
mkdir src
clone <url of package> e.g. https://github.com/kishornaik10/bond_core.git (checkout for ros2-devel branch)
cd ..
source /opt/ros/crystal/setup.sh
colcon build

2. Do the tests
colcon test

3. The executable are generated follow the below steps to run tests.

3.1 source local setup 
source install/local_setup.sh

3.2 run the executables.
./install/test_bond/lib/test_bond/exercise_bond_cpp_exc

./install/test_bond/lib/test_bond/test_callbacks_cpp_exc


ROS2 Migration changes:
The basic concept and design is same as ROS.
Most of changes for migration has been done as per Migration guide.
1. Node is passed to bond constructor.
2. The timeout class is removed, earlier steady timer was available and was used to create connect timer, disconnect timer, heartbeat timer and also publishing timer. All timeout i.e. connect timeout , disconnect timeout, heartbeat timeout etc. were set using APIs of steardy timer provided by roscpp.
Now, in ROS2, steady timer is not available by this time so, used walltimer of node. Walltimer is used in such way that whatever functionality was carried by steady timer is achieved.
3.  a) For each timer, Timeout time is set. b) Callback function is written in such a way that it will be called on valid request, so flag is used. c) Timer is created and canceled whenever required.
4. waitUntilFormed  and  waitUntilBroken functions modified, locking system removed to handle callback of node as Callback Queue is not available in ROS2.

Limitations:
1. Work around done for timer.
2. CallbackQueue is not available hence function related to Callback Queue commented out.
3. Dead publising period and timer is not used.
4. bondpy needs to be migrate.

