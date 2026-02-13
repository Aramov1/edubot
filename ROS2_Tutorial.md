Ros = Robot Operating System
ROS1 - First version of Ros
ROS 2 - Second Version of ROS -> Recommended Version

to install any package, just run sud apt ros-<distro>-<package name>

To be able to run ROS2, you need to first source in every ternminal you want to use ROS (using source /opt/ros/..../setup.bash -> this is the script that needs to be sourced)


After that we can simply run ros2 commands. Add it to the .bachrc file at the end

# enable ROS2 Jazzy in every terminal
source /opt/ros/jazzy/setup.bash

gedit ~/.bashrc
when download ROS2, there are packages that you can use directly without having to create one
ROS2 Node = ROS2 Programme that is going to interact with ROS2 coomunication and tools

Nodes are organized in packages (navigation, camera driver= Each package can have multiple nodes)
Nodes can just comminicate a betweeen each other and log ion the terminal, create GUI, interact with the hardware, host web server, ...
to run a node run ros2 run <name of the package> <name of the node>
A node is simply a program that you are going to run (either py or cpp), and the specifidity is that it is connected to a ROS2 environment

# rqt_graph ->_ ROS tool to actually see the graph, that is a representation of all the nodes that are running

Nodes are also using tools like logging/debug (as rqt_graph)
nodes can provide GUI or and update based on other node

# Create/setup ROS2 Workspace
workspace is where code is orgbuoanized (using differente packages) and is also possible to build and install custom code in your workspace such that you can actually used

It is what is shared between teams
he file /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash to teh .bashrc file
The build tool for ROS2 is colcon, which is a command line tool. 
To allow auto complition add source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash to the end of ~/.bashrc file

workspace is nothing more nothing less than a folder, commonly called ros2_ws. Inside there is a source folder (should be named src), in which ros2 nodes are actually written. How it works? we are able to use ros global installations functions and custom. There will be an overlay with the global ros2 installation

inside the ws folder we can do colcon build (not in the src folder)!. Thisl will create 3 new folders -> Colcon will fetch code that is inside the src folder and is going to build and install nodes inside install folder (where we will have another setup.bach script which will have to be sourced after doing that it is posssible to use the notes that have been created! In irder not to run it every time, ath the source ... to the end og the .bashrc file (source ~/ros2_ws/install/setup.bash) (remember to source again the .bashrc in terminal if it is already oppened/or open a new one so that tha changes take effect)

# Create ROS2 Python Package
Before starting writting the code for any note, it is needed to create a package.

We will write nodes inside packages, which help organize the code in a better way as well as the dependencies between packages. To create a package (inside src folder in ros_ws) run "ros2 pkg create <package name withoit spaces_>". Notyes are written inside this folder.

Convention: when creating a package for a robot, first add robot name and after what you want to do (ricardo_controller) - Its a good practice to have. Each package is going a sub part of your application. Each package can have multiple nodes, which are specific to this specific package. Packages can have dependencies on other packages. Also specity if it is a python or a cpp package (by running next to create package name --build-type ament_(cmake/python))

Colcon -> Build tool
ament -> Built system

you can also add dependencies on other packages and functionalitie sthat are needed for a given package by adding --dependencies (rclpy -python lybrary for ros2)

packages created:
package.xml -> located in every package, which contain information about it (title, description, dependencies...). If we figure out we need more dependencies, we can just add a dpend line in th xml file

the setup.cfg is used to tell ros where are the nodes going to be installed

setup.py has a field (entry points, which tell where to intall the ROS2 nodes). The folder with the same name as the package that the ros 2 nodes are created

(after package is created, rebuild the ws uing colcon build (in the ros_ws folder) If there is an error change version of setuptools (in the video 58.2.0 is used).

buuild will fetch all the nodes that you need to install from the available packages and is going to install them in the install folder. From then it will be possible to use ros2 command line tools after setup.bash file is sourced

# ROS2 NODE 
write first node (get familiar to write, start and how to use command line tool to introspect the node)

create a new pythopun file: touch my_first_node.py
Make it executable: chmod +x my_first_node (visibel by green color on the terminal)

use code . to open vs code in the given location

Start by adding the interpreter line -> tell interpreter to use python 3

#!/usr/bin/env python3 # Indicates interpreter path! Allow to execute dirctly from terminal by running ./...
import rclpy # Pythion library for ros2 (if rclpy is not recognized (and to allow for auto complition install ROS extension ))
from rclpy.node import NODE

def main(args==None):
    rclpy.init(args = args) # start ros2 communication
    
    # code - Where nodes are created
    node = MyNode()
    rclpy.spin(node) # Serve to indicat e the node should keep spinning rather than be filled exactly after code runs. That is node is kept alive indefinitly until you kill it. While it is alive, all callbacks will be able to be run and communicating tith ros2 functionalities
    rclpy.shutdown() # end ros2 communicationm

if __name__ == '__main__': # If we want to execute the code directly from the terminal. Main is usefil when we want to install the node with ros2 functionalities
    main()

    Always include rclpy.init and rclpy.shutdown(). Every thing you will write will be confined between those two lines

the node is not the file itself but rather it will be created inside the programme. Nodes are created as object in Object Oriented Programming

Node class -> from programming purpuses
Node name in Init -> Name that will actually be  used when we actually run the node

class MyNode(Node): # Nodes inherit NODE class from ROS 2 standard packages

    def __init__(self): # Constructor
        super.__init__("my_node") # Use all lower letters in name 
        
        # From now note will inherit functionalituies from node class. Example:
        selg.get_logger().info("Hello from ROS2")

To be able to run the node with ros2 functionalitiex (example with ros2 run). For that we have to install the node. Go to setup.py inside package folder. in entry points, console scripts we write sth like

entry_points = 
{
    'console_scripts': {
        "node_name" = package_name.node_name:main   | withouts extension name
    }
}

everu time you build, have to source tyour workspace!! - 
colcon build
source ~/.bashrc
ros2 run package_name node name

NOTE DIFFERENCE BETWEEEN 3:_

Nodes name inside ros functionalities
filename where ypu are going to write your code
executable name which is the name that will be installed with colcon build and used to run using ros2 run in the command line tool

It is possible to use the same name for all of them!!, which can be used in the futuir

Every thing the python code is changes, you will need to rebuild the code again to b e able to sun the updated version of the node

colcon build --symlink-install -> If symlink-install is used it is possible to runrun the node without buildiong and source  kinda python is interpreted language, just start reading code again - Directly be able to run new version of teh script without having to install it every time

How to repeat message avery x seconds? Use ROS2 Timer () creazted inside the node and Ros2 callback (function called) - created inside the node!


ros2 node list -> List Running Nodes
ros2 node info /name_of_node -> gives information about the node

# ROS2 Topics

Basic of ROS2 communication. Allow nodes to communicate with each other thowrow nodes

Example: for the demo_nodes package, chatter is the toppic used to communicate between talker and listener

Talker node is going to publish on the chapter topic and listener node is going to subscribe to the chatter. The talker does not directly talk with the listener, but rather through a centralized topic center

to see all open topics, run ros2 topic list. To see information about the topic, run ros2 topic info ... When you publish/subcrive you have to respect the data type. the data type of a topic can be read using 

ros2 interface show <message type> -> string data -> data is the name of the mensage while it is of tipy string

what is being sent between talker and listener is the message called std_msgs/msg/string which contains data of tyope string

!!! USING Command line tools it is possible to know what is going on within your graph


ros2 topic echo /chatter -> allow to receive data 
we will see all mesages which are published to that topic. When using the echo we are subscribing to the toipic again

All puplisher publish to a given topic and all sbscribers will receive the same message!

another example, 
ros2 topic type /turtle/cmd_vel ==== geometry_msgs/msg/Twist

ros2 interface show geometry_msgs/msg/Twist

Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z

    eadch toipic has a name (kinda an address) such that nodes know where to publish/subscribe and datatype so that nodes know what to send and what to receive

    It is anonymous, a receiver will only receive a message, and no information about the publisher



# ROS2 python publisher

inside the package folder, create a file. eg: Remember to make it executable!
touch draw_circle.py
chmod +x draw_circle.py
then in the src folder run code . to start editing and create the node

for python files, remember to darr the interpreter line and base python package

GOOD PRACTICE: When creating a node, always add a logger saying that the node has been started. For that place sth li,e this inside the constructor
        self.get_logger().info("Draw Circle Node has been started")

If we want to make a node publish to a given topic, we can create an attribute for the node that will be the publisher attribute.

Good practice: publusher attribute name: topicName_pub

NOTE: TOPICS ARE CREATED when some node pushleses/subscribes to it. Otherwise it will not exist. When writting a publihser and want to see how to commun icate with the topic, run other node that is using that topic such that the topic becames available in ros2 topic list


in the example case, you can crate the publihse wioth the line 

self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

Note that Twist comes from the package from geometry_msgs.msg import Twist, and Twist the the message we are using for this application, geometry_msgs is the package name. geometry_msgs was installed with ros2. Do not forget to add dependency in packages.xml


Twist is the message type, the "/turtle1/cmd_vel" is the 
name of the topic and 10 is the queue size, that acts as a buffer to ensure all messages are sent

After creating the publisher, we then can use it to actually publish to the topic what we want (for example using a timer with a callback)

a message is created using msg = Twist(). Twins is the message type defined previously

To see what to send in the message and know how to poupulate use ros2 interface show geometry_msgs/msg/Twist

to publish the message use
self.cmd_vel_pub_.publish(msg)

NOTE --symlink install only works for nodes that were previously installed

Nodes. Do not forget args=args in rclpy.init(args=args), and the node in rclpy.spin(node)

NOTE: FOR THE COMMUNICATION TO WORK, ALWAYS USE THE NAME NAME AND TYPE

for debugguing the following commands are useful

ros2 topic list
ros2 topic info <topic name>
ros2 topic echo <msg>


# ROS2 Subscriber

Allows to receive data from a topic. Before start, we need to know what kind of data do we want to receive/what topic do we want to listen to

remember: ros2 topic ech "name of the tipic" allow tio listen what is being published to the topic
    ros2 topic info "message namhith these 3 commands, we get the name of the topic, what is inside the message 

Again good practice, name subscriver as topicName_sub_. a subscropion ios created using self.create_subscription

when creatiung a subscriber,  we need a callback, because when creating a subscriber we will receive a messages and you want to process the messages when youi receive them. The callback function will be called every time we receive a message

Rememeber Queue size acts like a buffere (for example for long messages or weeak network)

Good practice: call callbacks can be nanmeswhatyouwanttoreceive_callback 

REMEMBER rclpy.innit(args=args)!!

to print mesages in the terminal use str(msg)

NOTE: ros2 pkg list -> list packages available!

# ROS2 Service
A note sends a request (to a server), and the server is going to process stuff and reply to the client

this kind of of iteration is not what topics are made for (Topics just serve to send data drom one point to another, but without expecting an answer - that is the purpuse of ROS Services)

Eg:

ros2 run demo_nodes_cpp add_two_ints_server
(After run, the node is just waiting, but it is a node where we will have a service)

!!! With rqt_graph it is not possible to see services! - To debug services only command line features are available (eg: ros2 service list)

Just like topic, a service will also have a name and a type

each node will have a ser of services

it is possible to use ros2 interface show for servives

To call a service, a request has to be made:
ros2 service call /add_two_ints /example_interfaces/srv/AddTwoInts "{'a': 2, 'b':5}"

A services is simply a client server communication. You can only have one server, but can have multiple clients connecting to the server

Services are mainly used for two kinds of requests: 
    1) Computation
    2)Change of settings (Yiou may have lots of settings, create a service to update the settings and then other nodes can xcreate requests to change the settings)

NOTE: It is possible to call a service with no response/input 

# ROS2 Service Client

Service called inside a node:
Good practice: call_serviceName_service

to criate the client, make
ser.service_client = self.create_client(service_type, service name)

self.get_logger().warn() -> print in yellow color
client.wait_for_service -> ype.Request()
request.x = x

client.call(request) -> call and wait for response
future = client.call_async -> call assyncronously

futur is sth that will be done in the future, when you hget response from the service, future will call this callback and process what to do

NOTE: To not forget the callback for when the service replice with the response