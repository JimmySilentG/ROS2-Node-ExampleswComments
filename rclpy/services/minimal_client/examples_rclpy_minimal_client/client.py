# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from example_interfaces.srv import AddTwoInts #from the example_interfaces package within the in the .srv module(a python file with specific code) import the AddTwoInts class

import rclpy #importing client python library, basically gives us access to top level python functions without worrying about communication between nodes


def main(args=None):
    rclpy.init(args=args) #initalizes rclpy client library for python with command line arguments given
    node = rclpy.create_node('minimal_client') #creates a ROS2 node called 'minimal_client' and assigns it to the variable node
    cli = node.create_client(AddTwoInts, 'add_two_ints') #creates a new service client with the service type AddTwoInts that was imported earlier and names the ROS service 'add_two_ints' and assigns it to the variable cli

    req = AddTwoInts.Request() #references the defined fields in the request class of the service definition and assigns it to the variable req, NOT SURE IF THIS WORKS
    req.a = 41 #assigns the properties of the request class to integers. must match type in service definition
    req.b = 1
    while not cli.wait_for_service(timeout_sec=1.0): #method of the client class that waits for server to become ready, returns true if so or false if not before timeout
        node.get_logger().info('service not available, waiting again...') #use the get logger method of the node class to and info method of RcutilsLogger to log message with INFO severaity

    future = cli.call_async(req) #make a service call with the request class of the imported interface and asyncronously get the result as a future which is a class which represents the outcome of a task in the future
    #future class has a variety of attributes that can be assigned true or false based on if the service was canceled, completed, etc.
    rclpy.spin_until_future_complete(node, future) #callbacks and other work in the node will run until future attribute 'done' returns True. can specify a timeout

    result = future.result() #get the result attribute of the future, which is the response of the service node with attributes the same as the field name in the defined service type, and store it in the result variable
    node.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (req.a, req.b, result.sum)) #use the get logger method of the node class to and info method of RcutilsLogger to log message with INFO severaity
            #notice result.sum is an attribute based on AddTwoInts definiton
    node.destroy_node() #destroys the node
    rclpy.shutdown() #shuts down the rclpy in the default context, which I assume is that in the main function of this node


if __name__ == '__main__':
    main()
