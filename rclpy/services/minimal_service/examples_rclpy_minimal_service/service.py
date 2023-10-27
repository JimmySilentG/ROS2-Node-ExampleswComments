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

from example_interfaces.srv import AddTwoInts #from the example_interfaces package within the .srv module(a python file with specific code) import the AddTwoInts class

import rclpy #importing client python library, basically gives us access to top level python functions without worrying about communication between nodes

g_node = None #define the global variable g_node and assign it the value of none


def add_two_ints_callback(request, response): #defining a function, intended for callbacks, and passing it request and response parameters
    global g_node #refer to the global variable g_node in this function when used
    response.sum = request.a + request.b #assign properties a and b of the request class passed to a sum property of the response class passed
    g_node.get_logger().info(
        'Incoming request\na: %d b: %d' % (request.a, request.b)) #use the get logger method of the node class and info method of RcutilsLogger to log message with INFO severaity

    return response #function output will be the response class with the modified


def main(args=None):
    global g_node #refer to global variable g_node in this funciton, dont make a local variable
    rclpy.init(args=args) #Initialize client library with command line arguments

    g_node = rclpy.create_node('minimal_service') #creates a node called 'minimal service' and assigns it to the variable g_node

    srv = g_node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback) #method of the node class to create a new service server with a service type "AddTwoInts" and call the service 'add_two_ints'
                                                                                   #and assign the add_two_ints_callback as the callback function which executes when a service request is recieved by the server, assigned to srv variable
    while rclpy.ok(): #check if rclpy has been shutdown
        rclpy.spin_once(g_node) #execute 'one item of work' which i think means callback, or wait until a timeout expires. works in this case i guess because node only has a service to respond to

    # Destroy the service attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    g_node.destroy_service(srv)
    rclpy.shutdown() #shutdown rclpy client library


if __name__ == '__main__':
    main()
