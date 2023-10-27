# Copyright 2018 Open Source Robotics Foundation, Inc.
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
from rclpy.callback_groups import ReentrantCallbackGroup #import reentrant callback group that allows parallel execution of callbacks, in python this is not
                                                         #true parallelism but instead allows it to call a callback inside a callback


def main(args=None):
    rclpy.init(args=args) #initializes client library with commandline arguments
    node = rclpy.create_node('minimal_client') #creates a node called minimal client and assigns it to the variable node
    # Node's default callback group is mutually exclusive. This would prevent the client response
    # from being processed until the timer callback finished, but the timer callback in this
    # example is waiting for the client response
    #IN MY OWN WORDS, there are different types of callback groups that affect the behavior of 
    #callbacks. when creating a node the default callback group is mutually exclusive which means
    #that only one callback can be executed to completion at a time. in this code that means that the line
    # result = await future will not execute because it is a waitable instance but the create_timer
    #callback was called to get to that point so the timer callback would need to finish first which
    #would result in a timeout whereas by changing the grouping you can process the await within
    #the call_service callback. otherwise you get stuck spinning a node that wont be recieving or
    #sending anything
    cb_group = ReentrantCallbackGroup() #creates an instance of the reentrantcallbackgroup class and assigns it to variable cb_group
    cli = node.create_client(AddTwoInts, 'add_two_ints', callback_group=cb_group) #creates a service client called add_two_ints with the service type AddTwoInts with a unique reentrant callback group defined above
    did_run = False #did_run variable set to False
    did_get_result = False #did_get_result set t0 False

    async def call_service(): #defines an asyncronous corutine in python called call_service()
        nonlocal cli, node, did_run, did_get_result #for funcitons inside functions, allows you to access variables of outer function without explicitly passing them
        did_run = True #set did_run to True
        try: #try to run this code, if an expection occurs cancel
            req = AddTwoInts.Request() #get the request component of the AddTwoInts service and assign them to req for assignment right after
            req.a = 41
            req.b = 1
            future = cli.call_async(req) #make a service call with the request class of the imported interface and asyncronously get the result as a future which is a class which represents the outcome of a task in the future
                                         #future class has a variety of attributes that can be assigned true or false based on if the service was canceled, completed, etc.
            result = await future #wait for the corutine that is the asyncrounous call to finish before assigning future to the variable result 
            node.get_logger().info(
                'Result of add_two_ints: for %d + %d = %d' %
                (req.a, req.b, result.sum)) #notice result.sum is an attribute based on AddTwoInts definition
        finally: #always run this no matter the result of the try, expect statements
            did_get_result = True

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    timer = node.create_timer(0.5, call_service, callback_group=cb_group)

    while rclpy.ok() and not did_run: #runs when did_run is false and rclpy is running
        rclpy.spin_once(node)

    if did_run:
        # call timer callback only once
        timer.cancel()

    while rclpy.ok() and not did_get_result:
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
