#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

class ParamResetClient(Node):

    def __init__(self):
        super().__init__('set_param_service_client')

        self._amcl_param_client = self.create_client(
            SetParameters, 'amcl/set_parameters'
        )
        
        while not self._amcl_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to start...')
        
        self.req = SetParameters.Request()

    def send_request(self):

        param = Parameter()
        param.name = "max_particles"
        param.value.type = ParameterType.PARAMETER_INTEGER
        param.value.integer_value = 200
        self.req.parameters.append(param)

        param = Parameter()
        param.name = "min_particles"
        param.value.type = ParameterType.PARAMETER_INTEGER
        param.value.integer_value = 100
        self.req.parameters.append(param)

        param = Parameter()
        param.name = "alpha1"
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = 0.85
        self.req.parameters.append(param)

        future = self._amcl_param_client.call_async(self.req)

        return future

def main(args=None):
    rclpy.init(args=args)

    amcl_param_reset_client = ParamResetClient()
    future = amcl_param_reset_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(amcl_param_reset_client)

        if future.done():
            try:
                response = future.result()
            except Exception as e:
                amcl_param_reset_client.get_logger().info(
                    'Service call failed %r' % (e,)
                )
            else:
                amcl_param_reset_client.get_logger().info(
                    'Result of set parameters: for %s' % (str(response))
                )
            break

    amcl_param_reset_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()