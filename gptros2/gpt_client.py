import sys
import rclpy
from gptros2_interfaces.srv import GPTPrompt
from rclpy.node import Node


class GPTClient(Node):

    def __init__(self):
        super().__init__('gpt_client')
        self.cli = self.create_client(GPTPrompt, 'gpt_prompt')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GPTPrompt.Request()

    def send_request(self, model, api_key, system_role, prompt):
        self.req.model = model
        self.req.api_key = api_key
        self.req.system_role = system_role
        self.req.prompt = prompt
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):

    api_key = 'your-api-key'
    model = "gpt-3.5-turbo"
    system_role = "You are a great mathematician"
    message = "What is your favourite prime number?"
    rclpy.init(args=args)

    minimal_client = GPTClient()
    response = minimal_client.send_request(model, api_key, system_role, message)
    minimal_client.get_logger().info(
        'GPT response: %s' % response.message)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()