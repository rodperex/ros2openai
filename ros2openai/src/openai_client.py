import os
import rclpy
from gptros2_interfaces.srv import GPTPrompt
from rclpy.node import Node


class GPTClient(Node):

    def __init__(self, service_name='gpt_prompt'):
        super().__init__(service_name)
        super(GPTClient, self).__init__('gpt_client')
        self.cli = self.create_client(GPTPrompt, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GPTPrompt.Request()

    def send_request(self, model, api_key, prompt):
        self.req.model = model
        self.req.api_key = api_key
        self.req.prompt = prompt
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):

    api_key_ev = 'OPENAI_API_KEY'

    if api_key_ev in os.environ:
        rclpy.init(args=args)
        api_key = os.environ[api_key_ev]
        model = "gpt-3.5-turbo"   
        service_name = 'gpt_prompt'                  
        
        message = input('Please enter a prompt: ')
        
        if message == 'clear':
            service_name = 'gpt_clear'
        
        minimal_client = GPTClient(service_name)
        minimal_client.get_logger().info(
            'OPEN-AI API key: %s' % api_key)
        
        response = minimal_client.send_request(model, api_key, message)
        
        minimal_client.get_logger().info(
            'GPT response: %s' % response.message)

        minimal_client.destroy_node()
        rclpy.shutdown()
    else:
        print(f'The environmental variable {api_key_ev} is not set.')

    
if __name__ == '__main__':
    main()