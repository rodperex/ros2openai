import os
import rclpy
from ros2openai_interfaces.srv import OpenAIPrompt
from rclpy.node import Node


class OpenAIClient(Node):

    def __init__(self, service_name='openai_prompt'):
        super().__init__(service_name)
        super(OpenAIClient, self).__init__('openai_client')
        self.cli = self.create_client(OpenAIPrompt, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = OpenAIPrompt.Request()

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
        # api_key = os.environ[api_key_ev]
        api_key = "lm-studio"
        # model = "gpt-3.5-turbo"
        model="QuantFactory/Meta-Llama-3-8B-Instruct-GGUF"   
        service_name = 'openai_prompt'                  
        
        message = input('Please enter a prompt: ')
        
        if message == 'clear':
            service_name = 'openai_clear'
        
        minimal_client = OpenAIClient(service_name)
        minimal_client.get_logger().info(
            'OPEN-AI API key: %s' % api_key)
        
        response = minimal_client.send_request(model, api_key, message)
        
        minimal_client.get_logger().info(
            'Model response: %s' % response.message)

        minimal_client.destroy_node()
        rclpy.shutdown()
    else:
        print(f'The environmental variable {api_key_ev} is not set.')

    
if __name__ == '__main__':
    main()

    