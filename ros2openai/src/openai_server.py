import rclpy
from openai import OpenAI
import json
import os
from ros2openai_interfaces.srv import OpenAIPrompt
from rclpy.node import Node


class OpenAIService(Node):

    def __init__(self):
        super().__init__('openai_service')
        self.srv = self.create_service(OpenAIPrompt, 'openai_prompt', self.openai_prompt_callback)
        self.clear_srv = self.create_service(OpenAIPrompt, 'openai_clear', self.clear_history_callback)
        self.client = OpenAI(base_url="http://localhost:1234/v1")
        
    def save_historic(self, api_key, conversation_history):
        file_path = 'src/ros2openai/tmp/historic_' + api_key + '.json'
        self.get_logger().debug('Saving conversation history to %s' % file_path)
        with open(file_path, 'w') as file:
            json.dump(conversation_history, file, indent=4)
    
    def load_historic(self, api_key):
        file_path = 'src/ros2openai/tmp/historic_' + api_key + '.json'
        self.get_logger().debug('Loading conversation history from %s' % file_path)
        try:
            with open(file_path, 'r') as file:
                return json.load(file)
        except (FileNotFoundError, json.JSONDecodeError):
            return []

    def clear_history_callback(self, request, response):
        try:
            api_key = request.api_key
            file_path = 'src/ros2openai/tmp/historic_' + api_key + '.json'
            os.remove(file_path)  # Delete the conversation history file
            response.message = "History cleared successfully"
        except Exception as e:
            self.get_logger().info('Clear history failed %r' % (e,))
            response.message = 'Clear history failed %r' % (e,)
        return response
    
    def openai_prompt_callback(self, request, response):
        self.get_logger().info('Incoming request')
        # openai.api_key = request.api_key
        self.client.api_key = request.api_key
        self.get_logger().info('OPEN-AI API key: %s' % request.api_key)
        self.get_logger().info('Model: %s' % request.model)
        self.get_logger().info('Prompt: %s' % request.prompt)
        self.get_logger().info('Loading conversation history...')
        conversation_history = self.load_historic(request.api_key)
        self.get_logger().debug('%s' % conversation_history)
        

        try:
            
            conversation_history.append({"role": "user", "content": request.prompt})
            
            completion = self.client.chat.completions.create(
                model=request.model,
                messages=conversation_history
            )
            response.role = completion.choices[0].message.role
            response.message = completion.choices[0].message.content
            conversation_history.append({"role": "system", "content": response.message})
            self.get_logger().debug('Model response: %s' % response.message)
            self.get_logger().info('Saving conversation history...')
            self.save_historic(request.api_key, conversation_history)
            
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
            response.message = 'Service call failed %r' % (e,)
            response.role = ''
            
        return response


def main(args=None):
    rclpy.init(args=args)

    openai_service = OpenAIService()

    rclpy.spin(openai_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()