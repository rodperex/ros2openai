import rclpy
import openai
from gptros2_interfaces.srv import GPTPrompt
from rclpy.node import Node


class GPTService(Node):

    def __init__(self):
        super().__init__('gpt_service')
        self.srv = self.create_service(GPTPrompt, 'gpt_prompt', self.gpt_prompt_callback)

    def gpt_prompt_callback(self, request, response):

        openai.api_key = request.api_key

        try:
            completion = openai.chat.completions.create(
                model=request.model,
                messages=[
                {"role": "system", "content": request.system_role},
                {"role": "user", "content": request.prompt}
                ]
            )
            response.role = completion.choices[0].message.role
            response.message = completion.choices[0].message.content
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
            response.message = 'Service call failed %r' % (e,)
            response.role = ''
            
        return response


def main(args=None):
    rclpy.init(args=args)

    gpt_service = GPTService()

    rclpy.spin(gpt_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()