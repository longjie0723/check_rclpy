import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


class AddTwoIntsServer(Node):
    """AddTwoIntsを提供するサーバー"""

    def __init__(self):
        super().__init__('add_two_ints_server')

        self.srv = self.create_service(
            AddTwoInts,
            '/add_two_ints',
            self.add_two_ints_callback,
        )

    def add_two_ints_callback(self, request, response):
        self.get_logger().info('Request received: {} + {}'.format(request.a, request.b))
        response.sum = request.a + request.b
        return response


class ServiceFromTimer(Node):
    """Timerのコールバック関数の中からServiceを呼び出すサンプル"""
    def __init__(self):
        super().__init__('service_from_timer')

        # ReentrantCallbackGroupを使う必要がある
        self.callback_group = ReentrantCallbackGroup()

        # MutuallyExclusiveCallbackGroupにすると、timer_callbackのawaitでデッドロックする
        # self.callback_group = MutuallyExclusiveCallbackGroup()

        self.client = self.create_client(
            AddTwoInts,
            '/add_two_ints',
            callback_group=self.callback_group
        )
        # 1秒周期のタイマーコールバックを作成
        self.timer = self.create_timer(
            1.0, self.timer_callback, callback_group=self.callback_group)

        self.get_logger().info('Timer start')

    async def timer_callback(self):
        # Serviceが利用可能かを確認
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('No service server available')
            return response

        # 非同期にServiceを呼び出す
        request = AddTwoInts.Request()
        request.a = 1
        request.b = 2
        future = self.client.call_async(request)
        # futureの完了を待つ（この間にexecutorにより他のコールバックが割り込む）
        await future
        self.get_logger().info('Response received: {}'.format(future.result().sum))


def main(args=None):
   rclpy.init(args=args)

   add_two_ints_server = AddTwoIntsServer()
   service_from_timer = ServiceFromTimer()

   # 実行されるパスが一つしかないので、SingleThreadedExecutorでも正常に動く
   executor = SingleThreadedExecutor()
   # MultiThreadedExecutorでも正常に動く
   # executor = MultiThreadedExecutor()

   executor.add_node(add_two_ints_server)
   executor.add_node(service_from_timer)
   executor.spin()

   rclpy.shutdown()


if __name__ == '__main__':
    main()
