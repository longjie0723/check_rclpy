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

class ServiceFromTask(Node):
    """Taskの中から他のServiceを呼び出すサンプル"""
    def __init__(self):
        super().__init__('service_from_task')

        # ReentrantCallbackGroupを使う
        self.callback_group = ReentrantCallbackGroup()

        # 実はこの場合はMutuallyExclusiveCallbackGroupでも正常動作する
        # self.callback_group = MutuallyExclusiveCallbackGroup()

        # NodeのRateタイマーオブジェクトを作成: 1000Hz
        self.rate = self.create_rate(1000)

        self.client = self.create_client(
            AddTwoInts,
            '/add_two_ints',
            callback_group=self.callback_group
        )

    def sleep_ms(self, msec : int):
        """ミリ秒単位でスリープするための関数"""
        for i in range(msec):
            self.rate.sleep()

    async def main_loop(self):
        """executorで実行されるメインループ"""
        while rclpy.ok():
            if not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('No service server available')
                continue

            # 非同期にServiceを呼び出す
            request = AddTwoInts.Request()
            request.a = 1
            request.b = 2
            future = self.client.call_async(request)

            # futureの完了を待つ（この間にexecutorにより他のコールバックが割り込む）
            await future
            self.get_logger().info('Response received: {}'.format(future.result().sum))
            
            # 1秒スリープ
            self.sleep_ms(1000)

            self.get_logger().info('wake up')

def main(args=None):
    rclpy.init(args=args)

    add_two_ints_server = AddTwoIntsServer()
    service_from_task = ServiceFromTask()

    # MultiThreadedExecutorを使う必要がある
    executor = MultiThreadedExecutor()

    # Rateタイマーと並列してコールバック処理をする必要があるので、SingleThreadedExecutorではmain_loopのrate.sleepでデッドロックする
    # executor = SingleThreadedExecutor()

    executor.add_node(node=add_two_ints_server)
    executor.add_node(node=service_from_task)
    executor.create_task(service_from_task.main_loop)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
