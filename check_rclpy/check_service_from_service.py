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


class ServiceFromService(Node):
    """Serviceの中から他のServiceを呼び出すサンプル"""

    def __init__(self):
        super().__init__('service_from_service')

        # ReentrantCallbackGroupを使う必要がある
        self.callback_group = ReentrantCallbackGroup()

        # MutuallyExclusiveCallbackGroupにすると、add_two_ints_proxy_callbackのawaitでデッドロックする
        # self.callback_group = MutuallyExclusiveCallbackGroup()

        self.client = self.create_client(
            AddTwoInts,
            '/add_two_ints',
            callback_group=self.callback_group
        )

        self.server = self.create_service(
            AddTwoInts,
            'add_two_ints_proxy',
            self.add_two_ints_proxy_callback
        )
        self.get_logger().info('Service is ready')

    # Serviceのコールバック関数
    def add_two_ints_proxy_callback(self, request, response):
        # Serviceが利用可能かを確認
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('No service server available')
            return response
        # 非同期にServiceを呼び出す
        future = self.client.call_async(request)
        # クライアントに結果を返すためにfutureの完了を待つ（この間にexecutorにより他のコールバックが割り込む）
        self.executor.spin_until_future_complete(future)
        # 結果を返す
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    add_two_ints_server = AddTwoIntsServer()
    service_from_service = ServiceFromService()

    # 実行されるパスが一つしかないので、SingleThreadedExecutorでも正常に動く
    #executor = SingleThreadedExecutor()
    # MultiThreadedExecutorでも正常に動く
    executor = MultiThreadedExecutor()

    executor.add_node(add_two_ints_server)
    executor.add_node(service_from_service)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
