import websocket 
from threading import Thread
import time
import sys
from os import getenv
from json import dumps, loads
from time import sleep
from pprint import pprint
from logging import exception, warning, info, basicConfig, INFO

basicConfig(level=INFO)


# Install requirements: pip install websocket-client (no longer needed as included locally, still install 'six')
class WSClient(Thread):

  def __init__(self, cb=None):
    super(WSClient, self).__init__()
    self.websocket_url = getenv('WEBSOCKET_URL', 'ws://172.19.0.2:8128/')
    self.ws = None
    self.daemon = True
    self.callback = cb
    websocket.enableTrace(False)
    self.connected = False

  def on_message(self, message):
    data = loads(message)
    if self.callback is not None:
      self.callback(data)
    pprint(data)

  def on_error(self, error):
    self.connected = False
    print(error)

  def on_close(self):
    self.connected = False
    info("### closed ###")

  def on_open(self):
    self.connected = True
    info("### open ###")
    # register as admin to call-a-robot
    self.ws.send(dumps({
      'method': 'register',
      'admin': True,
      'user': 'admin'
    }))
    self.ws.send(dumps({
      'method': 'get_states'
    }))

  def set_closest_node(self, user, node):
    """ Websockert callback from ROS """
    if self.ws:
      self.ws.send(dumps({
        'method': 'set_closest_node',
        'node': node,
        'user': user
      }))

  def set_state(self, user, state):
    if self.ws:
      self.ws.send(dumps({
        'method': 'set_state',
        'state': state,
        'user': user
      }))

  def run(self):
    while True:
      sleep(1)
      if not self.connected:
        info("attempt connection to %s" % self.websocket_url)
        self.ws = websocket.WebSocketApp(self.websocket_url,
                                         on_message=self.on_message,  # This publishes to coordinator
                                         on_error=self.on_error,
                                         on_close=self.on_close,
                                         on_open=self.on_open)
        # self.ws.on_open = self.on_open
        self.ws.run_forever()

  def spin(self):
    if not self.is_alive():
      self.start()
    while True:
      self.join(600)
      if not self.is_alive():
        break


if __name__ == "__main__":

  def callback(data):
    pprint(data)

  try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from diagnostic_msgs.msg import KeyValue

    class CARClient(Node):

      def __init__(self):
        super().__init__("car_client")
        self.logger = self.get_logger()

        self.pub_states = self.create_publisher(String, "~/get_states", 100)
        self.pub_states_kv = self.create_publisher(KeyValue, "~/get_states_kv", 100)
        self.pub_gps = self.create_publisher(String, "~/get_gps", 100)

        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.VOLATILE)
        self.sub_states = self.create_subscription(String, "~/set_states", self.set_state, qos_profile)
        self.sub_states_kv = self.create_subscription(KeyValue, "~/set_states_kv", self.set_state_kv, qos_profile)
        self.closest_node = self.create_subscription(String, "~/set_closest_node", self.set_closest_node, qos_profile)
        self.closest_node_kv = self.create_subscription(KeyValue, "~/set_closest_node_kv", self.set_closest_node_kv, qos_profile)

        info_topic_dict = dict()
        topics = [t for t in self.get_topic_names_and_types() if t[0].startswith('/car_client/info/')]
        for t in topics:
          if t[0] not in info_topic_dict:
            print("initialising topic for {0}".format(str(t)))
            info_topic_dict[t[0]] = self.create_subscription(String, t[0], self.save_info, callback_args=t[0])

        self.ws_client = WSClient(self.publish)
        self.ws_client.start()

      def publish(self, data):
        print("\n\n")
        self.logger.info('received data from web socket')

        if 'method' not in data:
          self.logger.warn('unsure what to do with data: {0}'.format(data))
          return

        if data['method'] == 'update_position':
          self.logger.info('publishing gps')
          self.pub_gps.publish(String(data=dumps(data)))

        elif data['method'] == 'update_orders':
          self.logger.info('publishing update to orders')
          for k, v in data['states'].items():
            self.pub_states_kv.publish(KeyValue(key=k, value=v))
          self.pub_states.publish(String(data=dumps(data)))

        elif data['method'] == 'new_user':
          self.logger.info('publishing new user id')
          self.logger.info('/{0}/new_agent'.format(data['ri_ref']))
          pub = self.create_publisher(String, '/{0}/new_agent'.format(data['ri_ref']), 100)
          pub.publish(String(data=data['user']))

      def set_state(self, msg):
        payload = loads(msg.data)
        self.ws_client.set_state(user=payload['user'], state=payload['state'])

      def set_state_kv(self, msg):
        self.ws_client.set_state(user=msg.key, state=msg.value)

      def set_closest_node(self, msg):
        payload = loads(msg.data)
        self.ws_client.set_closest_node(user=payload['user'], node=payload['node'])

      def set_closest_node_kv(self, msg):
        self.ws_client.set_closest_node(user=msg.key, node=msg.value)

      def save_info(self, msg, topic):
        print("\n\n\nreceived {0} {1}".format(topic, msg.data))
        self.ws_client.set_state(user=topic, state=msg.data)


    rclpy.init()
    car_client = CARClient()
    rclpy.spin(car_client)
    car_client.destroy_node()
    rclpy.shutdown()

  except Exception as e:
    print("ROS exception {0}".format(str(e)))
    exception('no ROS, running fallback {0}'.format(str(e)))
    client = WSClient(callback)
    client.start()
    sleep(5)
    # after 5 seconds test state setting
    client.set_state('marc', 'INIT')
    client.spin()

