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
        self.websocket_url = getenv('WEBSOCKET_URL', 'ws://localhost:8128/')
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
        if not self.isAlive():
            self.start()
        while True:
            self.join(600)
            if not self.isAlive():
                break


if __name__ == "__main__":

    def callback(data):
        pprint(data)

    try:
        import rospy
        from std_msgs.msg import String
        from diagnostic_msgs.msg import KeyValue

        rospy.init_node('car_client')
        pub_states = rospy.Publisher('~get_states', String, queue_size=100, latch=True)
        pub_states_kv = rospy.Publisher('~get_states_kv', KeyValue, queue_size=100, latch=True)
        pub_gps = rospy.Publisher('~get_gps', String, queue_size=100, latch=True)

        def ros_publish(data):
            print("\n\n")
            rospy.loginfo('received data from web socket')

            if 'method' not in data:
                rospy.logwarn('unsure what to do with data: %s'%data)
                return

            if data['method'] == 'update_position':
                rospy.loginfo('publishing gps')
                pub_gps.publish(dumps(data))

            elif data['method'] == 'update_orders':
                rospy.loginfo('publishing update to orders')
                for k, v in data['states'].items():
                    pub_states_kv.publish(KeyValue(key=k, value=v))
                pub_states.publish(dumps(data))

            elif data['method'] == 'new_user':
                rospy.loginfo('publishing new user id')
                pub = rospy.Publisher('/%s/new_agent'%data['ri_ref'], String, queue_size=100, latch=True)
                pub.publish(String(data['user']))


        ws_client = WSClient(ros_publish)

        def set_state(msg):
            payload = loads(msg.data)
            ws_client.set_state(user=payload['user'], state=payload['state'])
        def set_state_kv(msg):
            ws_client.set_state(user=msg.key, state=msg.value)
        rospy.Subscriber('~set_states', String, set_state)
        rospy.Subscriber('~set_states_kv', KeyValue, set_state_kv)


        def set_closest_node(msg):
            payload = loads(msg.data)
            ws_client.set_closest_node(user=payload['user'], node=payload['node'])
        def set_closest_node_kv(msg):
            ws_client.set_closest_node(user=msg.key, node=msg.value)
        rospy.Subscriber('~set_closest_node', String, set_closest_node)
        rospy.Subscriber('~set_closest_node_kv', KeyValue, set_closest_node_kv)

        ws_client.start()
        info_topic_dict = dict()

        def save_info(msg, topic):
            print("\n\n\nrecieved %s %s" % (topic, msg.data))
            ws_client.set_state(user=topic, state=msg.data)

        while not rospy.is_shutdown():
            rospy.sleep(2)
            topics = [t for t in rospy.get_published_topics() if t[0].startswith('/car_client/info/')]
            for t in topics:
                if t[0] not in info_topic_dict:
                    print('initialising topic for %s'%str(t))
                    info_topic_dict[t[0]] = rospy.Subscriber(name=t[0], data_class=String, callback=save_info, callback_args=t[0])


    except Exception as e:
        print("ROS exception %s" % str(e))
        exception('no ROS, running fallback %s' % str(e))
        client = WSClient(callback)
        client.start()
        sleep(5)
        # after 5 seconds test state setting
        client.set_state('marc', 'INIT')
        client.spin()

