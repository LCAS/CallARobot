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
                                                 on_message=self.on_message,
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

    def ros_main():
        try:
            import rospy
            from std_msgs.msg import String
            from diagnostic_msgs.msg import KeyValue

            rospy.init_node('car_client')
            pub_states = rospy.Publisher('~get_states', String, queue_size=100, latch=True)
            pub_gps = rospy.Publisher('~get_gps', String, queue_size=100, latch=True)

            def ros_publish(data):
                rospy.loginfo('received data from web socket')
                if data['method'] == 'update_position':
                    pub_gps.publish(dumps(data))
                else:
                    pub_states.publish(dumps(data))
                print("\n\n")
            ws_client = WSClient(ros_publish)

            def set_state(msg):
                payload = loads(msg.data)
                ws_client.set_state(user=payload['user'], state=payload['state'])

            def set_state_kv(msg):
                ws_client.set_state(user=msg.key, state=msg.value)

            rospy.Subscriber('~set_states', String, set_state)
            rospy.Subscriber('~set_states_kv', KeyValue, set_state_kv)

            ws_client.start()
            rospy.spin()
        except Exception as e:
            print("ROS exception %s" % str(e))
            exception('no ROS, running fallback %s' % str(e))
            client = WSClient(callback)
            client.start()
            sleep(5)
            # after 5 seconds test state setting
            client.set_state('marc', 'INIT')
            client.spin()


    ros_main()
