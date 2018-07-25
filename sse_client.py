from json import loads, dumps
from pprint import pprint
from logging import exception, warning, info
import sseclient
from threading import Thread
from time import sleep
import urllib3
from os import getenv

class SSEClient(Thread):

    def __init__(self, url='http://0.0.0.0:8127/car/events', callback=None, reconnect_timeout=1):
        super(SSEClient, self).__init__()
        # make it a daemon thread: https://docs.python.org/2/library/threading.html#threading.Thread.daemon
        self.daemon = True
        self.reconnect_timeout = reconnect_timeout
        self.url = url
        self.connected = False
        self.callback = callback
        self.initial_response = None
        self.client = None
        self.connect()

    def connect(self):
        try: 
            http = urllib3.PoolManager()
            self.initial_response = http.request('GET', self.url, preload_content=False)
            self.client = sseclient.SSEClient(self.initial_response)
            self.connected = True
        except Exception as e:
            self.connected = False
            warning('failed to connect: %s, %s' % (type(e), str(e)))

    def run(self):
        while self.isAlive():
            if not self.connected:
                self.connect()
                if not self.connected:
                    sleep(self.reconnect_timeout)
            else:
                try:
                    for event in self.client.events():
                        d = loads(event.data)
                        if self.callback is not None:
                            self.callback(d)
                except Exception as e:
                    self.connected = False
                    print '****', e
                    exception('exception while processing events: %s' % str(e))

    def spin(self):
        self.start()
        while True:
            self.join(600)
            if not self.isAlive():
                break


if __name__ == "__main__":
    websocket_url = getenv('WEBSOCKET_URL', 'http://0.0.0.0:8127/car/events')

    def cb(data):
        pprint(data)

    def main():
        client = SSEClient(url=websocket_url, callback=cb)
        client.spin()
        # test this for 4 seconds only
        #sleep(4)

    def ros_main():
        try:
            import rospy
            from std_msgs.msg import String
            rospy.init_node('sse_client')
            pub_states = rospy.Publisher('/sse_client/states', String, queue_size=100)
            pub_gps = rospy.Publisher('/sse_client/gps', String, queue_size=100)

            def ros_publish(data):
                if data['event'] == 'gps':
                    pub_gps.publish(dumps(data))
                else:
                    pub_states.publish(dumps(data))
            sse_client = SSEClient(url=websocket_url, callback=ros_publish)
            sse_client.start()
            rospy.spin()
        except:
            exception('no ROS, running fallback')
            main()


    ros_main()
