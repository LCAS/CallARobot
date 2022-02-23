#!/usr/bin/env python

import webnsock
import web
from signal import signal, SIGINT
from os import path
from logging import error, warn, info, debug, basicConfig, INFO, exception

from csv import DictWriter
from datetime import datetime
from time import mktime
from threading import Lock
from collections import defaultdict
from json import dumps
from threading import Condition
from uuid import uuid4
from os import getenv
from pprint import pprint

import requests

basicConfig(level=INFO)


class CARState:

    def __init__(self):
        self.states = {}
        self.clients = set([])
        self.admin_clients = set([])
        self.users = {}
        self.gps = defaultdict(dict)

        for u in self.users: self.set_state(u, 'INIT')
        self.log_filename = 'call-a-robot-'+datetime.now().strftime("%Y%m%d-%H%M%S") + '.csv'
        self.csvfile = open(self.log_filename, 'w', 0)
        self.log_fieldnames = [
            'id', 'timestamp', 'datetime', 'user', 'log_user', 'uid',
            'state', 'latitude', 'longitude', 'row']
        self.log_writer = DictWriter(self.csvfile, fieldnames=self.log_fieldnames)
        self.log_writer.writeheader()
        self.log_id = 0
        self.log_lock = Lock()

        self.log_uid = defaultdict(int)

    def log(self, user, state='NONE', log_user="", latitude=-1, longitude=-1, row=''):
        dt = datetime.now()
        ts = mktime(dt.timetuple())

        if 'latitude' in self.gps[user]:
            latitude = self.gps[user]['latitude']
        if 'longitude' in self.gps[user]:
            longitude = self.gps[user]['longitude']
        if 'row' in self.gps[user]:
            row = self.gps[user]['row']

        entry = {
            'id': self.log_id,
            'timestamp': int(ts),
            'datetime': dt.strftime("%Y%m%d-%H%M%S"),
            'user': user,
            'log_user': log_user,
            'uid': self.log_uid[user],
            'state': state,
            'latitude': latitude,
            'longitude': longitude,
            'row': row
        }

        with self.log_lock:
            self.log_writer.writerow(entry)
        if state is not 'GPS':
            for a in self.admin_clients:
                a.sendJSON({
                    'method': 'add_log',
                    'id': self.log_id,
                    'timestamp': int(ts),
                    'datetime': dt.strftime("%Y%m%d-%H%M%S"),
                    'user': user,
                    'log_user': log_user,
                    'uid': self.log_uid[user],
                    'state': state,
                    'latitude': latitude,
                    'longitude': longitude,
                    'row': row
                })
        self.log_id += 1
        if state == 'INIT' or state == 'INIT':
            self.log_uid[user] += 1

    def get_state(self, user):
        if user not in self.states:
            self.states[user] = 'INIT'
        return self.states[user]

    def set_state(self, user, state, log_user="", latitude=-1, longitude=-1, row=''):
        prev_state = self.states[user]
        self.states[user] = state
        if latitude == -1 and user in self.gps and 'latitude' in self.gps[user]:
            latitude = self.gps[user]['latitude']
        else:
            latitude = -1
        if longitude == -1 and user in self.gps and 'longitude' in self.gps[user]:
            longitude = self.gps[user]['longitude']
        else:
            longitude = -1
        # self.trigger_webhook()
        self.log(user, state, log_user, latitude, longitude, row)

    def send_updated_states(self, extra_socket=None):
        if extra_socket is None:
            addressees = set([])
        else:
            addressees = {extra_socket}
        addressees = addressees.union(self.clients)
        for m in addressees:
            info('send update to manager %s' % str(m))
            m.sendJSON({
                'method': 'update_orders',
                'states': self.states
            })

    def send_update_position(self, user, lat, long, acc, ts, row):
        for m in self.admin_clients:
            info('send pos update  %s' % str(m))
            m.sendJSON({
                'method': 'update_position',
                'user': user,
                'lat': lat,
                'long': long,
                'accu': acc,
                'timestamp': ts,
                'row': row
                # 'heading': heading,
                # 'velocity':velocity
            })


car_states = CARState()
server_details = dict()

class CARWebServer(webnsock.WebServer):

    def __init__(self):
        self.car_states = car_states
        self.is_running = True
        self.websocket_url = getenv('WEBSOCKET_URL', '')  # 'wss://lcas.lincoln.ac.uk/car/ws' a localhost websocket url is not accessible from other devices
        self.gmaps_api = getenv('GMAPS_API', 'XXX')
        self.rows_str = getenv('CAR_ROWS', 'A1 A2 A3 A4 A5 A6 B1 B2 B3 B4 B5 B6')
        self.rows = self.rows_str.split(' ')
        self.params = {'n_users': len(self.car_states.users),
                       'users': list(self.car_states.users)}

        self.ns = "/rasberry"
        ws_url = self.ns+'/'
        www = path.join(path.dirname(__file__),'www')

        webnsock.WebServer.__init__(self, add_static=www+'/static', static_prefix=ws_url)

        self._renderer = web.template.render(path.realpath(www), base='base', globals=globals())

        # self.map = {'0': {'0': ['0>1', '1>2'], '1': ['0>1', '1>2'], '2': ['0>1', '1>2'], '3': ['0>1', '1>2'], '4': ['0>1', '1>2']},
        #             '1': {'6': ['0>1', '1>2'], '7': ['0>1', '1>2'], '8': ['0>1', '1>2'], '9': ['0>1', '1>2'], '10': ['0>1', '1>2']}}
        # self.robots = {'short': {'logistics': ['thorvald_014']},
        #                'tall': {'uv_treatment': ['thorvald_002_tall', 'thorvald_030'],
        #                         'data_gathering': ['thorvald_002_tall']}}

        self_app = self

        class AMZBtn(self.page):
            path = '/car/button'

            def POST(self):
                user_data = web.input(username='')
                info('pressed the button')
                user = str(user_data.username)
                if user is not '':
                    self_app.car_states.users[user] = web.ctx
                    self_app.params = {
                        'n_users': len(self_app.car_states.users),
                        'users': list(self_app.car_states.users)
                    }

                    info('user %s pressed the button')
                    self_app.car_states.set_state(user, 'BUTTON')
                    self_app.car_states.send_updated_states()
                return web.ok()

        class Download(self.page):
            path = '/car/log'

            def GET(self):
                web.header(
                    'Content-Disposition', 'attachment; filename="car.log"')
                web.header('Content-type', 'images/jpeg')
                web.header('Content-transfer-encoding', 'binary')
                return open(self_app.car_states.log_filename, 'rb').read()

        class RobotInteractions(self.page):

            def INIT(self):
                print("This needs to be overloaded.")

            def GET(self):
                print("\n\n\nGET "+self.ri_type)
                self.user = web.cookies().get('_%s_user'%self.ri_ref)
                print("user: " + str(self.user))
                self_app.car_states.users[self.user] = web.ctx
                self_app.params = {
                    'n_users': len(self_app.car_states.users),
                    'users': list(self_app.car_states.users)
                }
                if self.user is None:
                    print("user is None: " + str(self.user))
                    return self_app._renderer.login(self_app.params, self_app.get_text, self.path, self.ri_type)
                else:
                    print("user is not None: " + str(self.user))
                    return self.INIT()

            def POST(self):
                print('RI.POST')
                user_data = web.input(username='')
                self.user = user_data.username
                if self.user is not '':
                    info('login as %s' % user_data)
                    web.setcookie('_%s_user'%self.ri_ref, self.user)
                else:
                    web.setcookie('_%s_user'%self.ri_ref, '', -1)
                print('RI.seeother : '+ self.path)
                return web.seeother(self.path)

        class SendARobot(RobotInteractions):
            ri_type = 'SendARobot'
            ri_ref = 'sar'
            path = self_app.ns+'/sar/'

            def INIT(self):
                req = ['_map','_robots']
                if any([i not in server_details for i in req]): return self_app._renderer.resourcemissing(self_app.params)
                for r in req: web.setcookie(r,server_details[r])
                return self_app._renderer.sendarobot(self_app.params, self_app.get_text, self.user, self_app.websocket_url)

        class CallARobot(RobotInteractions):
            ri_type = 'CallARobot'
            ri_ref = 'car'
            path = self_app.ns+'/car/'

            def INIT(self):
                row = ''
                if 'row' in self_app.car_states.gps[self.user]:
                    row = self_app.car_states.gps[self.user]['row']
                return self_app._renderer.callarobot(self_app.params, self_app.get_text, self.user, self_app.rows, self_app.websocket_url, row)

        class Orders(RobotInteractions):
            ri_type = 'Orders'
            ri_ref = 'ras'
            path = self_app.ns+'/orders/'

            def INIT(self):
                print('Orders.INIT : '+self.user)
                if self.user == 'lcas':
                    print('Orders.INIT.user==lcas : ' + self.user)
                    return self_app._renderer.orders(self_app.params, self_app.get_text, self_app.websocket_url, self_app.gmaps_api)

                print('bak2login: ' + self.user)
                return self_app._renderer.login(self_app.params, self_app.get_text, self.path, self.ri_type)


        # class Orders(self.page):
        #     path = self_app.ns+'/orders/'
        #
        #     def GET(self):
        #         print("\n\n\nGET "+self.ri_type)
        #         user = web.cookies().get('_admin_user')
        #         print("user: " + str(self.user))
        #
        #         self_app.params = {
        #             'n_users': len(self_app.car_states.users),
        #             'users': list(self_app.car_states.users)
        #         }
        #         if user is None:
        #             return self_app._renderer.login(self_app.params, self_app.get_text, user, self_app.ns+'/orders')
        #         else:
        #             return self_app._renderer.orders(self_app.params, self_app.get_text, self_app.websocket_url, self_app.gmaps_api)
        #
        #     def POST(self):
        #         user_data = web.input(username='')
        #
        #         user = user_data.username
        #         if user == 'lcas':
        #             info('admin login as %s' % user_data)
        #             web.setcookie('_admin_user', user)
        #         else:
        #             web.setcookie('_admin_user', '', -1)
        #         return web.seeother(self_app.ns+'/orders')

        class Redirector(self.page):
            def GET(self): return web.seeother(self.path.rstrip('/#')+'/')

        class RedirCAR(Redirector): path = self_app.ns+'/car'
        class RedirSAR(Redirector): path = self_app.ns+'/sar'
        class RedirOrd(Redirector): path = self_app.ns+'/orders'

        class RedirCARs(Redirector): path = self_app.ns+'/car/'
        class RedirSARs(Redirector): path = self_app.ns+'/sar/'
        class RedirOrds(Redirector): path = self_app.ns+'/orders/'

    def get_text(self, text):
        return text

    def stop(self):
        info('shutdown: self.is_running = False')
        self.is_running = False
        webnsock.WebServer.stop(self)


class CARProtocol(webnsock.JsonWSProtocol):

    def __init__(self):
        super(CARProtocol, self).__init__()
        self.car_states = car_states
        self.log_user = 'unknown'

    def onOpen(self):
        info('websocket opened')
        self.car_states.clients.add(self)

    def onClose(self, wasClean, code, reason):
        info("WebSocket connection closed: {0}".format(reason))
        if self in self.car_states.clients:
            info('unregistered')
            self.car_states.clients.remove(self)
        if self in self.car_states.admin_clients:
            info('unregistered')
            self.car_states.admin_clients.remove(self)

    def on_set_state(self, payload):
        print("\n\n")
        if '/info/' in payload['user']:
            cookie = '_'+payload['user'].split('/')[-1]
            info('cookie set of name %s: %s' % (cookie, payload['state']))
            server_details[cookie] = payload['state']
        else:
            info('update state for user %s: %s' % (payload['user'], payload['state']))
            self.update_state(payload['user'], payload['state'])

    def on_register(self, payload):
        info('registering management interface %s' % str(self))
        if payload['admin']:
            self.car_states.admin_clients.add(self)
        else:
            if payload['user'] in self.car_states.users:
                info('user %s already known' % payload['user'])
            else:
                info('new user %s registered' % payload['user'])
                self.car_states.users[payload['user']] = payload['user']
                self.update_state(payload['user'], 'INIT')
        self.log_user = payload['user']

    def on_get_states(self, payload):
        info('states requested')
        self.send_updated_states()

    def on_location_update(self, payload):
        info('GPS update: ' + str(payload))
        self.car_states.gps[payload['user']] = {
            'latitude': payload['latitude'],
            'longitude': payload['longitude'],
            'row': payload['row']   
        }
        self.car_states.log(
            payload['user'],
            state='GPS',
            latitude=payload['latitude'],
            longitude=payload['longitude'],
            row=payload['row'],
        )
        self.car_states.send_update_position(
            payload['user'],
            payload['latitude'],
            payload['longitude'],
            payload['accuracy'],
            payload['rcv_time'],
            payload['row']
            # payload['heading'],
            # payload['velocity']
            )

    def send_updated_states(self):
        self.car_states.send_updated_states(self)

    def update_state(self, user, state):
        # initialise the user if not already done
        if user not in self.car_states.states:
            self.car_states.get_state(user)
        self.car_states.set_state(user, state, self.log_user)
        self.send_updated_states()

    def on_ping(self, payload):
        return {'result': True}

    def on_call(self, payload):
        info('user %s called a robot' % payload['user'])
        self.update_state(payload['user'], 'CALLED')

    def on_cancel(self, payload):
        info('user %s cancelled a robot' % payload['user'])
        self.update_state(payload['user'], 'INIT')

    def on_get_state(self, payload):
        print("\n")
        info('user %s requested state' % payload['user'])
        return {
            'method': 'set_state',
            'state': self.car_states.get_state(payload['user'])
        }



    """Responses from Button presses on CallARobot."""
    def on_car_init(self, p):
        info('user(%s) initiated interface' % p['user'])
        self.update_state(p['user'], 'car_INIT')

    def on_car_call(self, p):
        info('user(%s) made a call' % p['user'])
        self.update_state(p['user'], 'car_CALLED')

    def on_car_accept(self, p): pass

    def on_car_arrived(self, p): pass

    def on_car_load(self, p):
        info('user(%s) completed loading' % p['user'])
        self.update_state(p['user'], 'car_LOADED')

    def on_car_cancel_task(self, p):
        info('user(%s) cancelled task' % p['user'])
        self.update_state(p['user'], 'car_CANCEL')

    def on_car_complete(self, p): pass

    def on_car_reset(self, p):
        info('user(%s) made a call' % p['user'])
        self.update_state(p['user'], 'car_RESET')



    """Responses from Button presses on SendARobot."""
    def on_sar_begin_task(self, p):
        us, t, r, e, ta, ro = p['user'], p['tunnel'], p['row'], p['edge'], p['task'], p['robot']
        info('user(%s) begun %s task with robot(%s) over t(%s)_r(%s)_e(%s)' % (us, ta, ro, t, r, e))
        self.update_state(us, 'sar_BEGUN-%s-%s-%s-%s-%s' % (t, r, e, ta, ro))

    def on_sar_begin_edge_task(self, p):
        us, t, r, e1, e2, ta, ro = p['user'], p['tunnel'], p['row'], p['edge1'], p['edge2'], p['task'], p['robot']
        e = 'all' if 'all' in [e1, e2] else '%s>%s'%(e1,e2)
        info('user(%s) begun %s task with robot(%s) over t(%s)_r(%s)_e(%s)' % (us, ta, ro, t, r, e))
        self.update_state(us, 'sar_BEGUN-%s-%s-%s-%s-%s' % (t, r, e, ta, ro))

    def on_sar_await_start(self, p): pass

    def on_sar_await_completion(self, p): pass

    def on_sar_cancel_task(self, p):
        info('user(%s) cancelled task' % p['user'])
        self.update_state(p['user'], 'sar_CANCEL')

    def on_sar_emergency_stop(self, p):
        info('user(%s) called an emergency stop' % p['user'])
        self.update_state(p['user'], 'sar_EMERGENCY_STOP')

    def on_sar_emergency_resume(self, p):
        info('user(%s) resumed an emergency stop' % p['user'])
        self.update_state(p['user'], 'sar_EMERGENCY_RESUME')

    def on_sar_init(self, p):
        self.update_state(p['user'], 'sar_INIT')  # On begin task, move SAR to next state


def main():
    webserver = webnsock.WebserverThread(CARWebServer())
    backend = webnsock.WSBackend(CARProtocol)
    signal(SIGINT,
           lambda s, f: webnsock.signal_handler(webserver, backend, s, f))
    webserver.start()
    backend.talker()


if __name__ == "__main__":
    main()
