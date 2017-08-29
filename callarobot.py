#!/usr/bin/env python

import webnsock
import web
from signal import signal, SIGINT
from os import path
from logging import error, warn, info, debug, basicConfig, INFO

basicConfig(level=INFO)


class CARState:

    def __init__(self):
        self.states = {}
        self.managers = set([])
        self.users = {}
        for u in self.users:
            self.set_state(u, 'INIT')

    def get_state(self, user):
        if user not in self.states:
            self.states[user] = 'INIT'
        return self.states[user]

    def set_state(self, user, state):
        self.states[user] = state

    def send_updated_states(self, extra_socket=None):
        if extra_socket is None:
            addressees = set([])
        else:
            addressees = set([extra_socket])
        addressees = addressees.union(self.managers)
        for m in addressees:
            info('send update to manager %s' % str(m))
            m.sendJSON({
                'method': 'update_orders',
                'states': self.states
            })


car_states = CARState()


class CARWebServer(webnsock.WebServer):

    def get_text(self, text):
        return text

    def __init__(self):

        self.car_states = car_states

        self.params = {
            'n_users': len(self.car_states.users),
            'users': list(self.car_states.users)
        }

        webnsock.WebServer.__init__(
            self,
            path.join(
                path.dirname(__file__),
                'www/static'
            ),
            '/car/'
        )

        self._renderer = web.template.render(
            path.realpath(
                path.join(
                    path.dirname(__file__),
                    'www'
                )
            ),
            base='base', globals=globals())

        self_app = self

        class AMZBtn(self.page):
            path = '/car/button'

            def POST(self):
                user_data = web.input(username='')
                info('pressed the button')
                print user_data
                user = user_data.username
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

        class Index(self.page):
            path = '/car/'

            def GET(self):
                user = web.cookies().get('_car_user')
                if user is None:
                    return self_app._renderer.login(
                        self_app.params, self_app.get_text)
                else:
                    self_app.car_states.users[user] = web.ctx
                    self_app.params = {
                        'n_users': len(self_app.car_states.users),
                        'users': list(self_app.car_states.users)
                    }

                    print self_app.params
                    return self_app._renderer.index(
                        self_app.params, self_app.get_text, user)

            def POST(self):
                user_data = web.input(username='')

                user = user_data.username
                if user is not '':
                    info('login as %s' % user_data)
                    web.setcookie('_car_user', user)
                else:
                    web.setcookie('_car_user', '', -1)
                return web.seeother('/car/')

        class Orders(self.page):
            path = '/car/orders'

            def GET(self):
                return self_app._renderer.orders(
                    self_app.params, self_app.get_text)


class CARProtocol(webnsock.JsonWSProtocol):

    def __init__(self):
        super(CARProtocol, self).__init__()
        self.car_states = car_states

    def onOpen(self):
        info('websocket opened')
        self.car_states.managers.add(self)

    def onClose(self, wasClean, code, reason):
        info("WebSocket connection closed: {0}".format(reason))
        if self in self.car_states.managers:
            info('unregistered')
            self.car_states.managers.remove(self)

    def on_set_state(self, payload):
        info(
            'update state for user %s: %s' %
            (payload['user'], payload['state']))
        self.update_state(payload['user'], payload['state'])

    def on_register(self, payload):
        info('registering management interface %s' % str(self))
        self.car_states.managers.add(self)

    def on_get_states(self, payload):
        info('states requested')
        self.send_updated_states()

    def send_updated_states(self):
        self.car_states.send_updated_states(self)

    def update_state(self, user, state):
        self.car_states.set_state(user, state)
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
        info('user %s requested state' % payload['user'])
        return {
            'method': 'set_state',
            'state': self.car_states.get_state(payload['user'])
        }



def main():
    webserver = webnsock.WebserverThread(CARWebServer())
    backend = webnsock.WSBackend(CARProtocol)
    signal(SIGINT,
           lambda s, f: webnsock.signal_handler(webserver, backend, s, f))
    webserver.start()
    backend.talker()


if __name__ == "__main__":
    main()
