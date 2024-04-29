# CallARobot
A simple web interface to call a robot

## Use in Docker

1. clone this repository: `git clone https://github.com/LCAS/CallARobot.git`
1. go into the `.devcontainer` directory: `cd .devcontainer`
1. run `docker compose up -d`
1. find the port to access the car server: `docker compose port nginx 80`, it should show something like `0.0.0.0:32881`
1. open you browser to go to http://localhost:PORTNUMBER, in the above case it would be http://localhost:32881


## Use Devcontainer

Even easier may be to use the devcontainer in VSCode:

1. In VSCode open the repository
1. It should prompt you to open as devcontainer
1. Once start, under ports, the forwarded port for the CAR server is found
1. In the VSCode terminal, run `python callarobot.py` and you should be able to access it under the "CAR server" forwarded port

## Installation

run `sudo pip install -r requirements.txt`

## Start Server

* run as `WEBSOCKET_URL="wss://lcas.lincoln.ac.uk/rasberry/ws" python callarobot.py`

## Start Client (usues ROS if available)

* run as `WEBSOCKET_URL="wss://lcas.lincoln.ac.uk/rasberry/ws" python ws_client.py`

## Using as ROS Client

The demo client `ws_client` will start as ROS node if it find `rospy` module

* `rostopic echo /car_client/get_states` returns a JSON string looking something like this on every state change:
    ```
    data: '{"states": {"marc": "INIT"}, "_id": "13ed12d2-a398-4bdc-95e4-27a4563fddfe"
      , "method": "update_orders"}'
    ```

* `rostopic echo /car_client/get_gps` returns a JSON string looking something like this on every gps update change:
    ```
    data: '{"lat": 53.232973799999996, "_id": "578a0829-1fec-4843-9e2d-6f82136058b5",
       "method": "update_position", "long": -0.5458096000000001, "user": "marc"
      }'
    ```

* set a state, by publishing a JSON string to `/car_client/set_state`: e.g.
    ```
    rostopic pub /car_client/set_states std_msgs/String "data: '{\"user\": \"marc\", \"state\": \"CALLED\"}'"
    ```

## Apache config

```
<Location /rasberry/car>
    SSLRequireSSL
    SetEnv proxy-initial-not-pooled
    Order allow,deny
    Allow from all
    ProxyPass http://localhost:8127/car
    ProxyPassReverse http://localhost:8127/rasberry/car
</Location>

<LocationMatch "/rasberry/car/ws">
    SSLRequireSSL
    ProxyPassMatch ws://localhost:8128/ disablereuse=On
    ProxyPassReverse ws://localhost:8128/
    Order allow,deny
    Allow from all
    #LogLevel Debug
</LocationMatch>
```
