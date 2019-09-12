# CallARobot
A simple web interface to call a robot

## Installation

run `sudo pip install -r requirements.txt`

## Start Server

* run as `WEBSOCKET_URL="wss://lcas.lincoln.ac.uk/car/ws" python callarobot.py`

## Start Client (usues ROS if available)

* run as `WEBSOCKET_URL="wss://lcas.lincoln.ac.uk/car/ws" python ws_client.py`

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
<Location /car>
    SSLRequireSSL
    SetEnv proxy-initial-not-pooled
    Order allow,deny
    Allow from all
    ProxyPass http://localhost:8127/car
    ProxyPassReverse http://localhost:8127/car
</Location>

<LocationMatch "/car/ws">
    SSLRequireSSL
    ProxyPassMatch ws://localhost:8128/ disablereuse=On
    ProxyPassReverse ws://localhost:8128/
    Order allow,deny
    Allow from all
    #LogLevel Debug
</LocationMatch>
```