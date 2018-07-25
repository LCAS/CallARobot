# CallARobot
A simple web interface to call a robot

## Start Server

`WEBSOCKET_URL="wss://lcas.lincoln.ac.uk/car/ws" python callarobot.py`

## Start Client

`WEBSOCKET_URL="wss://lcas.lincoln.ac.uk/car/ws" python callarobot.py`


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