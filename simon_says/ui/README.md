### Running locally
```
yarn install  
yarn start
```

### Running production
```
yarn install
ROSBRIDGE_WS='wss://1.2.3.4:5678/' yarn serve
```
Where 1.2.3.4 and :5678 are the IP address and port of the WS endpoint, respectively. You can also specify an accessible URL.

The ROS WS endpoint can be changed in `/src/config.js`

### Additional details
The default port is 3000
