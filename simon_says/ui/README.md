### Running locally
```
yarn install  
yarn start
```
By default, the webserver will listen for a rosbridge server endpoint at ws:localhost:9090.

### Running production
```
yarn install
ROSBRIDGE_WS='wss://0.0.0.0:9090/' yarn serve
```
Where 0.0.0.0 and 9090 are the IP address and port of the rosbridge server endpoint, respectively. You can also specify a URL.

The ROSBRIDGE_WS endpoint can alternatively be changed in `/src/config.js`

### Additional details
The default port for the webserver is 3000.
