export const rosEndpoint = window.rosEndpoint || process.env.REACT_APP_ROSBRIDGE_WS || 'ws:localhost:9090/';
export const modes = {
  off: {
    value: 'OFF',
    modeId: 0
  },
  source: {
    value: 'SOURCE',
    modeId: 1
  },
  sink: {
    value: 'SINK',
    modeId: 2
  },
  sourcesink: {
    value: 'SOURCESINK',
    modeId: 3
  }
};
export const modeMap = ['off', 'source', 'sink', 'sourcesink'];
export const statuses = {
  0: 'OFF',
  1: 'ERROR',
  2: 'RUNNING'
};
export const directions = {
  up: ['linear.x', 1],
  down: ['linear.x', -1],
  left: ['angular.z', 1],
  right: ['angular.z', -1]
};

// hard-coded compatible devices
export const robots = {
  turtlebot: {
    name: 'Turtlebot',
    deviceId: 1,
    defaultMode: 'off',
    modes: ['off', 'sink']
  },
  alphabot: {
    name: 'AlphaBot',
    deviceId: 2,
    defaultMode: 'off',
    modes: ['off', 'sink']
  },
  gopi: {
    name: 'GoPiGo',
    deviceId: 3,
    defaultMode: 'off',
    modes: ['off', 'source', 'sink', 'sourcesink']
  },
  quad: {
    name: 'Quad',
    deviceId: 4,
    defaultMode: 'off',
    modes: ['off', 'sink']
  },
  joystick: {
    name: 'Joystick',
    deviceId: 0,
    defaultMode: 'off',
    modes: ['off', 'source']
  }
};
