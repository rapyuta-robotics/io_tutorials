import React from 'react';
import _ from 'lodash';
import ROSLIB from 'roslib';
import mousetrap from 'mousetrap';

import {
  rosEndpoint,
  robots,
  modes,
  modeMap,
  statuses,
  directions,
} from './config';

const initialCommand = {
  linear: {x: 0, y: 0, z: 0},
  angular: {x: 0, y: 0, z: 0},
};
let isStopped = true;
let publishCommand;
const resetCommand = () => {
  isStopped = true;
  publishCommand = { ...initialCommand };
};

class Controller extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      directionArray: [],
      mouseInterval: null,
      ros: null,
      robotModes: _.mapValues(robots, robot => robot.defaultMode),
      robotStatus: _.mapValues(robots, () => 'live'),
    };
    this.updateMode = this.updateMode.bind(this);
    this.addDirection = this.addDirection.bind(this);
    this.removeDirection = this.removeDirection.bind(this);
    this.offDevices = this.offDevices.bind(this);
  }
  addDirection(d) {
    const { directionArray } = this.state;
    if (!_.includes(directionArray, d)) {
      this.setState({
        directionArray: [...directionArray, d],
      });
    }
  }
  removeDirection(d) {
    _.set(
      publishCommand,
      directions[d][0],
      0,
    );
    const { directionArray } = this.state;
    if (_.includes(directionArray, d)) {
      this.setState({
        directionArray: _.filter(directionArray, n => n !== d),
      });
    }
  }
  componentDidMount() {
    resetCommand();
    this.setState({
      ros: new ROSLIB.Ros(),
      mouseInterval: setInterval(() => {
        const { directionArray } = this.state;
        if (_.size(directionArray) > 0) {
          _.each(directionArray, d => {
            _.set(
              publishCommand,
              directions[d][0],
              _.get(publishCommand, directions[d][0]) + directions[d][1]
            );
          });
          isStopped = false;
          this.state.messageCommand.publish(
            new ROSLIB.Message(publishCommand),
          );
        } else if(!isStopped) {
          resetCommand();
          this.state.messageCommand.publish(
            new ROSLIB.Message(publishCommand),
          );
        }
      }, 250),
    }, () => {
      const {
        ros,
      } = this.state;
      ros.connect(rosEndpoint);
      this.setState({
        messageStatus: new ROSLIB.Topic({
          ros,
          name: '/ui/device_status',
          messageType: 'simon_says_msgs/StatusList'
        }),
        messageMode: new ROSLIB.Topic({
          ros,
          name: '/ui/set_mode',
          messageType: 'simon_says_msgs/SetMode'
        }),
        messageCommand: new ROSLIB.Topic({
          ros,
          name: '/ui/command',
          messageType: 'geometry_msgs/Twist'
        }),
      }, () => {
        this.state.messageStatus.subscribe(({
          status_list: statusList,
        }) => {
          const robotStatus = _.mapValues(robots, r => statusList[r.deviceId].data);
          const robotModes = _.mapValues(robots, r => modeMap[statusList[r.deviceId].mode.data]);
          // Because the status is received a lot of times but usually doesn't change.
          // Render is slowed down if the state is updated for every message.
          // This is an optimization to render only if the status is changed.
          // False positives are not an issue in this case but false negatives will not occur.
          if(
            JSON.stringify(robotStatus) !== JSON.stringify(this.state.robotStatus) ||
            JSON.stringify(robotModes) !== JSON.stringify(this.state.robotModes)
          ) {
            console.log(JSON.stringify(robotModes));
            this.setState({
              robotStatus,
              robotModes,
            });
          }
        });
      });
    });
    const keyDirections = {
      w: 'up',
      a: 'left',
      s: 'down',
      d: 'right',
    };
    _.each(keyDirections, (dir, key) => {
      mousetrap.bind(key, () => this.addDirection(dir), 'keydown');
      mousetrap.bind(key, () => this.removeDirection(dir), 'keyup');
    });
    mousetrap.bind('k', this.offDevices);
    mousetrap.bind('space', () => {
      resetCommand();
      this.state.messageCommand.publish(
        new ROSLIB.Message(publishCommand),
      );
    });
  }
  updateMode(robot, mode) {
    const robotModes = {
      ...(
        (mode === 'source' || mode === 'sourcesink') ? _.mapValues(robots, r => r.defaultMode) : this.state.robotModes
      ),
      [robot]: mode,
    };
    _.each(robotModes, (mode, robot) => {
      this.state.messageMode.publish(
        new ROSLIB.Message({
          device: {id: robots[robot].deviceId},
          mode: {data: modes[mode].modeId},
        }),
      );
    })
  }
  offDevices() {
    _.each(this.state.robotModes, (mode, robot) => {
      this.state.messageMode.publish(
        new ROSLIB.Message({
          device: {id: robots[robot].deviceId},
          mode: {data: modes.off.modeId},
        }),
      );
    })
  }
  componentWillUnmount() {
    clearInterval(this.state.mouseInterval);
  }

  moveEvents(direction) {
    this.setState({
      direction,
    });
  }

  render() {
    const {
      robotModes,
      robotStatus,
    } = this.state;
    return (
      <div id="leftbar">
        <img src="/logo.png" id="logo" alt=""/>
        <h4>Devices</h4>
        <div
          className="sectionContents"
        >
          {
            _.map(robots, (robot, robotKey) => (
              <div className="deviceRow" key={robotKey}>
                <div className={`deviceStatus ${statuses[robotStatus[robotKey]]}`} />
                <p>{robot.name}</p>
                {
                  statuses[robotStatus[robotKey]] !== statuses[0] && (
                    <select
                      value={robotModes[robotKey]}
                      onChange={(e) => {this.updateMode(robotKey, e.target.value)}}
                    >
                      {
                        _.map(robot.modes, mode => (
                          <option value={mode} key={mode}>{modes[mode].value}</option>
                        ))
                      }
                    </select>
                  )
                }
              </div>
            ))
          }
        </div>
        <h4>Controls</h4>
        <div
          className="sectionContents"
        >
          <div className="controlsRow">
            <button
              className="controlButton"
              onMouseDown={() => this.addDirection('up')}
              onMouseUp={() => this.removeDirection('up')}
            >
              <img src="/up.svg" alt=""/>
              <p className="buttonLetter">W</p>
            </button>
          </div>
          <div className="controlsRow">
            <button
              className="controlButton"
              onMouseDown={() => this.addDirection('left')}
              onMouseUp={() => this.removeDirection('left')}
            >
              <img src="/anticlock.svg" alt=""/>
              <p className="buttonLetter">A</p>
            </button>
            <button
              className="controlButton"
              onMouseDown={() => this.addDirection('down')}
              onMouseUp={() => this.removeDirection('down')}
            >
              <img src="/down.svg" alt=""/>
              <p className="buttonLetter">S</p>
            </button>
            <button
              className="controlButton"
              onMouseDown={() => this.addDirection('right')}
              onMouseUp={() => this.removeDirection('right')}
            >
              <img src="/clock.svg" alt=""/>
              <p className="buttonLetter">D</p>
            </button>
          </div>
        </div>
      </div>
    )
  }
}

export default Controller;
