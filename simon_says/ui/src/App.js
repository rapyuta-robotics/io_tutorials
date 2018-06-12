import React, { Component } from 'react';

import Controller from './Controller';

class App extends Component {
  constructor(props) {
    super(props);
  }
  render() {
    return (
      <div id="contents">
        <Controller />
      </div>
    );
  }
}

export default App;
