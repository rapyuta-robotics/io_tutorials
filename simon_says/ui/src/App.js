import React, { Component } from 'react';

import Leftbar from './Leftbar';

class App extends Component {
  constructor(props) {
    super(props);
  }
  render() {
    return (
      <div id="contents">
        <Leftbar />
      </div>
    );
  }
}

export default App;
