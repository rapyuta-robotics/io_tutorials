import React from 'react';
import ReactDOM from 'react-dom';

import './main.css';

import App from './App';
import registerServiceWorker from './registerServiceWorker';

ReactDOM.render(<App />, document.getElementById('root'));
if(window.location.prototol === 'https:') {
  registerServiceWorker();
}
