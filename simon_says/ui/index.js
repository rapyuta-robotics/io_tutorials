const express = require('express');
const fs = require('fs');

const rosEndpoint = process.env.ROSBRIDGE_WS;

let htmlString = fs.readFileSync('build/index.html').toString();

fs.unlink('build/index.html', () => {});

if(rosEndpoint) {
  htmlString = htmlString.replace('rosEndpoint=""', 'rosEndpoint="'+(
    rosEndpoint.replace('https', 'wss')
  )+'"');
}

const app = express();
app.use(express.static('build'));
app.get('*', (req, res) => {
  return res.send(htmlString);
});

app.listen(3000, () => console.log('Express app listening on 3000!'));
