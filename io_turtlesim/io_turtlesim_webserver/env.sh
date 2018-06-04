#!/bin/bash

WS_URL='wss://localhost:9090'

if [ ! -z "$WS" ]; then
    WS_URL='wss://'${WS##*/}
fi

# write env object to env.js
cat <<EOT > /usr/share/nginx/html/env.js
env = {
  WS_URL: '${WS_URL}'
}
EOT
