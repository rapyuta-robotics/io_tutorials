FROM  nginx:stable
MAINTAINER Gautham (gautham.manoharan@rapyuta-robotics.com)

COPY ./nginx.conf /etc/nginx/nginx.conf
COPY ./site.conf /etc/nginx/conf.d/default.conf

RUN chmod -R g+w /var/log/nginx /var/cache/nginx /var/run

COPY env.sh /usr/bin/
COPY src/ /usr/share/nginx/html/

ENV WS="ws://localhost:9090"

EXPOSE 8080
ENTRYPOINT /bin/bash -c "bash /usr/bin/env.sh && nginx -g 'daemon off;'"
