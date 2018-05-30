FROM kkarczmarczyk/node-yarn:latest

COPY . /appsrc

RUN chmod -R g+w /appsrc

ENV NODE_ENV production
WORKDIR /appsrc
RUN yarn install

EXPOSE  3000
CMD ["yarn", "serve"]
