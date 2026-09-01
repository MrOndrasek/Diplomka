FROM python:3

WORKDIR /app



#RUN npm install

COPY . .

ENV port=9000
EXPOSE 9000

CMD ["npm","start"]