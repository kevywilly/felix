FROM nginx

WORKDIR /

RUN apt-get update && apt-get install -y git \
    && rm -rf /var/lib/apt/lists/*

COPY ./nginx.conf /etc/nginx/conf.d/default.conf

RUN git clone https://github.com/kevywilly/nano-control
RUN rm -rf /var/www/build
RUN mkdir -p /var/www/build
RUN cp -r nano-control/build /var/www/
RUN rm -rf nano-control

