FROM nginx

WORKDIR /

RUN apt-get update && apt-get install -y git \
    && rm -rf /var/lib/apt/lists/*

COPY ./nginx.conf /etc/nginx/conf.d/default.conf

WORKDIR /var/www
RUN git clone https://github.com/kevywilly/nano-control -b felix

