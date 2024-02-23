#!/bin/bash
gpu_option=$1
container_name="sciurus17"

if [[ $gpu_option == "-gpu" ]]; then
    container_name="${container_name}-gpu"
fi

ps_names=$(docker ps --format "{{.Names}}")

if [[ $ps_names == *"$container_name"* ]]; then
    echo "Container is already exists"
else
    echo "execute $container_name"
    docker compose up -d --build $container_name
fi

docker compose exec $container_name /bin/bash

