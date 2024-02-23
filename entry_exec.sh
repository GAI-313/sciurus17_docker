#!/bin/bash
ps_names=$(docker ps --format "{{.Names}}")

if [[ $ps_names == *"humble-s17"* ]]; then
    echo "Container is already exists"
else
    docker compose up -d --build sciurus17
fi
docker compose exec sciurus17 /bin/bash
