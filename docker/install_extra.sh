#!/bin/bash

# extra packages to install for development
script_content="
apt update
apt install clangd -y
apt install python3-pylsp -y
apt install software-properties-common -y
apt update
add-apt-repository ppa:maveonair/helix-editor -y
apt install helix -y
apt clean && rm -rf /var/lib/apt/lists/*"

container_name="oriens-container"
container_image="oriens-image"

# Check if container is running
if docker container inspect -f '{{.State.Status}}' "$container_name" | grep -q running; then
  echo "Container '$container_name' is running."
else
  echo "Error: Container '$container_name' is not running."
  exit 1
fi

docker exec -it "$container_name" bash -c "echo '$script_content' > /tmp/install_packages.sh"
docker exec -it "$container_name" bash -c "chmod +x /tmp/install_packages.sh"
docker exec -it "$container_name" bash -c "sudo /tmp/install_packages.sh"
docker exec -it "$container_name" bash -c "rm /tmp/install_packages.sh"

container_id=$(docker container inspect -f '{{.Id}}' "$container_name")
docker commit "$container_id" "$container_image:latest"

echo "Fineshed to install packages"
