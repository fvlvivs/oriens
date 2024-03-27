image="oriens-image:latest"
name="oriens-container"

docker container run --rm \
	-it \
	--name=${name} \
	--privileged \
	--env="DISPLAY" \
	--workdir /home/ros \
	--volume="$HOME/proiecta/oriens_ws/:/home/ros" \
	--volume=/dev:/dev \
	$image
