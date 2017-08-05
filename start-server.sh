#!/bin/sh

#make directory that will be mounted in the docker instance.
mkdir -p ~/mydonkey/models
mkdir -p ~/mydonkey/datasets

# First check whether the named volume and image exist; if not, build/initialize them.

# Create volume if not already there
HASVOL=$( docker volume ls | grep mydonkey | wc -l )
if [ $HASVOL == 0 ]; then
	echo "start-server: Initializing Donkey server volume..." >&2
    docker volume create --name mydonkey
fi

HASVOL2=$( docker volume ls | grep donkey | wc -l )
if [ $HASVOL == 0 ]; then
	echo "start-server: Initializing Donkey code volume..." >&2
    docker volume create --name donkey
fi

# Build if not already built
HASIMAGE=$( docker image ls | grep donkey | wc -l )
if [ $HASIMAGE == 0 ]; then
	echo "start-server: Building Donkey server image..." >&2
    docker build -t donkey .
fi

# Now check the arguments and accordingly build/initialize/run the server.

while getopts ":vbd" opt; do
  case $opt in
  	v)
		if [ $HASVOL == 1 ]; then #means the volume existed, so it was not built
			echo "start-server: Initializing Donkey server volume..." >&2
			docker volume create --name mydonkey
		fi
		if [ $HASVOL2 == 1 ]; then #means the volume existed, so it was not built
			echo "start-server: Initializing Donkey code volume..." >&2
			docker volume create --name donkey
		fi
		;;
    b)
		if [ $HASIMAGE == 1 ]; then #means the image existed, so it was not built
			echo "start-server: Building Donkey server image..." >&2
			docker build -t donkey .
		fi
		;;
    d)
		echo "start-server: Running Donkey server container without serve.py and attaching..." >&2
		docker run -p 8887:8887 -v ~/mydonkey:/root/mydonkey -v ~/donkey:/donkey --entrypoint=/bin/bash -it donkey
		exit
		;;
    \?)
		echo "start-server: Invalid option: -$OPTARG" >&2
		exit
		;;
  esac
done

echo "start-server: Running Donkey server container..." >&2
docker run -p 8887:8887 -v ~/mydonkey:/root/mydonkey -v ~/donkey:/donkey donkey
