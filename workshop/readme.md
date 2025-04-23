## Introduction

The workshop documents are written in `Markdown` language and built using [Sphinx](https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html?) documentation generator.

## Generate the docs
### Install dependencies

#### Clone the repository
```bash
git clone <repo-name>
```

#### Setup the virtual environment in the workshop folder to generate the docs
Navigate to the workshop folder
```bash
cd ros2_i_training/workshop
```

Create the python environment and activate it
```bash
python3 -m venv sphinxEnv
source sphinxEnv/bin/activate
```

install the dependencies from the requirements.txt
````bash
python3 -m pip install -r requirements.txt
````

### Build the html doc
From the workshop directory:
 ```bash
 make html
 ```

### Open the docs
From the workshop directory:
```bash
xdg-open build/html/index.html
```

### To add new content
Place the content source in the following folders in the appropriate session.
-  `_source`: Markup (.md) files 
- `_static`: Images and other resource files  

Add the workshop's heading and filepath relative to `~/ros2_i_training/workshop/source/_source` to `index.rst`. To build the html:
 ````bash
 cd ~/ros2_i_training/workshop/
 make html
 ````
`index.rst` is built into `index.html` in the documentation output directory `~/ros2_i_training/workshop/build/html/index.html`. 

![docs](/workshop/source/_static/demo_rtd.png)


### Setup Docker for the workshops

To install `docker` on your `Ubuntu` system, follow the instructions ([here](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-18-04). To use `docker` without `sudo`, make sure to follow the instructions in step 2, "Executing the Docker Command Without Sudo".

The docker image for the training is available on [DockerHub](https://hub.docker.com/r/ipahsd/ros2-training-foxy).

```
docker pull ipahsd/ros2-training-foxy
```
To test the docker image, run:
```
docker run -it --rm ipahsd/ros2-training-foxy ros2 run demo_nodes_cpp talker
```
In a new terminal, run:
```
docker run -it --rm ipahsd/ros2-training-foxy ros2 run demo_nodes_cpp listener
```
To be able to use RViz and Gazebo, the docker container needs graphics support. In the terminal:
```
wget https://raw.githubusercontent.com/ros-planning/moveit2/main/.docker/gui-docker
chmod +x gui-docker
./gui-docker -it ipahsd/ros2-training-foxy:01 rviz2
```
