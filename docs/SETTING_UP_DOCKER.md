# **Setting up Atlas and Bestla Docker Containers**

This guide outlines the steps to set up and run the Atlas and Bestla robot simulations using Docker.

## **1\. Clone the Repository**

First, you need to clone the necessary repository from GitHub. Open your terminal and run the following command:

git clone https://github.com/RAISE-NTU/ohms\_netsim\_robot\_workspaces.git

## **2\. Navigate to the Docker Directory**

Once the repository is cloned, change your current directory to the atlas\_docker folder:

cd ohms\_netsim\_robot\_workspaces/atlas\_docker

## **3\. Create a Docker Network**

To allow the containers to communicate with each other, you need to create a dedicated Docker network.

docker network create robotAnet

## **4\. Build and Run the Atlas Docker Container**

### **Build the Image**

From within the atlas\_docker directory, build the Docker image for Atlas:

docker build \-t atlas:latest .

### **Run the Container**

Now, run the Atlas container. This command connects it to the network you created and sets up the display environment for GUI applications.

sudo docker run \-it \--env="DISPLAY" \--network=robotAnet \--env="QT\_X11\_NO\_MITSHM=1" \-v /tmp/.X11-unix:/tmp/.X11-unix:rw \--name=atlas atlas:latest /bin/bash

*Note: You may need to run xhost \+local:docker on your host machine to allow the container to access your display.*

## **5\. Build and Run the Bestla Docker Container**

### **Navigate and Build**

First, navigate to the bestla\_docker directory from the root of the cloned repository:

\# Assuming you are at the root of ohms\_netsim\_robot\_workspaces  
cd bestla\_docker

*If you are still in the atlas\_docker directory, you can use cd ../bestla\_docker.*

Now, build the Bestla image:

docker build \-t bestla:latest .

### **Run the Container**

Finally, run the Bestla container, connecting it to the same robotAnet network.

sudo docker run \-it \--network=robotAnet \--name=bestla bestla:latest /bin/bash

After these steps, both your atlas and bestla containers will be running and connected to the robotAnet network, allowing them to communicate with each other.