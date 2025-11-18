# ros2_parrot_arsdk

A colcon wrapper around the Parrot ARSDK. 

## Dependencies

```
sudo apt install libavahi-client-dev
```

## Installation

For the installation :

```
cd ~/ros2_ws/src
git clone https://github.com/jeremyfix/ros2_parrot_arsdk src/ros2_parrot_arsdk
colcon build --packages-select ros2_parrot_arsdk
```

The build shoud take around 2 to 3 minutes.

If this fails to build quickly, it happens to be due to the repo command waiting for a reply on enabling color display. I'm not sure how to bypass that question, at least you can 

```
cd ros2_ws/src/ros2_parrot_arsdk
mkdir build
cd build
cmake ..
make
```

just to get prompted to answer the question.
