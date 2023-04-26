## Installation Instructions

#### Make environment
```
mkdir -p nasa-uam-colcon/src
cd nasa-uam-colcon/
git clone https://github.com/uli-s2a2/nasa_uam.git src --recursive
```

#### Install Dependecies

```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

#### Build!

```
colcon build --symlink-install
```
