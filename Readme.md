**Robot and Laptop Networking Guide**

This guide explains how to connect a new laptop to the robot setup using CycloneDDS and SSH keys.

**1. Startup sequence**

Power on the robot without the Ethernet cable attached

Wait until the robot has fully booted

After that, plug in the Ethernet cable

The laptop should use the Local Link network and will appear as 192.168.0.71

**2. CycloneDDS configuration on the laptop**

Your laptop needs a cyclonedds_profile.xml in the home directory or any location you reference via CYCLONEDDS_URI.

Example file:

```
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>false</AllowMulticast>
      <EnableMulticastLoopback>false</EnableMulticastLoopback>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>

    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>

      <Peers>
        <Peer Address="192.168.0.102"/>
        <Peer Address="localhost"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Adjust the Peer Address to match the robot’s IP on the Local Link network.

**3. CycloneDDS configuration on the robot**

On the robot the file is located at:/etc/opt/tmc/robot/cyclonedds_profile.xml

**4. Required environment variables**

Add these lines to .bashrc on both robot and laptop:
```
export ROS_DOMAIN_ID=5
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/cyclonedds_profile.xml
bash
```

**5 Backpack Laptop SSH**

```
ssh-keygen -t ed25519 -C "yourname" -f ~/.ssh/id_ed25519_hassouna
```

this creates:
id_ed25519_hassouna
id_ed25519_hassouna.pub

use that public key in your github

**6. Git identity setup**

Add this alias to backpack .bashrc:
```alias use_hassouna='git config user.name "sunava" && git config user.email "hassouna@example.com"' ```
before commiting: 
``` $ use_hassouna```


**7. cyclone_ws on every new laptop**

Each new laptop needs the cyclone_ws workspace cloned, built, and sourced.
Use the same workspace layout as on existing machines.

