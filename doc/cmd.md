port
```
sudo socat PTY,link=/dev/ttyV0,mode=777 PTY,link=/dev/ttyV1,mode=777
```


cmd_vel

```

ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "
    twist:
    linear:
        x: 0.7
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 1.0"
```