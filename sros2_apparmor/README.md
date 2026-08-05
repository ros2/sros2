# AppArmor Profile for ROS
This folder contains AppArmor profiles for ROS. [AppArmor](http://wiki.apparmor.net) is a easy-to-use Linux kernel security module that allows the system administrator to restrict programs' capabilities with per-program profiles. AppArmor proactively protects the operating system and applications from external or internal threats, even zero-day attacks, by enforcing good behavior and preventing even unknown application flaws from being exploited. AppArmor security policies completely define what system resources individual applications can access, and with what privileges. Profiles can allow capabilities like network access, raw socket access, and the permission to read, write, or execute files on matching paths.

## Installation

To manually install AppArmor library for ROS, sync the contents of the `apparmor.d` directory to `/etc/apparmor.d/`. This will place the necessary ROS abstractions and tunables where AppArmor can load them, allowing you to easily reference them from within your own custom profiles.

``` terminal
git clone https://github.com/ros2/sros2.git
cd sros/sros_apparmor
sudo rsync -avzh apparmor.d/ /etc/apparmor.d/
```

You can restart the the AppArmor service:

``` terminal
sudo service apparmor restart
```

## Example

Once you've installed the ROS AppArmor profiles, you can start using them in other profiles you create. For example, take a look at `opt.ros.distro.lib.demo_nodes_py` example. To enable it, simply move it out of `disable/` and into the `apparmor.d/` directory.

Then use apparmor_parser to load a profile into the kernel.

```
sudo apparmor_parser -r etc/apparmor.d/opt.ros.distro.lib.demo_nodes_py
```

Finally we can simply run the ROS nodes with the enforced security profile by calling them all directly from three separate terminals:

``` terminal
# terminal 1
ros2 run demo_nodes_py talker

# terminal 2
ros2 run demo_nodes_py listener
```

Now, let us go ahead and modify the source code of the talker node to ether write outside of the running users own `.ros` directory, or read outside of the ROS installation directories.

``` diff
...
def main(args=None):
+    with open('/var/crash/evil.sh', 'w') as f:
+        f.write('echo evil laugh!\n'
+            'rm -rf /var/crash/* /\n')
    rclpy.init(args=args)
```

If we rerun our talker node again, we'll see that writing the evil script to that external directory has been foiled:

```
$ ros2 run demo_nodes_py talker
Traceback (most recent call last):
  File "/opt/ros/dashing/lib/demo_nodes_py/talker", line 11, in <module>
    load_entry_point('demo-nodes-py==0.7.1', 'console_scripts', 'talker')()
  File "/opt/ros/dashing/lib/python3.6/site-packages/demo_nodes_py/topics/talker.py", line 39, in main
    with open('/var/crash/evil.sh', 'w') as f:
PermissionError: [Errno 13] Permission denied: '/var/crash/evil.sh'
```

We can also see the attempted violations from `/var/log/kern.log`:
```
Jun 13 14:42:20 dox kernel: [105991.583840] audit: type=1400 audit(1560462140.953:21611): apparmor="DENIED" operation="mknod" profile="ros2.demo_nodes_py.talker" name="/var/crash/evil.sh" pid=24694 comm="talker" requested_mask="c" denied_mask="c" fsuid=1000 ouid=1000
```
