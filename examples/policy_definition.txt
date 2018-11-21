## Values used in policy generation
**ipc_types**: The type of ROS IPC in use, such as service, action, or topic  
## Definitions
|name|description|
|----|-----------|
|access_permission|The access permission of that node for the specified icp|
|ipc|Inter-process communication, how messages get from one node to another|
|ipc identifier|The specific subsystem id to provide access to (topic name, service name...)|
|ipc types|The inter-process communication subsystem (topics, services...)|

## Options
Most ipc permissions are given on a client/source basis.
Parameter permissions are slightly different. These specify whether this node is allowed to read/write to another node.
|ipc_type|identifier|access permission options|
|--------|----------|-------------------------|
|topics|topic name|subscribe, publish|
|services|service name|request, reply|
|actions|action name|call, execute|
|parameters|node_name|read, write|

## Policy yaml file layout

```
nodes:
    <node_name>:
        <ipc_type>:
            <ipc_identifier>
                access:
                    -<access_permissions>
```
