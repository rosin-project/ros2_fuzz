# client_service_example

This is the canonical example for fuzzing a client/service infrastructure.
It depends on the `tutorial_interfaces` package, which defines a
`AddThreeInts` ROS type.

This is the necessary excerpt in the `fuzz.yaml` file:

```yaml
services:
  add_three_ints:
    headers_file: tutorial_interfaces/srv/add_three_ints.hpp
    source: src/client_service_example/src/add_three_ints_server.cpp
    type: tutorial_interfaces::srv::AddThreeInts
    parameters: []
```
