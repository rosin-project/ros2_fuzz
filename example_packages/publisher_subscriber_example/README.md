# publisher_subscriber_example

This is the canonical example for fuzzing a publisher/subscriber infrastructure.

This is the necessary excerpt in the `fuzz.yaml` file:

```yaml
topics:
  minimal_topic:
    headers_file: std_msgs/msg/string.hpp
    source: src/publisher_subscriber_example/src/publisher_member_function.cpp
    type: std_msgs::msg::String
    parameters: []
```
