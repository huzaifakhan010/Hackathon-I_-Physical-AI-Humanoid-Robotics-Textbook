---
sidebar_position: 3
---

# Services and Request-Response Patterns

In this section, we'll explore ROS 2 services, which enable request-response communication patterns. Services work like phone calls - one node makes a specific request, and another node responds with the requested information or confirms completion of a task.

## What are Services?

Services in ROS 2 provide a **request-response** communication pattern. Unlike topics which broadcast continuously, services are used for specific, on-demand interactions. When a node needs specific information or wants to request an action, it sends a request to a service, and the service responds with the requested data or confirmation.

### Key Characteristics of Services

#### Synchronous Communication
- Client sends a request and waits for a response
- Communication happens in pairs: request → response
- Client must wait for the response before continuing (in basic implementation)
- Like making a phone call and waiting for an answer

#### Direct Connection
- One client connects directly to one service server
- Unlike topics, multiple clients can't receive the same response
- The service handles one request at a time
- More like a private conversation than a broadcast

#### Request and Response Types
- Services have two message types: request and response
- Request type defines what information the client sends
- Response type defines what information the server returns
- Both types are defined together in a service definition file

## Phone Call Analogy

Think of ROS 2 services like making phone calls:

| Phone System | ROS 2 Service System | Function |
|--------------|---------------------|----------|
| Person making call | Service Client | Makes the request |
| Phone number | Service name | Address to contact |
| Person answering | Service Server | Processes the request |
| Conversation | Request/Response | Exchange of information |

Just as you dial a specific number to get specific information (like calling a restaurant for hours), a client node calls a specific service to get specific information or perform a specific action.

## Common Use Cases for Services

### Information Requests
- Getting current robot position
- Requesting battery status
- Asking for robot configuration
- Querying system parameters

### Action Requests
- Saving a map location
- Starting or stopping robot operations
- Calibrating sensors
- Changing robot modes

### Configuration Changes
- Setting system parameters
- Updating robot behavior settings
- Changing operational modes
- Loading new configurations

## Service Communication Flow

Here's how service-based communication works:

### 1. Service Definition
First, define the service interface in a `.srv` file:
```
# This is a conceptual example
# Request (input): Get the robot's current position
float64 x
float64 y
float64 theta
---
# Response (output): Return the position
float64 x
float64 y
float64 theta
bool success
string message
```

### 2. Service Server Setup
```python
# This is a conceptual example, not executable code
import rclpy
from rclpy.node import Node
# Import your service type (defined in the .srv file)
from example_interfaces.srv import Trigger  # Built-in service type

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server')
        # Create a service that responds to requests
        self.srv = self.create_service(
            Trigger,           # Service type
            'service_name',    # Service name
            self.service_callback  # Function to handle requests
        )

    def service_callback(self, request, response):
        # Process the request
        self.get_logger().info('Service request received')

        # Perform the requested action
        # (In this example, just return success)
        response.success = True
        response.message = 'Action completed successfully'

        return response  # Return the response
```

### 3. Service Client Setup
```python
# This is a conceptual example, not executable code
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client')
        # Create a client for the service
        self.client = self.create_client(Trigger, 'service_name')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def call_service(self):
        # Create a request object
        request = Trigger.Request()

        # Call the service asynchronously
        future = self.client.call_async(request)
        # Handle the response when it arrives
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        # Process the response
        response = future.result()
        self.get_logger().info(f'Service response: {response.success}')
```

### 4. The Communication Process
1. **Service Server registers** with ROS 2 system to provide a service
2. **Service Client discovers** the available service
3. **Client sends** a request to the service
4. **Server processes** the request and prepares a response
5. **Server sends** the response back to the client
6. **Client receives** and processes the response

## Service Names and Organization

### Naming Convention
- Use descriptive names that indicate the action: `/get_robot_pose`, `/save_map`
- Follow ROS 2 naming standards with forward slashes
- Be specific about what the service does
- Use verbs that indicate the action: `get_`, `set_`, `trigger_`

### Common Service Examples
- `/get_parameter` - Retrieve a specific parameter
- `/set_mode` - Change robot operational mode
- `/save_map` - Save current map to file
- `/get_plan` - Request a path plan from navigation
- `/trigger_camera` - Request camera to take a picture

## Advantages of Service-Based Communication

### Guaranteed Response
- Every request gets a response (even if it's an error)
- Clients know if their request was successful
- Good for critical operations that must complete

### Direct Interaction
- One-to-one communication ensures privacy
- No other nodes can intercept the request/response
- Suitable for sensitive operations

### Synchronous Processing
- Request is processed immediately by the server
- Good for operations that need immediate attention
- Clear cause-and-effect relationship

## Comparison: Topics vs Services

| Aspect | Topics | Services |
|--------|--------|----------|
| Communication Pattern | Publish/Subscribe | Request/Response |
| Timing | Continuous data flow | On-demand interaction |
| Connection | One-to-many | One-to-one |
| Use Case | Sensor data, status updates | Information requests, actions |
| Response | No direct response | Guaranteed response |
| Latency | Low (data pushed) | Higher (request-response cycle) |

## Service vs Topic: When to Use Each

### Use Topics When:
- Broadcasting sensor data continuously
- Sharing status that multiple nodes need
- Publishing data that changes frequently
- No immediate response is needed

### Use Services When:
- Requesting specific information
- Need confirmation of an action
- Performing configuration changes
- Need guaranteed response

## Quality of Service for Services

Services have different QoS considerations than topics:

### Reliability
- Services are inherently reliable - requests are guaranteed to be delivered
- Responses are guaranteed to return to the correct client
- Built-in retry mechanisms for failed requests

### Deadline
- Can set timeouts for service calls
- Useful for real-time systems where responses must arrive within time limits

### Durability
- Services don't typically store historical requests
- Each request is processed independently

## Common Beginner Mistakes

### Service Not Available
- **Problem**: Client tries to call service before server is ready
- **Solution**: Always check if service is available using `wait_for_service()`

### Timeout Issues
- **Problem**: Service takes too long to respond
- **Solution**: Set appropriate timeout values and handle timeout exceptions

### Request/Response Type Mismatches
- **Problem**: Client and server use different service definitions
- **Solution**: Ensure both use the same service type definition

### Blocking the Main Thread
- **Problem**: Synchronous service calls blocking other operations
- **Solution**: Use asynchronous service calls when possible

## Key Takeaways

- Services enable request-response communication in ROS 2
- Use services for specific information requests and actions
- Services work like phone calls - direct, synchronous communication
- Services guarantee a response to every request
- Choose services over topics when you need specific information or actions
- Services are perfect for configuration changes, information queries, and action requests

## Exercise

Think of a robot that can navigate around a building. Identify 5 different services this robot would need and explain:

1. What each service does
2. What the request would contain
3. What the response would contain
4. Why a service is better than a topic for this use case

Consider services for navigation, safety, and system management.