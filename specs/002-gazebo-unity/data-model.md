# Data Model: Digital Twin (Gazebo & Unity)

## Core Entities

### Digital Twin
- **Definition**: A virtual representation of a physical robot system that enables simulation, testing, and validation of AI algorithms in a safe environment
- **Attributes**:
  - Physical system representation (robot model, sensors, environment)
  - Simulation fidelity level (physics, visual, sensor accuracy)
  - Synchronization mechanism (real-time vs offline)
  - Integration points (ROS 2 interfaces, data exchange protocols)
- **Relationships**: Connects physical robot to virtual simulation environment
- **Validation**: Must accurately reflect physical system characteristics

### Gazebo Simulation Environment
- **Definition**: A physics-based simulation environment that provides realistic physics, gravity, and collision detection for robot models
- **Attributes**:
  - Physics engine (ODE, Bullet, Simbody)
  - World configuration (environment, lighting, objects)
  - Sensor plugins (LiDAR, cameras, IMU, etc.)
  - Robot models (URDF/SDF definitions)
- **Relationships**: Provides physics simulation layer for digital twin
- **Validation**: Must provide realistic physics behavior matching real-world

### Unity Environment
- **Definition**: A high-fidelity rendering environment that provides realistic visual representation and interaction capabilities for robot simulation
- **Attributes**:
  - Rendering quality settings (textures, lighting, shadows)
  - Scene configuration (3D models, environments)
  - Interaction systems (user input, physics)
  - ROS 2 integration (Unity Robotics Hub)
- **Relationships**: Provides visual rendering layer for digital twin
- **Validation**: Must provide realistic visual representation for perception

### LiDAR Sensor
- **Definition**: A light detection and ranging sensor that provides 3D point cloud data for environment mapping and navigation
- **Attributes**:
  - Range limits (min/max detection distance)
  - Resolution (angular resolution, scan frequency)
  - Field of view (horizontal/vertical)
  - Noise characteristics (accuracy, precision)
- **Relationships**: Simulates real LiDAR sensor data in digital twin
- **Validation**: Must generate realistic point cloud data matching specifications

### Depth Camera
- **Definition**: A sensor that captures depth information for 3D scene understanding and object recognition
- **Attributes**:
  - Resolution (image width/height)
  - Field of view (horizontal/vertical)
  - Depth range (min/max distance)
  - Noise characteristics (depth accuracy)
- **Relationships**: Provides depth perception in digital twin
- **Validation**: Must generate realistic depth images matching specifications

### IMU (Inertial Measurement Unit)
- **Definition**: A sensor that measures acceleration, angular velocity, and orientation for robot state estimation
- **Attributes**:
  - Accelerometer range (measurement limits)
  - Gyroscope range (angular velocity limits)
  - Noise characteristics (bias, drift)
  - Update rate (frequency of measurements)
- **Relationships**: Provides motion sensing in digital twin
- **Validation**: Must generate realistic motion data matching specifications

### Physics Simulation
- **Definition**: Computational modeling of physical forces, gravity, and collisions to create realistic robot behavior in virtual environments
- **Attributes**:
  - Gravity parameters (acceleration, direction)
  - Collision detection (algorithm, accuracy)
  - Friction coefficients (static, dynamic)
  - Dynamics model (mass, inertia, damping)
- **Relationships**: Provides realistic physical behavior in digital twin
- **Validation**: Must match real-world physics behavior within acceptable tolerance

## Conceptual Models

### Digital Twin Architecture
- **Components**:
  - Physical Robot (real-world system)
  - Communication Layer (data exchange mechanism)
  - Virtual Model (simulation representation)
  - Interface Layer (user interaction)
- **Behavior**: Bidirectional synchronization between physical and virtual systems

### Simulation Pipeline
- **Gazebo Pipeline**: Robot Model → Physics Engine → Sensor Simulation → ROS 2 Interface
- **Unity Pipeline**: Robot Model → Visual Rendering → Interaction System → ROS 2 Interface
- **Integration**: ROS 2 as middleware connecting both simulation environments

### Sensor Fusion Model
- **Components**: Multiple sensor simulations (LiDAR, cameras, IMU, etc.)
- **Integration**: Data fusion algorithms in simulation environment
- **Output**: Combined perception data for AI agents

## State Models

### Simulation States
- **States**: Configuration → Initialization → Running → Paused → Stopped
- **Transitions**:
  - Configuration → Initialization: Simulation setup
  - Initialization → Running: Simulation start
  - Running ↔ Paused: Pause/resume
  - Any → Stopped: Simulation termination
- **Validation**: State transitions properly handled with appropriate cleanup

### Robot States in Simulation
- **States**: Idle → Moving → Sensing → Interacting → Error
- **Transitions**: Based on ROS 2 commands and sensor feedback
- **Validation**: State changes properly reflected in simulation

## Validation Rules

### From Functional Requirements
- **FR-001**: All digital twin concepts must align with established definitions and practices
- **FR-002**: Physics simulation must demonstrate realistic behavior in Gazebo
- **FR-003**: Unity environments must provide high-fidelity visual representation
- **FR-004**: Sensor simulations must generate realistic data for LiDAR, Depth Cameras, and IMUs
- **FR-005**: All exercises must be completable by target audience
- **FR-006**: Technical claims must be verified against authoritative sources
- **FR-007**: Content complexity appropriate for graduate-level learners
- **FR-008**: Terminology consistent across all chapters
- **FR-009**: Chapters function independently with clear dependencies
- **FR-010**: Configuration snippets included as specified
- **FR-011**: Content builds on prior ROS 2 knowledge from Module 1
- **FR-012**: Focus maintained on humanoid robot applications
- **FR-013**: ROS 2 integration concepts demonstrated properly
- **FR-014**: Advanced topics properly excluded per constraints

## Relationships Between Entities

### Hierarchical Relationships
- Module contains Chapters
- Chapters contain Concepts, Configuration Examples, and Exercises
- Concepts may contain Sub-concepts

### Dependency Relationships
- Unity environment requires ROS 2 integration setup
- Gazebo simulation requires robot model definitions
- Sensor simulation requires physics simulation
- Digital twin requires both Gazebo and Unity components

### Integration Relationships
- Gazebo and Unity connected via ROS 2 middleware
- Sensor data flows from simulation to AI agents
- Robot state synchronized between physical and virtual systems
- Configuration parameters affect both physics and visual simulation