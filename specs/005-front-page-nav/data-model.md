# Data Model: Front Page Module Navigation

## Module Card Entity

**Name**: ModuleCard
**Fields**:
- id: string (unique identifier for the module)
- title: string (display name of the module)
- description: string (brief description of module content)
- link: string (URL path to first chapter of the module)
- order: number (position in the grid layout)

**Validation Rules**:
- title: Required, max 100 characters
- description: Required, max 300 characters
- link: Required, valid URL path format
- id: Required, unique across all modules

## Module Configuration Array

**Structure**: Array of ModuleCard objects
**Purpose**: Data source for rendering module cards on homepage
**Constraints**: Fixed to 4 modules for current implementation

**Sample Data**:
```javascript
const modules = [
  {
    id: "module-1-ros2",
    title: "ROS 2 – Robotic Nervous System",
    description: "Module 1 – Using ROS 2 as middleware to control humanoid robots, bridging AI agents with physical actuators",
    link: "/docs/module-1-ros2-arch/chapter-1",
    order: 1
  },
  {
    id: "module-2-gazebo",
    title: "Digital Twin (Gazebo & Unity)",
    description: "Module 2 – Creating photorealistic simulation environments for humanoid robot testing and development",
    link: "/docs/module-2-gazebo-unity/chapter-1",
    order: 2
  },
  {
    id: "module-3-isaac",
    title: "AI-Robot Brain (NVIDIA Isaac)",
    description: "Module 3 – Advanced perception, simulation, and navigation for humanoid robots using NVIDIA Isaac",
    link: "/docs/module-3-isaac/chapter-1",
    order: 3
  },
  {
    id: "module-4-vla",
    title: "Vision-Language-Action (VLA)",
    description: "Module 4 – Integration of LLMs with robotics for voice-driven cognitive planning and autonomous humanoid behavior",
    link: "/docs/module-4-vla/chapter-1",
    order: 4
  }
];
```

## Homepage Layout Entity

**Name**: HomepageLayout
**Fields**:
- modules: Array of ModuleCard objects
- layoutType: string (grid or flex layout)
- responsiveBreakpoints: object (CSS breakpoints for different screen sizes)
- containerClasses: string (CSS classes for main container)

**Relationships**:
- Contains multiple ModuleCard entities
- Defines responsive behavior for all contained cards