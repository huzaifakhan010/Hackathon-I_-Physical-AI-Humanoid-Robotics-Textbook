---
id: 1
title: Create Project Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution, setup, governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```text
Project:
AI/Spec-Driven Book on Physical AI & Humanoid Robotics with Embedded RAG Chatbot

Core principles:
- Technical accuracy aligned with real-world robotics systems
- Clarity for graduate-level AI & Robotics learners
- Spec-driven, reproducible content generation (Spec-Kit Plus)
- Practical focus on embodied intelligence and humanoid systems

Key standards:
- All technical claims must align with official docs (ROS 2, Gazebo, NVIDIA Isaac, OpenAI)
- Code examples must be executable or clearly labeled pseudocode
- Consistent terminology across AI, robotics, and simulation
- Modular, self-contained chapters

Constraints:
- Format: Markdown (Docusaurus-compatible)
- Authoring: Claude Code + Spec-Kit Plus
- Deployment: GitHub Pages
- Include code (Python, ROS 2, URDF, pseudocode)
- Integrated RAG chatbot (FastAPI, OpenAI Agents/ChatKit, Neon, Qdrant)
- Chatbot answers strictly from book content or user-selected text

Scope:
- ROS 2 (robotic nervous system)
- Gazebo & Unity (digital twins)
- NVIDIA Isaac (perception, navigation)
- Vision-Language-Action with LLMs
- Capstone: Voice-to-action autonomous humanoid

Success criteria:
- Fully deployable book with valid navigation
- Functional embedded RAG chatbot
- No broken links or invalid configs
- Production-ready capstone curriculum
```

## Response snapshot

Created the project constitution file with the following principles:
1. Technical Accuracy and Real-World Alignment
2. Graduate-Level Clarity for AI & Robotics Learners
3. Spec-Driven Reproducible Content Generation
4. Embodied Intelligence and Humanoid Systems Focus
5. Modular and Self-Contained Chapters
6. Integrated RAG Chatbot Functionality

The constitution includes sections for technical constraints, development workflow standards, and governance guidelines.

## Outcome

- ✅ Impact: Created initial project constitution at .specify/memory/constitution.md with 6 core principles and governance structure
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md
- 🔁 Next prompts: Develop specification for first chapter or feature
- 🧠 Reflection: Constitution captures all key requirements from user input and establishes governance framework

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - all requirements met
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin developing first chapter specification