<!--
Sync Impact Report:
Version change: 0.0.0 -> 1.0.0 (MAJOR: Initial constitution for new project)
List of modified principles: None (all new for this project)
Added sections: None (template sections filled)
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ✅ updated
- README.md ✅ updated
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. AI/Spec-Driven Development
Prioritize AI-driven tools (Claude Code, Spec-Kit Plus) for all development stages (spec, plan, tasks, code generation). Adhere to Spec-Driven Development (SDD) principles to ensure clear requirements, detailed planning, and automated task generation.

### II. Modular & Extensible Architecture
Design the book project, including Docusaurus, RAG chatbot, and any additional features (Signup/Signin, Personalization, Translation), with a modular and extensible architecture. Components must be loosely coupled, independently deployable, and easily extendable for future enhancements.

### III. Robust RAG Chatbot Integration
Ensure the integrated RAG chatbot is robust, accurate, and provides a seamless user experience. It must leverage OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud. The chatbot must answer questions about the book content, including text selection.

### IV. User-Centric Design & Personalization
Implement user-centric design principles, including optional Signup/Signin with Better-Auth. Personalization and translation features (Urdu) should enhance the reader's learning experience, with clear mechanisms for user control.

### V. Continuous Integration & Deployment
Establish a continuous integration and deployment (CI/CD) pipeline for the Docusaurus book to GitHub Pages. Ensure automated testing, building, and deployment processes are in place for rapid iteration and reliable updates.

### VI. Hardware & Simulation Awareness
Given the course's focus on Physical AI and Humanoid Robotics, ensure the textbook content accurately reflects the hardware requirements (RTX GPUs, Jetson kits, RealSense cameras) and simulation environments (Gazebo, Unity, NVIDIA Isaac Sim). Emphasize best practices for sim-to-real transfer and addressing latency concerns.

## Technical Stack & Tools

The project will utilize Docusaurus for book generation and deployment to GitHub Pages. The RAG chatbot will be built with OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud. Better-Auth will be used for Signup/Signin. Development will be driven by Claude Code and Spec-Kit Plus.

## Development Workflow & Quality Gates

Development will follow Spec-Driven Development (SDD) principles, utilizing Spec-Kit Plus artifacts (spec.md, plan.md, tasks.md). Prompt History Records (PHRs) will be created for every user interaction. Architectural Decision Records (ADRs) will be suggested for significant decisions. Code reviews, automated testing, and CI/CD will ensure quality.

## Governance
This constitution serves as the foundational agreement for all development within the "Physical AI & Humanoid Robotics Textbook" project. All code contributions, architectural decisions, and feature implementations MUST adhere to these principles. Amendments to this constitution require a formal proposal, team review, and a version bump in accordance with semantic versioning rules. Compliance will be verified during code reviews and project milestones.

**Version**: 1.0.0 | **Ratified**: 2025-11-28 | **Last Amended**: 2025-11-28
