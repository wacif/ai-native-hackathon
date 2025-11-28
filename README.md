# Physical AI & Humanoid Robotics Textbook

This repository contains the source for an AI-native textbook on Physical AI & Humanoid Robotics, developed using Claude Code and Spec-Kit Plus. The project aims to provide a comprehensive learning resource for students to understand, simulate, and deploy humanoid robots.

## Project Deliverables

1.  **AI/Spec-Driven Book Creation**: The textbook is built using Docusaurus and deployed to GitHub Pages, following a Spec-Driven Development (SDD) approach.
2.  **Integrated RAG Chatbot**: An embedded Retrieval-Augmented Generation (RAG) chatbot provides interactive learning, answering questions about the book's content, including selected text. It leverages OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud.
3.  **Optional Bonus Features**:
    *   Reusable intelligence via Claude Code Subagents and Agent Skills.
    *   User Signup and Signin with [Better-Auth](https://www.better-auth.com/) for personalized content.
    *   Content personalization and Urdu translation for logged-in users.

## Course Overview

The textbook covers:
*   ROS 2 (Robot Operating System) Fundamentals
*   Robot Simulation with Gazebo & Unity
*   NVIDIA Isaac Platform for advanced perception and training
*   Vision-Language-Action (VLA) for integrating LLMs and Robotics

## Development Philosophy

This project adheres to the principles outlined in the `.specify/memory/constitution.md`, emphasizing AI/Spec-Driven Development, modular architecture, user-centric design, and continuous integration.

---

# Website (Docusaurus)

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
