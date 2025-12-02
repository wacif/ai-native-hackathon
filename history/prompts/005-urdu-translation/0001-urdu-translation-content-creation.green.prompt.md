---
id: "0001"
title: "urdu-translation-content-creation"
stage: green
date: 2024-12-02
surface: vscode-copilot
model: claude-opus-4-5
feature: "005-urdu-translation"
branch: main
user: wasi
command: /sp.implement
labels: [translation, urdu, content, documentation]
links:
  spec: specs/005-urdu-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - static/translations/ur/hardware-setup.md
  - static/translations/ur/module1-ros2.md
  - static/translations/ur/module2-simulation.md
  - static/translations/ur/module3-isaac.md
  - static/translations/ur/module4-vla.md
  - static/translations/ur/weekly-schedule.md
  - specs/005-urdu-translation/spec.md
tests:
  - type: manual
    description: "Language toggle UI test for Urdu translations"
    status: pending
---

## Prompt

Create all the Urdu content for the entire Physical AI & Robotics textbook. The English source content is in `/docs/physical-ai/` and translations go to `/static/translations/ur/`.

## Response snapshot

Analyzed existing translations (4 files: intro.md, chapter1.md, chapter2.md, chapter3.md) and English source content (7 chapters). Created 6 new comprehensive Urdu translation files:

1. **hardware-setup.md** - ہارڈویئر سیٹ اپ گائیڈ (Hardware requirements, Jetson Orin setup)
2. **module1-ros2.md** - روبوٹک اعصابی نظام (ROS 2 fundamentals, nodes, topics, services)
3. **module2-simulation.md** - ڈیجیٹل ٹوئن (Gazebo, Unity simulation environments)
4. **module3-isaac.md** - AI-Robot دماغ (NVIDIA Isaac integration)
5. **module4-vla.md** - Voice-to-Action (VLA models, multimodal AI)
6. **weekly-schedule.md** - ہفتہ وار شیڈول اور لیبز (Course schedule and labs)

All translations maintain:
- RTL text direction for Urdu
- Code blocks preserved in English
- Technical terms with English equivalents in parentheses
- Consistent formatting with source markdown

## Outcome

- ✅ Impact: Complete Urdu translation of Physical AI textbook (10 total files)
- 🧪 Tests: Manual UI testing pending
- 📁 Files: 6 new translation files + 1 spec update
- 🔁 Next prompts: TEST-001 end-to-end testing of language toggle
- 🧠 Reflection: Efficient bulk content creation by analyzing source structure first, then creating translations in parallel

## Evaluation notes (flywheel)

- Failure modes observed: None - successful implementation
- Graders run and results (PASS/FAIL): N/A (content task, not code)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add automated translation validation tests
