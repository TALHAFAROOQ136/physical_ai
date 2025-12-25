"""Seed data script for modules and chapters.

Run with: python -m scripts.seed_data
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from src.lib.database import async_session_maker, engine
from src.models.module import Module
from src.models.chapter import Chapter


MODULES_DATA = [
    {
        "id": "intro",
        "title": "Introduction",
        "description": "Welcome to Physical AI & Humanoid Robotics. Learn the fundamentals of building intelligent robots.",
        "order_sequence": 0,
        "objectives": [
            "Understand the course structure and prerequisites",
            "Set up your development environment",
            "Learn the key concepts of physical AI",
        ],
        "icon": "rocket",
    },
    {
        "id": "ros2",
        "title": "Module 1: ROS 2 - The Robotic Nervous System",
        "description": "Learn ROS 2 middleware architecture for robot control, including nodes, topics, services, and actions.",
        "order_sequence": 1,
        "objectives": [
            "Understand ROS 2 architecture and concepts",
            "Create and manage ROS 2 nodes",
            "Work with topics, services, and actions",
            "Define robot descriptions with URDF",
        ],
        "icon": "network",
    },
    {
        "id": "simulation",
        "title": "Module 2: Simulation - The Digital Twin",
        "description": "Build physics simulations with Gazebo and Unity to test robot behaviors safely.",
        "order_sequence": 2,
        "objectives": [
            "Set up Gazebo simulation environments",
            "Create robot models and worlds",
            "Integrate Unity for advanced visualization",
            "Test robot behaviors in simulation",
        ],
        "icon": "cube",
    },
    {
        "id": "isaac",
        "title": "Module 3: NVIDIA Isaac - The AI-Robot Brain",
        "description": "Leverage NVIDIA Isaac for photorealistic simulation and GPU-accelerated perception.",
        "order_sequence": 3,
        "objectives": [
            "Set up Isaac Sim environment",
            "Create photorealistic simulations",
            "Implement perception pipelines",
            "Train robots with reinforcement learning",
        ],
        "icon": "gpu",
    },
    {
        "id": "vla",
        "title": "Module 4: VLA - Vision-Language-Action",
        "description": "Integrate voice commands and LLMs for intuitive robot control through natural language.",
        "order_sequence": 4,
        "objectives": [
            "Implement voice interfaces for robot control",
            "Integrate LLMs for task planning",
            "Build end-to-end VLA pipelines",
            "Create natural language robot interactions",
        ],
        "icon": "brain",
    },
    {
        "id": "capstone",
        "title": "Capstone Project: Build Your Humanoid",
        "description": "Complete capstone project integrating all skills to build a fully functional humanoid robot system.",
        "order_sequence": 5,
        "objectives": [
            "Design a complete humanoid robot system",
            "Integrate ROS 2, simulation, and AI components",
            "Implement real-world deployment strategies",
            "Present and document your project",
        ],
        "icon": "robot",
    },
]

CHAPTERS_DATA = [
    # Introduction chapters
    {
        "id": "intro-index",
        "module_id": "intro",
        "title": "Welcome to Physical AI",
        "slug": "intro",
        "order_sequence": 0,
        "estimated_time_min": 10,
        "objectives": ["Understand what physical AI is", "Learn about course goals"],
        "prerequisites": [],
        "difficulty": "beginner",
    },
    {
        "id": "intro-prerequisites",
        "module_id": "intro",
        "title": "Prerequisites",
        "slug": "intro/prerequisites",
        "order_sequence": 1,
        "estimated_time_min": 15,
        "objectives": ["Review required knowledge", "Install necessary software"],
        "prerequisites": [],
        "difficulty": "beginner",
    },
    {
        "id": "intro-setup",
        "module_id": "intro",
        "title": "Environment Setup",
        "slug": "intro/setup",
        "order_sequence": 2,
        "estimated_time_min": 30,
        "objectives": ["Set up ROS 2 environment", "Configure development tools"],
        "prerequisites": ["intro-prerequisites"],
        "difficulty": "beginner",
    },
    # ROS 2 Module chapters
    {
        "id": "ros2-index",
        "module_id": "ros2",
        "title": "Introduction to ROS 2",
        "slug": "module-1-ros2",
        "order_sequence": 0,
        "estimated_time_min": 20,
        "objectives": ["Understand ROS 2 architecture", "Learn key concepts"],
        "prerequisites": ["intro-setup"],
        "difficulty": "beginner",
    },
    {
        "id": "ros2-nodes-topics",
        "module_id": "ros2",
        "title": "Nodes and Topics",
        "slug": "module-1-ros2/nodes-and-topics",
        "order_sequence": 1,
        "estimated_time_min": 45,
        "objectives": ["Create ROS 2 nodes", "Publish and subscribe to topics"],
        "prerequisites": ["ros2-index"],
        "difficulty": "beginner",
    },
    {
        "id": "ros2-services-actions",
        "module_id": "ros2",
        "title": "Services and Actions",
        "slug": "module-1-ros2/services-and-actions",
        "order_sequence": 2,
        "estimated_time_min": 45,
        "objectives": ["Implement ROS 2 services", "Create action servers and clients"],
        "prerequisites": ["ros2-nodes-topics"],
        "difficulty": "intermediate",
    },
    {
        "id": "ros2-urdf",
        "module_id": "ros2",
        "title": "URDF Basics",
        "slug": "module-1-ros2/urdf-basics",
        "order_sequence": 3,
        "estimated_time_min": 60,
        "objectives": ["Write URDF robot descriptions", "Visualize robots in RViz"],
        "prerequisites": ["ros2-nodes-topics"],
        "difficulty": "intermediate",
    },
    # Simulation Module chapters
    {
        "id": "sim-index",
        "module_id": "simulation",
        "title": "Introduction to Simulation",
        "slug": "module-2-simulation",
        "order_sequence": 0,
        "estimated_time_min": 15,
        "objectives": ["Understand simulation importance", "Compare simulation tools"],
        "prerequisites": ["ros2-urdf"],
        "difficulty": "beginner",
    },
    {
        "id": "sim-gazebo",
        "module_id": "simulation",
        "title": "Gazebo Basics",
        "slug": "module-2-simulation/gazebo-basics",
        "order_sequence": 1,
        "estimated_time_min": 60,
        "objectives": ["Set up Gazebo", "Create simulation worlds"],
        "prerequisites": ["sim-index"],
        "difficulty": "intermediate",
    },
    {
        "id": "sim-unity",
        "module_id": "simulation",
        "title": "Unity Integration",
        "slug": "module-2-simulation/unity-integration",
        "order_sequence": 2,
        "estimated_time_min": 45,
        "objectives": ["Integrate Unity with ROS 2", "Create visual simulations"],
        "prerequisites": ["sim-gazebo"],
        "difficulty": "intermediate",
    },
    # Isaac Module chapters
    {
        "id": "isaac-index",
        "module_id": "isaac",
        "title": "Introduction to NVIDIA Isaac",
        "slug": "module-3-isaac",
        "order_sequence": 0,
        "estimated_time_min": 20,
        "objectives": ["Understand Isaac platform", "Learn Isaac capabilities"],
        "prerequisites": ["sim-gazebo"],
        "difficulty": "intermediate",
    },
    {
        "id": "isaac-setup",
        "module_id": "isaac",
        "title": "Isaac Sim Setup",
        "slug": "module-3-isaac/isaac-sim-setup",
        "order_sequence": 1,
        "estimated_time_min": 45,
        "objectives": ["Install Isaac Sim", "Configure GPU acceleration"],
        "prerequisites": ["isaac-index"],
        "difficulty": "intermediate",
    },
    {
        "id": "isaac-perception",
        "module_id": "isaac",
        "title": "Perception Pipelines",
        "slug": "module-3-isaac/perception-pipelines",
        "order_sequence": 2,
        "estimated_time_min": 60,
        "objectives": ["Build perception pipelines", "Process sensor data"],
        "prerequisites": ["isaac-setup"],
        "difficulty": "advanced",
    },
    # VLA Module chapters
    {
        "id": "vla-index",
        "module_id": "vla",
        "title": "Introduction to VLA",
        "slug": "module-4-vla",
        "order_sequence": 0,
        "estimated_time_min": 20,
        "objectives": ["Understand VLA concepts", "Learn about multimodal AI"],
        "prerequisites": ["isaac-perception"],
        "difficulty": "intermediate",
    },
    {
        "id": "vla-voice",
        "module_id": "vla",
        "title": "Voice Interfaces",
        "slug": "module-4-vla/voice-interfaces",
        "order_sequence": 1,
        "estimated_time_min": 45,
        "objectives": ["Implement speech recognition", "Create voice commands"],
        "prerequisites": ["vla-index"],
        "difficulty": "intermediate",
    },
    {
        "id": "vla-llm",
        "module_id": "vla",
        "title": "LLM Planning",
        "slug": "module-4-vla/llm-planning",
        "order_sequence": 2,
        "estimated_time_min": 60,
        "objectives": ["Integrate LLMs for planning", "Build action pipelines"],
        "prerequisites": ["vla-voice"],
        "difficulty": "advanced",
    },
    # Capstone chapters
    {
        "id": "capstone-index",
        "module_id": "capstone",
        "title": "Capstone Overview",
        "slug": "capstone",
        "order_sequence": 0,
        "estimated_time_min": 15,
        "objectives": ["Understand project requirements", "Plan your capstone"],
        "prerequisites": ["vla-llm"],
        "difficulty": "advanced",
    },
    {
        "id": "capstone-m1",
        "module_id": "capstone",
        "title": "Milestone 1: Design",
        "slug": "capstone/milestone-1",
        "order_sequence": 1,
        "estimated_time_min": 120,
        "objectives": ["Design system architecture", "Define requirements"],
        "prerequisites": ["capstone-index"],
        "difficulty": "advanced",
    },
    {
        "id": "capstone-m2",
        "module_id": "capstone",
        "title": "Milestone 2: ROS 2 Foundation",
        "slug": "capstone/milestone-2",
        "order_sequence": 2,
        "estimated_time_min": 180,
        "objectives": ["Implement ROS 2 nodes", "Create robot description"],
        "prerequisites": ["capstone-m1"],
        "difficulty": "advanced",
    },
    {
        "id": "capstone-m3",
        "module_id": "capstone",
        "title": "Milestone 3: Simulation",
        "slug": "capstone/milestone-3",
        "order_sequence": 3,
        "estimated_time_min": 180,
        "objectives": ["Build simulation environment", "Test behaviors"],
        "prerequisites": ["capstone-m2"],
        "difficulty": "advanced",
    },
    {
        "id": "capstone-m4",
        "module_id": "capstone",
        "title": "Milestone 4: AI Integration",
        "slug": "capstone/milestone-4",
        "order_sequence": 4,
        "estimated_time_min": 180,
        "objectives": ["Integrate perception", "Add VLA capabilities"],
        "prerequisites": ["capstone-m3"],
        "difficulty": "advanced",
    },
    {
        "id": "capstone-m5",
        "module_id": "capstone",
        "title": "Milestone 5: Presentation",
        "slug": "capstone/milestone-5",
        "order_sequence": 5,
        "estimated_time_min": 60,
        "objectives": ["Document your project", "Present your work"],
        "prerequisites": ["capstone-m4"],
        "difficulty": "advanced",
    },
]


async def seed_modules(session: AsyncSession) -> None:
    """Seed modules into the database."""
    for module_data in MODULES_DATA:
        # Check if module exists
        result = await session.execute(
            select(Module).where(Module.id == module_data["id"])
        )
        existing = result.scalar_one_or_none()

        if existing:
            print(f"  Module '{module_data['id']}' already exists, skipping...")
            continue

        module = Module(**module_data)
        session.add(module)
        print(f"  Created module: {module_data['title']}")

    await session.commit()


async def seed_chapters(session: AsyncSession) -> None:
    """Seed chapters into the database."""
    for chapter_data in CHAPTERS_DATA:
        # Check if chapter exists
        result = await session.execute(
            select(Chapter).where(Chapter.id == chapter_data["id"])
        )
        existing = result.scalar_one_or_none()

        if existing:
            print(f"  Chapter '{chapter_data['id']}' already exists, skipping...")
            continue

        chapter = Chapter(**chapter_data)
        session.add(chapter)
        print(f"  Created chapter: {chapter_data['title']}")

    await session.commit()


async def main() -> None:
    """Run the seed data script."""
    print("Starting seed data script...")
    print()

    async with async_session_maker() as session:
        print("Seeding modules...")
        await seed_modules(session)
        print()

        print("Seeding chapters...")
        await seed_chapters(session)
        print()

    await engine.dispose()
    print("Seed data complete!")


if __name__ == "__main__":
    asyncio.run(main())
