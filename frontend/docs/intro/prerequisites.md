---
sidebar_position: 2
title: Prerequisites
description: Required knowledge and skills before starting the course
---

# Prerequisites

Before diving into Physical AI and Humanoid Robotics, ensure you have the following prerequisites in place.

## Required Knowledge

### Programming Skills

You should be comfortable with:

- **Python 3.x** - Most robotics code and AI frameworks use Python
- **Basic C++** - ROS 2 has both Python and C++ APIs
- **Command line** - Terminal operations on Linux/Unix systems

### Mathematics Background

Familiarity with:

- **Linear Algebra** - Vectors, matrices, transformations
- **Basic Calculus** - Derivatives, integrals (for understanding control systems)
- **Probability** - For understanding sensor noise and uncertainty

### Computer Science Concepts

Understanding of:

- **Object-Oriented Programming** - Classes, inheritance, polymorphism
- **Concurrent Programming** - Threads, processes, callbacks
- **Basic Networking** - TCP/IP, publish-subscribe patterns

## Hardware Requirements

### Minimum System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 4 cores | 8+ cores |
| RAM | 8 GB | 16+ GB |
| Storage | 50 GB free | 100+ GB SSD |
| GPU | Integrated | NVIDIA GTX 1060+ |

### For NVIDIA Isaac (Module 3)

NVIDIA Isaac Sim requires:

- NVIDIA RTX GPU (RTX 2070 or better)
- NVIDIA Driver 525.60.11 or newer
- 32 GB RAM recommended

## Software Prerequisites

### Operating System

- **Ubuntu 22.04 LTS** (recommended)
- **Windows 11** with WSL2 (alternative)

### Required Software

```bash
# Python 3.10+
python3 --version

# Git
git --version

# Docker (optional but recommended)
docker --version
```

## Self-Assessment Checklist

Before proceeding, make sure you can:

- [ ] Write a Python class with methods and inheritance
- [ ] Navigate the terminal and use basic commands (`cd`, `ls`, `mkdir`)
- [ ] Understand what a matrix multiplication does
- [ ] Clone a Git repository and create branches
- [ ] Read and write JSON/YAML files

## Resources for Refreshing Skills

If you need to brush up on any prerequisites:

### Python
- [Python Tutorial](https://docs.python.org/3/tutorial/)
- [Real Python](https://realpython.com/)

### Linux
- [Linux Command Line Basics](https://ubuntu.com/tutorials/command-line-for-beginners)

### Linear Algebra
- [3Blue1Brown - Essence of Linear Algebra](https://www.3blue1brown.com/topics/linear-algebra)
- [Khan Academy - Linear Algebra](https://www.khanacademy.org/math/linear-algebra)

---

Once you've verified your prerequisites, proceed to [Environment Setup](/docs/intro/setup) to configure your development environment.
