---
sidebar_position: 2
title: شرائط
description: کورس شروع کرنے سے پہلے مطلوبہ علم اور مہارتیں
---

# شرائط

Physical AI اور Humanoid Robotics میں غوطہ لگانے سے پہلے، یقینی بنائیں کہ آپ کے پاس درج ذیل شرائط موجود ہیں۔

## مطلوبہ علم

### Programming کی مہارتیں

آپ کو ان میں مہارت ہونی چاہیے:

- **Python 3.x** - زیادہ تر robotics code اور AI frameworks Python استعمال کرتے ہیں
- **بنیادی C++** - ROS 2 میں Python اور C++ دونوں APIs ہیں
- **Command line** - Linux/Unix systems پر Terminal operations

### ریاضی کا پس منظر

ان سے واقفیت:

- **Linear Algebra** - Vectors، matrices، transformations
- **بنیادی Calculus** - Derivatives، integrals (control systems کو سمجھنے کے لیے)
- **Probability** - sensor noise اور uncertainty کو سمجھنے کے لیے

### Computer Science کے تصورات

ان کی سمجھ:

- **Object-Oriented Programming** - Classes، inheritance، polymorphism
- **Concurrent Programming** - Threads، processes، callbacks
- **بنیادی Networking** - TCP/IP، publish-subscribe patterns

## Hardware کی ضروریات

### کم از کم سسٹم کی ضروریات

| Component | کم از کم | تجویز کردہ |
|-----------|---------|-------------|
| CPU | 4 cores | 8+ cores |
| RAM | 8 GB | 16+ GB |
| Storage | 50 GB خالی | 100+ GB SSD |
| GPU | Integrated | NVIDIA GTX 1060+ |

### NVIDIA Isaac کے لیے (Module 3)

NVIDIA Isaac Sim کو درکار ہے:

- NVIDIA RTX GPU (RTX 2070 یا بہتر)
- NVIDIA Driver 525.60.11 یا نیا
- 32 GB RAM تجویز کردہ

## Software کی شرائط

### Operating System

- **Ubuntu 22.04 LTS** (تجویز کردہ)
- **Windows 11** WSL2 کے ساتھ (متبادل)

### مطلوبہ Software

```bash
# Python 3.10+
python3 --version

# Git
git --version

# Docker (اختیاری لیکن تجویز کردہ)
docker --version
```

## خود تشخیص چیک لسٹ

آگے بڑھنے سے پہلے، یقینی بنائیں کہ آپ یہ کر سکتے ہیں:

- [ ] methods اور inheritance کے ساتھ Python class لکھیں
- [ ] Terminal میں navigate کریں اور بنیادی commands استعمال کریں (`cd`، `ls`، `mkdir`)
- [ ] سمجھیں کہ matrix multiplication کیا کرتی ہے
- [ ] Git repository clone کریں اور branches بنائیں
- [ ] JSON/YAML files پڑھیں اور لکھیں

## مہارتیں تازہ کرنے کے وسائل

اگر آپ کو کسی شرط پر نظرثانی کی ضرورت ہے:

### Python
- [Python Tutorial](https://docs.python.org/3/tutorial/)
- [Real Python](https://realpython.com/)

### Linux
- [Linux Command Line Basics](https://ubuntu.com/tutorials/command-line-for-beginners)

### Linear Algebra
- [3Blue1Brown - Essence of Linear Algebra](https://www.3blue1brown.com/topics/linear-algebra)
- [Khan Academy - Linear Algebra](https://www.khanacademy.org/math/linear-algebra)

---

اپنی شرائط کی تصدیق کرنے کے بعد، اپنا development environment configure کرنے کے لیے [Environment Setup](/docs/intro/setup) پر جائیں۔
