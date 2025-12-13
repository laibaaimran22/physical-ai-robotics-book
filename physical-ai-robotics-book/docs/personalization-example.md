---
title: Personalization Example
sidebar_position: 1
---

# Personalization Example

This page demonstrates how the personalization system works based on your profile.

import PersonalizationEngine from '@site/src/components/Personalization/PersonalizationEngine';
import PersonalizedIntroduction from '@site/src/components/Personalization/PersonalizedIntroduction';

<PersonalizedIntroduction chapterTitle="Personalization Example" />

## Different Content Levels

### Beginner Level Content
<PersonalizationEngine difficulty="beginner" contentType="explanation">
  <div style={{border: '2px solid #28a745', padding: '1rem', borderRadius: '4px', backgroundColor: '#f8f9fa'}}>
    <h3>Beginner Content</h3>
    <p>This content is designed for beginners. We explain concepts in simple terms with lots of examples.</p>
    <p>For example, in robotics, a robot is a machine that can perform tasks automatically. Think of it like a programmable toy that can move and interact with its environment.</p>
  </div>
</PersonalizationEngine>

### Intermediate Level Content
<PersonalizationEngine difficulty="intermediate" contentType="explanation">
  <div style={{border: '2px solid #ffc107', padding: '1rem', borderRadius: '4px', backgroundColor: '#f8f9fa'}}>
    <h3>Intermediate Content</h3>
    <p>This content is designed for intermediate learners. We assume some basic knowledge of robotics concepts.</p>
    <p>Robotic systems typically include sensors for perception, actuators for movement, and controllers for decision-making. Common architectures include deliberative, reactive, and hybrid approaches.</p>
  </div>
</PersonalizationEngine>

### Advanced Level Content
<PersonalizationEngine difficulty="advanced" contentType="explanation">
  <div style={{border: '2px solid #dc3545', padding: '1rem', borderRadius: '4px', backgroundColor: '#f8f9fa'}}>
    <h3>Advanced Content</h3>
    <p>This content is designed for advanced learners with significant experience in robotics.</p>
    <p>Modern robotics leverages machine learning for perception and control, with approaches like reinforcement learning for policy optimization, computer vision for scene understanding, and motion planning algorithms for navigation in complex environments.</p>
  </div>
</PersonalizationEngine>