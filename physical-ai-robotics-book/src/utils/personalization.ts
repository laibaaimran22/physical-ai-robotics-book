// Personalization utilities for the frontend

export interface UserProfile {
  id: number;
  email: string;
  full_name?: string;
  software_background_level?: string;
  hardware_background_level?: string;
  preferred_languages?: string;
  learning_goals?: string;
  is_active: boolean;
  is_admin: boolean;
}

export interface PersonalizationData {
  chapter_id: string;
  difficulty_level: string;
  learning_objectives: string[];
  recommended_prerequisites: string[];
  content_modifications: {
    explanation_depth: string;
    example_complexity: string;
    mathematical_formalism: string;
    [key: string]: any; // For additional properties like preferred_languages
  };
  practice_exercises: Array<{
    type: string;
    description: string;
  }>;
  additional_resources: string[];
}

/**
 * Determine the user's overall experience level based on their background
 */
export function getUserExperienceLevel(userProfile: UserProfile): string {
  const softwareLevel = userProfile.software_background_level || 'beginner';
  const hardwareLevel = userProfile.hardware_background_level || 'beginner';

  // Return the higher of the two levels
  if (softwareLevel === 'advanced' || hardwareLevel === 'advanced') {
    return 'advanced';
  } else if (softwareLevel === 'intermediate' || hardwareLevel === 'intermediate') {
    return 'intermediate';
  } else {
    return 'beginner';
  }
}

/**
 * Get personalized learning objectives based on user profile
 */
export function getLearningObjectives(userProfile: UserProfile, chapterTitle: string): string[] {
  const experienceLevel = getUserExperienceLevel(userProfile);
  const objectives: string[] = [];

  switch (experienceLevel) {
    case 'beginner':
      objectives.push(
        `Understand fundamental concepts of ${chapterTitle}`,
        `Learn basic terminology and principles`,
        `Complete simple exercises to reinforce learning`
      );
      break;
    case 'intermediate':
      objectives.push(
        `Apply concepts from ${chapterTitle} to practical scenarios`,
        `Implement moderate complexity solutions`,
        `Understand core algorithms and methodologies`
      );
      break;
    case 'advanced':
      objectives.push(
        `Master advanced implementation techniques in ${chapterTitle}`,
        `Design custom solutions for complex problems`,
        `Explore cutting-edge research in this area`
      );
      break;
  }

  return objectives;
}

/**
 * Get personalized difficulty level based on user profile
 */
export function getDifficultyLevel(userProfile: UserProfile): 'beginner' | 'intermediate' | 'advanced' {
  const softwareLevel = userProfile.software_background_level || 'beginner';
  const hardwareLevel = userProfile.hardware_background_level || 'beginner';

  if (softwareLevel === 'advanced' || hardwareLevel === 'advanced') {
    return 'advanced';
  } else if (softwareLevel === 'intermediate' || hardwareLevel === 'intermediate') {
    return 'intermediate';
  } else {
    return 'beginner';
  }
}

/**
 * Get personalized content modifications based on user profile
 */
export function getContentModifications(userProfile: UserProfile): {
  explanation_depth: string;
  example_complexity: string;
  mathematical_formalism: string;
  preferred_languages?: string;
} {
  const experienceLevel = getDifficultyLevel(userProfile);

  let explanation_depth, example_complexity, mathematical_formalism;

  switch (experienceLevel) {
    case 'advanced':
      explanation_depth = 'detailed';
      example_complexity = 'complex';
      mathematical_formalism = 'extensive';
      break;
    case 'intermediate':
      explanation_depth = 'moderate';
      example_complexity = 'moderate';
      mathematical_formalism = 'some';
      break;
    case 'beginner':
    default:
      explanation_depth = 'basic';
      example_complexity = 'simple';
      mathematical_formalism = 'minimal';
      break;
  }

  const modifications: {
    explanation_depth: string;
    example_complexity: string;
    mathematical_formalism: string;
    preferred_languages?: string;
  } = {
    explanation_depth,
    example_complexity,
    mathematical_formalism
  };

  // Add preferred languages if specified
  if (userProfile.preferred_languages) {
    modifications.preferred_languages = userProfile.preferred_languages;
  }

  return modifications;
}

/**
 * Get personalized exercises based on user profile
 */
export function getPersonalizedExercises(userProfile: UserProfile, chapterTitle: string): Array<{type: string, description: string}> {
  const experienceLevel = getDifficultyLevel(userProfile);
  const exercises: Array<{type: string, description: string}> = [];

  switch (experienceLevel) {
    case 'beginner':
      exercises.push(
        { type: 'conceptual', description: `Explain the basic concepts of ${chapterTitle} in your own words` },
        { type: 'practical', description: `Complete the provided simple coding exercise for ${chapterTitle}` }
      );
      break;
    case 'intermediate':
      exercises.push(
        { type: 'implementation', description: `Implement a moderate complexity solution related to ${chapterTitle}` },
        { type: 'analysis', description: `Analyze the provided example and identify key components of ${chapterTitle}` }
      );
      break;
    case 'advanced':
      exercises.push(
        { type: 'research', description: `Design a novel solution to an advanced problem in ${chapterTitle}` },
        { type: 'optimization', description: `Optimize the provided implementation for performance in ${chapterTitle}` }
      );
      break;
  }

  return exercises;
}

/**
 * Format user background for display
 */
export function formatUserBackground(userProfile: UserProfile): string {
  const parts: string[] = [];

  if (userProfile.software_background_level) {
    parts.push(`Software: ${userProfile.software_background_level}`);
  }

  if (userProfile.hardware_background_level) {
    parts.push(`Hardware: ${userProfile.hardware_background_level}`);
  }

  if (userProfile.preferred_languages) {
    parts.push(`Languages: ${userProfile.preferred_languages}`);
  }

  if (userProfile.learning_goals) {
    parts.push(`Goals: ${userProfile.learning_goals}`);
  }

  return parts.join(', ') || 'No background information specified';
}