import { detectProhibitedTopic, getRefusalMessage } from '../agent/prompts/SystemPrompts';
import { logger } from '../utils/logger';

/**
 * Safety service for enforcing safety boundaries
 *
 * Detects prohibited topics and generates appropriate refusal messages
 * with helpful redirection.
 */
export class SafetyService {
  /**
   * Detect if input contains prohibited topics
   *
   * @param input - User input to check
   * @returns Prohibited topic type or null
   */
  detectProhibitedTopic(input: string): 'medical' | 'legal' | 'financial' | 'autonomous' | null {
    const topic = detectProhibitedTopic(input);

    if (topic) {
      logger.warn('Prohibited topic detected', {
        topic,
        inputPreview: input.substring(0, 100),
      });
    }

    return topic;
  }

  /**
   * Generate refusal message for prohibited topic
   *
   * @param topic - Prohibited topic type
   * @returns User-friendly refusal message with redirection
   */
  generateRefusalMessage(topic: 'medical' | 'legal' | 'financial' | 'autonomous'): string {
    const message = getRefusalMessage(topic);

    logger.info('Refusal message generated', { topic });

    return message;
  }

  /**
   * Check if response contains uncertainty patterns
   *
   * @param response - Assistant response to check
   * @returns True if uncertainty is admitted
   */
  detectUncertainty(response: string): boolean {
    const uncertaintyPatterns = [
      "i'm not certain",
      "i'm not sure",
      "i don't know",
      "i don't have enough information",
      "my knowledge is current as of",
      "this is outside my",
      "i cannot be certain",
      "i'm uncertain",
    ];

    const lowerResponse = response.toLowerCase();
    const hasUncertainty = uncertaintyPatterns.some(pattern =>
      lowerResponse.includes(pattern)
    );

    if (hasUncertainty) {
      logger.debug('Uncertainty detected in response', {
        responsePreview: response.substring(0, 100),
      });
    }

    return hasUncertainty;
  }

  /**
   * Pre-process user input for safety checks
   *
   * @param input - User input
   * @returns Refusal message if prohibited, null otherwise
   */
  preProcessInput(input: string): string | null {
    const prohibitedTopic = this.detectProhibitedTopic(input);

    if (prohibitedTopic) {
      return this.generateRefusalMessage(prohibitedTopic);
    }

    return null;
  }

  /**
   * Post-process assistant response for safety validation
   *
   * @param response - Assistant response
   * @returns Validation result with warnings
   */
  postProcessResponse(response: string): {
    isValid: boolean;
    warnings: string[];
    hasUncertainty: boolean;
  } {
    const warnings: string[] = [];
    const hasUncertainty = this.detectUncertainty(response);

    // Check for potential safety violations in response
    const prohibitedInResponse = this.detectProhibitedTopic(response);
    if (prohibitedInResponse) {
      warnings.push(`Response may contain ${prohibitedInResponse} advice`);
    }

    // Check if response is too short (might indicate error)
    if (response.length < 10) {
      warnings.push('Response is unusually short');
    }

    // Check if response is too long (might indicate rambling)
    if (response.length > 5000) {
      warnings.push('Response is very long');
    }

    const isValid = warnings.length === 0;

    if (warnings.length > 0) {
      logger.warn('Response validation warnings', {
        warnings,
        responsePreview: response.substring(0, 100),
      });
    }

    return {
      isValid,
      warnings,
      hasUncertainty,
    };
  }

  /**
   * Log safety event for monitoring
   *
   * @param event - Safety event type
   * @param details - Event details
   */
  logSafetyEvent(
    event: 'refusal' | 'uncertainty' | 'warning',
    details: Record<string, any>
  ): void {
    logger.info('Safety event', {
      event,
      ...details,
      timestamp: new Date().toISOString(),
    });
  }
}
