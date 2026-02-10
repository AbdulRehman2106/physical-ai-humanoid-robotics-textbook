import dotenv from 'dotenv';

// Load environment variables
dotenv.config();

interface Config {
  cohere: {
    apiKey: string;
    model: string;
    temperature: number;
    maxTokens: number;
  };
  server: {
    port: number;
    nodeEnv: string;
  };
  logging: {
    level: string;
  };
  session: {
    timeoutMs: number;
  };
  context: {
    maxTokens: number;
    maxMessagesPerConversation: number;
  };
}

function getEnvVar(key: string, defaultValue?: string): string {
  const value = process.env[key];
  if (!value && !defaultValue) {
    throw new Error(`Environment variable ${key} is required but not set`);
  }
  return value || defaultValue!;
}

function getEnvNumber(key: string, defaultValue: number): number {
  const value = process.env[key];
  if (!value) return defaultValue;
  const parsed = parseInt(value, 10);
  if (isNaN(parsed)) {
    throw new Error(`Environment variable ${key} must be a valid number`);
  }
  return parsed;
}

export const config: Config = {
  cohere: {
    apiKey: getEnvVar('COHERE_API_KEY'),
    model: getEnvVar('COHERE_MODEL', 'command-r'),
    temperature: parseFloat(getEnvVar('COHERE_TEMPERATURE', '0.7')),
    maxTokens: getEnvNumber('COHERE_MAX_TOKENS', 1000),
  },
  server: {
    port: getEnvNumber('PORT', 3000),
    nodeEnv: getEnvVar('NODE_ENV', 'development'),
  },
  logging: {
    level: getEnvVar('LOG_LEVEL', 'info'),
  },
  session: {
    timeoutMs: getEnvNumber('SESSION_TIMEOUT_MS', 1800000), // 30 minutes
  },
  context: {
    maxTokens: getEnvNumber('MAX_CONTEXT_TOKENS', 100000),
    maxMessagesPerConversation: getEnvNumber('MAX_MESSAGES_PER_CONVERSATION', 1000),
  },
};

export function validateConfig(): void {
  if (!config.cohere.apiKey || config.cohere.apiKey === 'your_cohere_api_key_here') {
    throw new Error(
      'COHERE_API_KEY environment variable is not set or is using the placeholder value. ' +
      'Please set a valid Cohere API key in your .env file.'
    );
  }

  if (config.server.port < 1 || config.server.port > 65535) {
    throw new Error('PORT must be between 1 and 65535');
  }

  if (config.cohere.temperature < 0 || config.cohere.temperature > 1) {
    throw new Error('COHERE_TEMPERATURE must be between 0 and 1');
  }
}
