import { createApp } from '../src/api/server';

// Ensure environment variables are available
console.log('Vercel serverless function starting...', {
  hasApiKey: !!process.env.COHERE_API_KEY,
  nodeEnv: process.env.NODE_ENV
});

// Export the Express app as a Vercel serverless function
export default createApp();
