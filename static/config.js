// Backend URL configuration
// Development: localhost:8001
// Production: Hugging Face Spaces
window.BACKEND_URL = window.location.hostname === 'localhost'
  ? 'http://localhost:8001'
  : 'https://abdul18-rag-chatbot.hf.space';
