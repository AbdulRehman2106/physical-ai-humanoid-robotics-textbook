// Use environment variable in production, fallback to localhost in development
window.BACKEND_URL = window.location.hostname === 'localhost'
  ? 'http://localhost:8001'
  : 'https://your-backend-url.railway.app';
