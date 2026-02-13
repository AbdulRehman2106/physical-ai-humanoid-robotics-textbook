# 🚀 Backend Server - Quick Start Guide

## ✅ Requirements Installed

All dependencies are installed and ready:
- FastAPI 0.128.4
- Cohere 5.20.1
- Qdrant Client 1.16.1
- Psycopg2 2.9.11
- Pydantic 2.12.5
- Uvicorn 0.40.0

## 🎯 Start Backend Server

### Option 1: Using Batch File (Windows)
```bash
start-backend.bat
```

### Option 2: Using Command Line
```bash
cd backend
uvicorn app.main:app --reload --port 8000
```

### Option 3: Using Python
```bash
cd backend
python -m uvicorn app.main:app --reload --port 8000
```

## 📡 Access Points

Once started, the backend will be available at:

- **API Base**: http://localhost:8000
- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc
- **Health Check**: http://localhost:8000/api/health

## 🧪 Test the API

### Health Check
```bash
curl http://localhost:8000/api/health
```

### Test Query
```bash
curl -X POST http://localhost:8000/api/chat/query \
  -H "Content-Type: application/json" \
  -d "{\"query\":\"What is Physical AI?\"}"
```

### Test with PowerShell
```powershell
Invoke-RestMethod -Uri "http://localhost:8000/api/health" -Method Get
```

## 📊 What's Running

When the server starts, you'll see:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

## 🎯 Next Steps

1. **Start the server** using one of the methods above
2. **Open browser** and go to http://localhost:8000/docs
3. **Test the API** using the Swagger UI
4. **Try a query**: Use the `/api/chat/query` endpoint

## 🔧 Configuration

Your backend is configured with:
- ✅ Cohere API Key: Active
- ✅ Qdrant Collection: physical_ai_textbook (328 vectors)
- ✅ Neon Postgres: Connected
- ✅ Model: command-r-08-2024

## 💡 Example Queries

Try these in the Swagger UI:

1. **Basic Query**:
```json
{
  "query": "What is Physical AI?"
}
```

2. **With Filters**:
```json
{
  "query": "Explain ROS 2",
  "filters": {"chapter": 3}
}
```

3. **With Context**:
```json
{
  "query": "Explain this concept",
  "selected_text": "Physical AI systems combine AI with physical embodiment"
}
```

## 🎉 Ready!

Your backend is ready to start. Run the command and test it!
