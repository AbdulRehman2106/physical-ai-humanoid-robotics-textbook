@echo off
echo Testing Physical AI RAG Backend...
echo.

echo [1/3] Testing Health Endpoint...
curl -s http://localhost:8001/api/health
echo.
echo.

echo [2/3] Testing Root Endpoint...
curl -s http://localhost:8001/
echo.
echo.

echo [3/3] Testing Chat Query...
curl -X POST http://localhost:8001/api/chat/query -H "Content-Type: application/json" -d "{\"query\":\"What is Physical AI?\"}"
echo.
echo.

echo ========================================
echo All tests complete!
echo.
echo Open Swagger UI: http://localhost:8001/docs
echo.
pause
